// telemetry.cpp — F11. Binary telemetry broadcaster.
//
// PLAN.md §5.1, §3. A FreeRTOS task on core 0 wakes at TELEMETRY_HZ
// (100 Hz), snapshots the relevant fields out of shared::g into a packed
// 52-byte little-endian frame, and broadcasts it on the AsyncWebSocket
// owned by net.cpp (F10). The simulator's S2 "Device" data source will
// parse these frames directly.
//
// Why a packed struct + memcpy instead of explicit byte serialization?
//   - ESP32 (Xtensa LX6) is little-endian and the wire format is also
//     little-endian, so member layout matches the wire layout exactly
//     once we suppress padding.
//   - It keeps the producer and the (future) consumer trivially in sync:
//     a `static_assert(sizeof(Frame) == FRAME_SIZE)` catches any drift
//     at compile time.
//
// Why broadcast only when ws->count() > 0?
//   - AsyncWebSocket's binaryAll() is cheap when there are no clients,
//     but allocating + copying the buffer still costs us. Skipping the
//     work entirely on a disconnected bot keeps the core-0 task quiet
//     and the heap calm.
//
// Why only call cleanupClients() at ~1 Hz, not every tick?
//   - It walks the client list and frees dead entries. At 100 Hz it would
//     be wasted work; at 1 Hz any disconnect is reaped well before the
//     next reconnect attempt.
//
// Backpressure: AsyncWebSocket queues per-client. If a client falls
// behind, queued frames pile up in RAM. We could check client->canSend()
// and drop frames per-client, but for a single laptop on the same LAN
// this has not been observed to be a problem. Revisit if heap pressure
// shows up during tuning sessions.

#include "telemetry.h"

#include <Arduino.h>
#include <atomic>
#include <string.h>

#include "config.h"
#include "motors.h"
#include "shared_state.h"
#include "uartbus.h"

namespace telemetry {

namespace {

// Frame struct now lives in telemetry.h (hoisted out so the coproc can
// also include it for status-page rendering — coproc decodes specific
// fields out of the byte stream rather than just relaying opaquely).

bool s_started = false;

// Tx gate. Set from cmdrx (rx task context) via setEnabled; read from
// the telemetry task. std::atomic for a clean cross-task read/write
// without dragging in a mutex for a single boolean.
std::atomic<bool> s_enabled{true};

void telemetryTask(void* /*arg*/) {
  const TickType_t period = pdMS_TO_TICKS(1000 / TELEMETRY_HZ);
  TickType_t lastWake = xTaskGetTickCount();

  // Cleanup pacing: roughly once per second regardless of TELEMETRY_HZ.
  uint32_t cleanupCounter = 0;
  const uint32_t cleanupEvery = TELEMETRY_HZ; // ~1 Hz

  // cleanupCounter / cleanupEvery were used to pace ws->cleanupClients();
  // no longer needed now that the WS lives on coproc.
  (void)cleanupCounter;
  (void)cleanupEvery;

  // Per-second tx-rate instrumentation. Matches the heartbeat shape on
  // the coproc side (coproc/src/main.cpp) so the two sides line up
  // 1:1 when diagnosing a gap. Two metrics:
  //   * frames sent in the interval / interval ms = actual tx rate
  //   * max interval between vTaskDelayUntil wakeups = how long the
  //     scheduler kept this task off the CPU at most. With period =
  //     16 ms (60 Hz), the healthy value is ~16 ms; a jump to 7000 ms
  //     means a higher-priority task preempted us continuously for
  //     7 s, and that's why telemetry tx slowed.
  uint32_t framesSinceLog = 0;
  uint32_t maxIntervalMs  = 0;
  uint32_t prevWakeMs     = 0;
  uint32_t lastLogMs      = millis();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    // Tick-time instrumentation. nowMs is the wall-clock moment we
    // actually got the CPU after this wake; if the scheduler held us
    // for far longer than `period`, the delta will show it.
    const uint32_t nowMs = millis();
    if (prevWakeMs != 0) {
      const uint32_t interval = nowMs - prevWakeMs;
      if (interval > maxIntervalMs) maxIntervalMs = interval;
    }
    prevWakeMs = nowMs;
    framesSinceLog++;

    Frame f;
    f.magic     = MAGIC;
    f.seq       = shared::g.seq.fetch_add(1, std::memory_order_relaxed);
    f.t         = static_cast<float>(millis()) * 1e-3f;
    f.theta     = shared::g.theta.load(std::memory_order_relaxed);
    f.thetaDot  = shared::g.thetaDot.load(std::memory_order_relaxed);
    f.xDot      = shared::g.xDotEst.load(std::memory_order_relaxed);
    f.thetaSet  = shared::g.thetaSet.load(std::memory_order_relaxed);
    f.vWheelCmd = shared::g.vWheelCmd.load(std::memory_order_relaxed);
    f.outerP    = shared::g.outerP.load(std::memory_order_relaxed);
    f.outerI    = shared::g.outerI.load(std::memory_order_relaxed);
    f.outerD    = shared::g.outerD.load(std::memory_order_relaxed);
    f.vBat      = shared::g.vBat.load(std::memory_order_relaxed);
    f.wheelActualMps = motors::wheelActualMps();
    f.accelX    = shared::g.accelX.load(std::memory_order_relaxed);
    f.gyroZ     = shared::g.gyroZ.load(std::memory_order_relaxed);
    f.flags     = shared::g.flags.load(std::memory_order_relaxed);
    f.reserved  = 0;
    f.targetV   = shared::g.targetV.load(std::memory_order_relaxed);
    // Per-wheel cmd is published by controller.cpp after turn split +
    // ±vMaxWheel saturation, so the L/R lines here track exactly what
    // motors::setWheelVelocity received this tick. Per-wheel actual is
    // pulled from the motors layer (LEDC backend: equals last commanded
    // step rate scaled to m/s with mounting invert undone).
    f.vWheelCmdL      = shared::g.vWheelCmdL.load(std::memory_order_relaxed);
    f.vWheelCmdR      = shared::g.vWheelCmdR.load(std::memory_order_relaxed);
    f.wheelActualMpsL = motors::wheelActualMpsL();
    f.wheelActualMpsR = motors::wheelActualMpsR();
    f.targetTurn      = shared::g.targetTurn.load(std::memory_order_relaxed);
    f.targetTurnUsed  = shared::g.targetTurnUsed.load(std::memory_order_relaxed);
    // LEDC diagnostic: cast int32_t step rates to float for the wire
    // format. Magnitudes here are in the low thousands (max step rate
    // ≈ vMaxWheel · stepsPerMeter, e.g. 1.5 m/s · 1273 steps/m ≈
    // 1900 Hz at 32 microsteps), well inside float's exact-int range,
    // so no precision loss.
    f.ledcReqL = static_cast<float>(motors::ledcReqStepsL());
    f.ledcGotL = static_cast<float>(motors::ledcGotStepsL());
    f.ledcReqR = static_cast<float>(motors::ledcReqStepsR());
    f.ledcGotR = static_cast<float>(motors::ledcGotStepsR());
    f.vBus     = shared::g.vBus.load(std::memory_order_relaxed);
    f.iBus     = shared::g.iBus.load(std::memory_order_relaxed);

    // Hand off to the UART codec. The coproc receives a PKT_TELEMETRY
    // packet wrapping these bytes verbatim and re-broadcasts to WS
    // clients with ws.binaryAll. We never serialize JSON on this path —
    // wire-format efficiency matters at 60 Hz. f.seq was already
    // post-incremented when we built the Frame above.
    //
    // Gated by s_enabled — when the operator has toggled the stream off
    // we still snapshot shared::g (cheap, keeps seq monotonic if they
    // toggle back on) but skip the tx. Status snapshots cover the
    // safety-critical bits (vBat / flags / readiness) at 1 Hz so the
    // coproc's battery page keeps refreshing.
    if (s_enabled.load(std::memory_order_relaxed)) {
      uartbus::sendTelemetry(reinterpret_cast<uint8_t*>(&f), sizeof(f));
    }

    // 1 Hz rate log. dt is measured from previous log, NOT assumed
    // 1000 ms — when the task is preempted for seconds at a time the
    // gap stretches and the printed rate auto-corrects. Compare against
    // the coproc's hb line: if main says tx_rate=60 but coproc says
    // tel_rx=6, frames are being lost on the wire. If both say 6, the
    // sender (this task) was actually starved by something on main.
    if (nowMs - lastLogMs >= 1000) {
      const uint32_t dt = nowMs - lastLogMs;
      const unsigned rate =
          (unsigned)((static_cast<uint64_t>(framesSinceLog) * 1000 + dt / 2) / dt);
      Serial.printf(
          "telemetry: tx_rate=%u/s (sent=%u in %u ms) max_tick_dt=%u ms (period=%u)\n",
          rate, (unsigned)framesSinceLog, (unsigned)dt,
          (unsigned)maxIntervalMs, (unsigned)(1000 / TELEMETRY_HZ));
      framesSinceLog = 0;
      maxIntervalMs  = 0;
      lastLogMs      = nowMs;
    }
  }
}

} // namespace

void start() {
  if (s_started) return;
  s_started = true;

  // Core 0 per PLAN.md §3 (telemetry sits with comms, not with control).
  // Stack: 4 KB is comfortable; the task allocates only the on-stack Frame
  // (56 B) plus a few locals. Priority 1 keeps it well below safety/control
  // on core 1 and below the WS server's own dispatch.
  xTaskCreatePinnedToCore(
    telemetryTask,
    "telemetry",
    4096,
    nullptr,
    1,
    nullptr,
    0 /* core 0 */);

  Serial.print(F("telemetry: started @ "));
  Serial.print(TELEMETRY_HZ);
  Serial.println(F(" Hz"));
}

void setEnabled(bool enabled) {
  const bool prev = s_enabled.exchange(enabled, std::memory_order_relaxed);
  if (prev != enabled) {
    Serial.printf("telemetry: %s\n", enabled ? "enabled" : "disabled");
  }
}

bool isEnabled() {
  return s_enabled.load(std::memory_order_relaxed);
}

} // namespace telemetry
