// diag.cpp — see diag.h.

#include "diag.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>

#include "boot_diag.h"
#include "imu.h"
#include "ina226.h"
#include "motors.h"
#include "params.h"
#include "safety.h"
#include "shared_state.h"
#include "telemetry.h"
#include "uartbus.h"

namespace diag {

namespace {

constexpr BaseType_t CORE_COMMS = 0;
constexpr int        DIAG_PERIOD_MS = 1000;   // 1 Hz
constexpr int        TASK_PRIORITY = 1;
constexpr int        TASK_STACK    = 6144;    // ArduinoJson serialization
                                              // can chew through stack on
                                              // a deep nested dump; the
                                              // current params doc is
                                              // shallow but give us room.

bool s_started = false;

// Build the binary status snapshot from the latest live state. Includes
// driver-register snapshots for the status page; those are six UART
// reads per side per second, well under the bus's bandwidth and
// serialised with the driver-watchdog task via the existing UartGuard.
void fillStatus(StatusSnapshot& s) {
  s.layoutVersion    = STATUS_LAYOUT_VERSION;
  s.imuReady         = imu::isReady() ? 1 : 0;
  s.driverReadyL     = motors::isDriverReadyL() ? 1 : 0;
  s.driverReadyR     = motors::isDriverReadyR() ? 1 : 0;
  s.ina226Ready      = ina226::isReady() ? 1 : 0;
  s.telemetryEnabled = telemetry::isEnabled() ? 1 : 0;
  s.safetyState      = static_cast<uint8_t>(safety::state());
  s.autoArmEnabled   = (params::current.autoArmEnabled > 0.5f) ? 1 : 0;
  s.flags            = shared::g.flags.load(std::memory_order_relaxed);
  s.resetReason      = static_cast<int16_t>(boot_diag::numericReason());
  s.reserved1        = 0;
  s.vBat             = shared::g.vBat.load(std::memory_order_relaxed);
  motors::readDriverSnapshot('L', s.drvL);
  motors::readDriverSnapshot('R', s.drvR);
  s.mainUptimeMs     = millis();
}

// Serialize params::current to a JSON byte sequence in `out`. Returns
// the number of bytes written, or 0 on overflow. JSON shape is the
// same {"type":"params","params":{…}} envelope the coproc would have
// pushed verbatim to a WS client previously — so the coproc can forward
// the bytes unchanged on a `getParams` request.
size_t serializeParamsJson(uint8_t* out, size_t cap) {
  // ArduinoJson v7 uses dynamic doc; size grows with content. Empirically
  // the full ControlParams dump is ~600 B; reserve 1.5 KB and abort if
  // we ever blow that.
  JsonDocument doc;
  doc["type"] = "params";
  JsonObject p = doc["params"].to<JsonObject>();
  params::toJson(params::current, p);
  const size_t n = serializeJson(doc, out, cap);
  return n;
}

void diagTask(void* /*arg*/) {
  // Last params copy we sent. memcmp on POD detects any changes; the
  // firstTick flag forces an unconditional send at boot.
  ControlParams lastParams{};
  bool     firstTick     = true;
  uint32_t lastSendMs    = 0;

  // Periodic re-broadcast. Without this, a coproc that reboots after
  // main has already sent its one boot-time params packet would never
  // receive params again (until something changes). Same applies to a
  // params packet lost to a single-frame UART CRC error. 5 s is a
  // tolerable staleness window for a fresh UI without spending
  // meaningful CPU on the ~1.4 KB ArduinoJson serialization.
  constexpr uint32_t PARAMS_REFRESH_MS = 5000;

  // Reusable JSON serialization buffer. 2 KB covers the current params
  // payload with margin.
  static uint8_t paramsBuf[2048];

  TickType_t nextWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(DIAG_PERIOD_MS));

    // ---- Status snapshot --------------------------------------------
    StatusSnapshot snap;
    fillStatus(snap);
    uartbus::sendStatus(reinterpret_cast<const uint8_t*>(&snap), sizeof(snap));

    // ---- Params dump (boot + on-change + periodic refresh) ----------
    const uint32_t       nowMs   = millis();
    const ControlParams& cur     = params::current;
    const bool changed = memcmp(&cur, &lastParams, sizeof(ControlParams)) != 0;
    const bool refresh = firstTick ||
                         (nowMs - lastSendMs) >= PARAMS_REFRESH_MS;
    if (changed || refresh) {
      const size_t n = serializeParamsJson(paramsBuf, sizeof(paramsBuf));
      if (n > 0 && n < sizeof(paramsBuf)) {
        uartbus::sendParams(paramsBuf, n);
        memcpy(&lastParams, &cur, sizeof(ControlParams));
        firstTick = false;
        lastSendMs = nowMs;
      } else {
        Serial.println(F("diag: params JSON didn't fit, skipping push"));
      }
    }
  }
}

} // namespace

void start() {
  if (s_started) return;
  s_started = true;
  xTaskCreatePinnedToCore(
      diagTask, "diag", TASK_STACK, nullptr,
      TASK_PRIORITY, nullptr, CORE_COMMS);
  Serial.println(F("diag: pushing status @ 1 Hz, params on change"));
}

} // namespace diag
