// telemetry.h — binary telemetry broadcaster. F11.
//
// PLAN.md §5.1. A 116-byte little-endian struct snapshot of the bot's
// running state is broadcast to every connected websocket client at
// TELEMETRY_HZ (100 Hz). Frames are sent on the websocket exposed by
// net::ws(); we don't manage clients directly.
//
// Frame layout (116 B, little-endian; matches the simulator's incoming
// parser in S2):
//
//   uint32 magic      = 0xB0B0B0B0
//   uint32 seq        // monotonic, increments per frame
//   float  t          // [s] uptime
//   float  theta      // [rad] complementary-filter output, what the
//                     //     controller consumes. There is no second
//                     //     post-comp LPF stage anymore.
//   float  thetaDot   // [rad/s] post-gyroAlpha LPF + optional notch
//   float  xDot       // [m/s] estimated cart velocity
//   float  thetaSet   // [rad] outer-loop output
//   float  vWheelCmd  // [m/s] inner-loop output (common-mode, pre-split)
//   float  outerP, outerI, outerD   // [rad]
//   float  vBat       // [V] smoothed
//   float  wheelActualMps  // [m/s] L+R average actual wheel speed
//   float  accelX     // [g] body-frame X accel (forward axis), raw.
//                     //     Diagnostic for "is the chassis really moving
//                     //     during a wobble, or is it just gyro pickup?".
//   float  gyroZ      // [rad/s] yaw-axis gyro, bias-corrected, raw axis
//                     //     (no sign flip). Diagnostic for yaw-rate
//                     //     bleed into the pitch channel during turns
//                     //     (mounting misalignment / cross-axis
//                     //     sensitivity); see shared_state.h.
//   uint16 flags      // bit0 FALLEN, bit1 LOW_BAT, bit2 MOTORS_ENABLED,
//                     // bit3 PS_CONNECTED  (mirrors shared::Flag bit
//                     // ordering exactly so the receiver can use the same
//                     // enum)
//   uint16 reserved   // 0; trails flags so the next field stays naturally
//                     // aligned in memory
//   float  targetV    // [m/s] raw shared::g.targetV (the unfiltered
//                     //     velocity command the operator/RC is asking
//                     //     for, BEFORE targetVAlpha smoothing). Charted
//                     //     alongside the outer-loop P/I/D so we can
//                     //     correlate setpoint inputs with controller
//                     //     output without having to reconstruct
//                     //     targetV from joystick state on the client.
//   float  vWheelCmdL // [m/s] per-wheel commanded velocity AFTER turn
//   float  vWheelCmdR //     differential split + ±vMaxWheel saturation,
//                     //     cart frame (positive = forward, mounting
//                     //     invert NOT applied — that's downstream in
//                     //     motors.cpp). vWheelCmd above is the
//                     //     common-mode pre-split value; these two are
//                     //     what was actually handed to setWheelVelocity.
//   float  wheelActualMpsL // [m/s] per-wheel actual derived from LEDC's
//   float  wheelActualMpsR //     achieved STEP frequency (lastLedcGotL/R
//                          //     in motors.cpp), mounting invert undone —
//                          //     cart frame. Diverges from cmd when LEDC
//                          //     can't realise the requested freq (e.g.
//                          //     above ~312 kHz at 8-bit res it idles
//                          //     the channel). Charted alongside
//                          //     vWheelCmdL/R for per-wheel cmd-vs-actual.
//   float  targetTurn // [m/s] raw shared::g.targetTurn (the unfiltered
//                     //     steering differential the operator/RC is
//                     //     asking for, BEFORE targetTurnAlpha smoothing
//                     //     and BEFORE the ±vMaxTurn clamp). Charted as
//                     //     "tgtTurn cmd" so it's clear this is what
//                     //     the user is *requesting*, not what the
//                     //     controller actually consumed.
//                     //     Sign convention matches shared_state.h:
//                     //     positive = bot turns right (CW seen from
//                     //     above, left wheel commanded faster).
//   float  targetTurnUsed // [m/s] post-targetTurnAlpha, post-±vMaxTurn
//                     //     clamp value that was actually summed into
//                     //     vL = vWheel + targetTurnUsed / vR = vWheel
//                     //     - targetTurnUsed THIS tick. Held at 0 by
//                     //     the controller while disarmed / IMU not
//                     //     ready. Charted alongside targetTurn so the
//                     //     UI shows both "what was commanded" and
//                     //     "what was used after ramping & clamping",
//                     //     making the difference between a step
//                     //     joystick input and the controller's
//                     //     consumed differential visually obvious.
//   float  ledcReqL   // [steps/sec, signed] LEDC step-pulse diagnostic
//   float  ledcGotL   //     pair for the L channel: req = what we last
//                     //     asked ledcWriteTone() to emit at the STEP
//                     //     pin, got = what it returned as the actual
//                     //     frequency it managed to set. Same units
//                     //     (Hz at the STEP pin, NOT m/s — the whole
//                     //     point is to show LEDC frequency-rounding
//                     //     in its native domain). Sign reflects the
//                     //     commanded direction at the time of the
//                     //     call (positive = forward, negative =
//                     //     reverse, 0 = channel idle / below deadband
//                     //     / stopped). At today's LEDC_RES_BITS=8 the
//                     //     two should track exactly; any divergence
//                     //     in the chart is the smoking gun for an
//                     //     LEDC quantisation regression after a
//                     //     resolution change.
//   float  ledcReqR   // Same as ledcReqL/ledcGotL but for the R
//   float  ledcGotR   //     channel. Both pairs reset to (0,0) on
//                     //     stop() so a Disarm doesn't leave the
//                     //     chart frozen on the last active spin.
//   float  vBus       // [V] INA226 bus voltage — the raw reading that
//                     //     batteryTask smooths into vBat above.
//                     //     Sampled at ~50 Hz on the shared I²C bus
//                     //     with the MPU6050; 0 if the chip is absent.
//   float  iBus       // [A] INA226 current (positive = drawn). Signed
//                     //     because regen / reverse current shows as
//                     //     negative. Use to chart wheel-load vs
//                     //     velocity-command, or detect a stalled
//                     //     stepper drawing peak current.
//
// ESP32 is little-endian, so a memcpy of the packed struct is wire-correct.
// When growing this layout, bump FRAME_SIZE in lock-step with the parser
// in simulator/src/datasource.ts (the static_assert below catches the
// firmware half automatically).

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace telemetry {

constexpr uint32_t MAGIC      = 0xB0B0B0B0u;
constexpr size_t   FRAME_SIZE = 116;

// Wire-format struct. Layout matches the byte-by-byte description above.
// Defined here (rather than in an anon namespace in telemetry.cpp) so the
// coproc can include this header to decode specific fields for its
// status-page renderer — coproc forwards the raw bytes to WS clients,
// but needs typed access for HTML formatting.
struct __attribute__((packed)) Frame {
  uint32_t magic;        // 0xB0B0B0B0
  uint32_t seq;          // monotonic
  float    t;            // [s] uptime
  float    theta;        // [rad]
  float    thetaDot;     // [rad/s]
  float    xDot;         // [m/s]
  float    thetaSet;     // [rad]
  float    vWheelCmd;    // [m/s]
  float    outerP;
  float    outerI;
  float    outerD;
  float    vBat;         // [V]
  float    wheelActualMps;
  float    accelX;       // [g]
  float    gyroZ;        // [rad/s]
  uint16_t flags;        // mirrors shared::Flag
  uint16_t reserved;     // 0
  float    targetV;      // [m/s] raw shared::g.targetV
  float    vWheelCmdL;
  float    vWheelCmdR;
  float    wheelActualMpsL;
  float    wheelActualMpsR;
  float    targetTurn;
  float    targetTurnUsed;
  float    ledcReqL;
  float    ledcGotL;
  float    ledcReqR;
  float    ledcGotR;
  float    vBus;         // [V] INA226
  float    iBus;         // [A] INA226
};
static_assert(sizeof(Frame) == FRAME_SIZE,
              "telemetry::Frame layout must match FRAME_SIZE");

// One-time init: spawns telemetryTask on core 0 (PLAN.md §3). Idempotent.
void start();

// Runtime gate on the 60 Hz tx stream. When disabled the task keeps
// running (status flags, log lines still fire) but uartbus::sendTelemetry
// is short-circuited — no UART bandwidth, no WS broadcast. Status
// snapshots (PKT_STATUS @ 1 Hz) are independent and continue regardless,
// so coproc-side battery page / status HTML still get fresh values.
// Default at boot: enabled. The UI persists the operator's choice and
// re-pushes a setTelemetryEnabled command on WS reconnect.
void setEnabled(bool enabled);
bool isEnabled();

} // namespace telemetry
