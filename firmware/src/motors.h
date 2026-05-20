// motors.h — TMC2209 + LEDC wheel-velocity driver. F6.
//
// Responsibilities (PLAN.md §1 / §3):
//   - Configure two TMC2209 stepper drivers over a shared UART bus
//     (run/hold currents, microsteps, stealthChop). Addresses are pinstrap
//     on hardware (MS1/MS2) — see DRV_ADDR_L/R in config.h. The control-
//     plane work is delegated to motors_drv.{h,cpp}; this module owns
//     only the step-pulse generation.
//   - Drive STEP_L/R via two ESP32 LEDC channels (ledcWriteTone), with
//     DIR_L/R as plain GPIO outputs. No acceleration ramp, no command
//     queue: every controller tick translates to a single LEDC frequency
//     write.
//   - Expose a non-blocking `setWheelVelocity(left, right)` API in m/s.
//     This is the only call the controller (F7) needs to issue at 200 Hz.
//
// motors::start() leaves the drivers configured but the EN pin held HIGH
// (disabled). The safety state machine (F8) owns EN gating; nothing in this
// module touches the EN pin after boot. That way a controller bug or a lost
// comms session can never silently re-enable the motors.
//
// Sign convention: positive wheel velocity = forward. Per-side mounting
// inversion is handled inside this module via MOTOR_INVERT_L/R from
// config.h, so callers always pass cart-frame signs.

#pragma once

#include <stdint.h>

class Print; // Arduino base output stream; forward-declared at file scope.

namespace motors {

// One-time bring-up. Configures UART2 to the driver bus, initialises both
// TMC2209s (currents/microsteps/stealth from params::current), brings up
// the LEDC step generator and DIR GPIOs. Returns true if both drivers
// responded over UART; false if either one is unreachable (in which case
// stepping still works but driver registers were not configured).
bool start();

// True iff start() ran and both drivers responded over UART.
bool isReady();

// Per-driver liveness: true iff the corresponding TMC2209 acked the
// IFCNT-increment probe at start(). Useful for status pages that want to
// distinguish "L ok, R missing" from "both missing" — isReady() collapses
// that into a single boolean.
bool isDriverReadyL();
bool isDriverReadyR();

// Update commanded wheel linear velocities, in m/s, in the cart frame.
// Internally converts to driver step rates using WHEEL_RADIUS_M and the
// configured microstep count, applies MOTOR_INVERT_L/R, and pokes the
// LEDC channels. Safe to call at the 200 Hz control rate.
void setWheelVelocity(float vLeft, float vRight);

// Equivalent to setWheelVelocity(0, 0) but stops both LEDC channels
// immediately (no ramp). Used by safety on fall/disarm.
void stop();

// Re-apply currents / microsteps from params::current. Cheap; intended
// for the param-update path. No-op if drivers aren't ready.
void applyParamsLive();

// Current actual wheel velocity in m/s, averaged across L and R, in the
// cart frame (positive = forward, per-side mounting invert undone).
// Computed from the LEDC peripheral's *achieved* STEP frequency (the
// "got" value returned by ledcSetup), NOT from the commanded step rate
// — so when LEDC silently rejects a frequency it can't realise (above
// ~312 kHz at 8-bit duty resolution → got=0 → channel idled), this
// function correctly reports zero forward velocity. The controller
// uses this for its outer-loop xDotEst feedback so a silent LEDC
// failure produces a visible velocity error instead of a fall.
float wheelActualMps();

// Per-side variants of wheelActualMps(). Same "from LEDC achieved
// frequency, not commanded rate" rationale; mounting invert undone,
// cart frame. Used by telemetry so the UI can chart asymmetric
// saturation / per-wheel stalls that the L+R average above hides,
// AND show post-LEDC-quantisation behaviour vs the cmd line.
float wheelActualMpsL();
float wheelActualMpsR();

// LEDC step-pulse diagnostic getters. For each side, the most recent
// (req, got) pair as last passed to / returned from ledcWriteTone():
//
//   ledcReqStepsL()/R()  — what we asked the LEDC peripheral to emit
//                          (signed: positive = forward direction at the
//                          time of the call, negative = reverse, 0 =
//                          channel idle / below deadband / stopped).
//   ledcGotStepsL()/R()  — what ledcWriteTone() returned as the actual
//                          frequency it managed to set (signed using the
//                          same direction sign as req). At 8-bit LEDC
//                          resolution the LEDC timer divider can hit
//                          common step rates exactly, but a future
//                          resolution change could introduce rounding;
//                          divergence between req and got is the smoking
//                          gun for that. Both expressed as signed step
//                          rates (Hz at the STEP pin) — same units as
//                          the controller's commanded value before the
//                          m/s conversion.
//
// Captured inside applyToChannel() (the only place that calls
// ledcWriteTone for a side) and inside stop() (which forces both to 0).
// Telemetered separately so the UI can chart req-vs-got per channel
// alongside cmd-vs-actual; if they ever diverge it's a firmware
// quantisation bug, not a mechanical one.
int32_t ledcReqStepsL();
int32_t ledcGotStepsL();
int32_t ledcReqStepsR();
int32_t ledcGotStepsR();

// Print a one-line driver status summary on the given stream (run current,
// microsteps, GSTAT/DRV_STATUS bits). Used by the F6 self-test and by
// later debug commands.
void printStatus(::Print& out);

// Live per-driver register snapshot. Filled by readDriverSnapshot() with
// the same UART reads printDriverDetailsHtml does. Layout is shared
// verbatim with the coproc (diag::StatusSnapshot embeds one of these
// per side, the coproc renders them on its status page). Pack so the
// wire size doesn't depend on host-side alignment quirks.
struct __attribute__((packed)) DriverSnapshot {
  uint8_t  valid;            // 0 = driver wasn't reachable; other fields meaningless
  uint8_t  pad0;
  uint16_t pad1;
  uint32_t ioin;             // IOIN register (top byte = silicon version)
  uint32_t gconf;
  uint32_t drv_status;
  uint32_t tstep;
  uint32_t reset_recoveries; // watchdog-observed re-init count for this side
  uint16_t microsteps;       // CHOPCONF.MRES decoded
  uint16_t rms_current_ma;   // configured (TMC2209 doesn't measure)
  uint8_t  ifcnt;
  uint8_t  gstat;            // NB: register read clears latches, so this
                             //     captures the LAST observed value
  uint8_t  cs_actual;        // (DRV_STATUS >> 16) & 0x1F
  uint8_t  pad2;
};
static_assert(sizeof(DriverSnapshot) == 32,
              "DriverSnapshot wire layout must be stable across main + coproc");

// Read all the registers the status page needs into `out`. Uses the same
// internal UART guard as printDriverDetailsHtml — safe to call from any
// task that wants a 1 Hz snapshot (e.g. diag::diagTask). side='L'/'R';
// out.valid is 0 if the driver wasn't detected at boot, otherwise 1 with
// all fields populated. Read cost: ~6 register reads over the TMC2209
// UART bus (~few hundred µs total).
void readDriverSnapshot(char side, DriverSnapshot& out);

// Emit live per-driver register state as HTML <tr> rows for the / status
// page. side='L' or 'R'. Internally calls readDriverSnapshot. Kept for
// the firmware's serial-debug paths; the coproc renders its own HTML
// from a streamed DriverSnapshot rather than calling this function.
void printDriverDetailsHtml(::Print& out, char side);

} // namespace motors
