// shared_state.h — inter-task shared data.
//
// PLAN.md §3: a single global struct holds the hot-path values that flow
// between the FreeRTOS tasks. All hot fields are 32-bit (`float` or
// `uint32_t` / `uint16_t`) and use `std::atomic` for lock-free access
// (lock-free on ESP32/Xtensa for naturally-aligned 32-bit types).
//
// No per-field mutex / spinlock is taken on the control loop's hot path.
// Coherence-across-fields (e.g. for a single telemetry frame) is approximate
// — telemetry just snapshots each field independently. That is acceptable:
// the receiving end never relies on micro-synchronous fields, and any
// stronger guarantee would either require a critical section in the control
// loop (which we do not want) or double-buffering (overkill for now).
//
// Writer/reader conventions are documented per field. Don't introduce new
// writers for a given field without revisiting concurrency.

#pragma once

#include <atomic>
#include <stdint.h>

class Print; // Arduino base output stream; forward-declared at file scope.

namespace shared {

// Status flag bits packed into SharedState::flags.
enum Flag : uint16_t {
  FLAG_FALLEN         = 1u << 0,
  FLAG_LOW_BAT        = 1u << 1,
  FLAG_MOTORS_ENABLED = 1u << 2,
  FLAG_PS_CONNECTED   = 1u << 3,
};

struct SharedState {
  // ---- Targets ----------------------------------------------------------
  // Writers: joystickTask, wsServerTask. Readers: controlTask.
  std::atomic<float>    targetV{0.0f};      // [m/s] commanded cart velocity
  // [m/s] commanded wheel-velocity differential (steering). Added to the
  // left wheel and subtracted from the right after the inner-loop
  // common-mode vWheel is filtered, so the cart-frame velocity estimate
  // is unaffected (it sees only the average). Sign: positive = left
  // faster = bot turns right (CW seen from above). Clamped at write time
  // by both setTurn (net.cpp) and the controller against vMaxTurn.
  std::atomic<float>    targetTurn{0.0f};

  // Post-controller view of targetTurn, exported for telemetry. Equals
  // the value that was actually summed into vL = vWheel + targetTurnUsed
  // / vR = vWheel - targetTurnUsed THIS TICK, after:
  //   1. the targetTurnAlpha 1-pole IIR ramp filter
  //   2. the ±vMaxTurn clamp
  //   3. the disarm / !imuOk reset (held at 0 while not running)
  // Whereas targetTurn above is the RAW operator/RC request straight
  // from the joystick / WS handler — what the user is *asking* for.
  // Charting both side-by-side lets the UI distinguish "the operator
  // commanded a step turn but the ramp is still settling" from "the
  // controller is consuming the full requested differential". Writer:
  // controlTask. Readers: telemetryTask.
  std::atomic<float>    targetTurnUsed{0.0f};

  // ---- IMU outputs ------------------------------------------------------
  // Writer: controlTask (after IMU read + filter). Readers: telemetryTask.
  std::atomic<float>    theta{0.0f};        // [rad] body tilt (pitch). Output
                                            //       of the complementary
                                            //       filter; consumed by the
                                            //       controller. There is no
                                            //       second post-comp LPF
                                            //       stage anymore — tune
                                            //       smoothing via compAlpha.
  std::atomic<float>    thetaDot{0.0f};     // [rad/s] post-gyroAlpha LPF +
                                            //         optional notch.
  // Body-frame X accel (forward axis), in g. Diagnostic: a stationary
  // tilted bot reads ax ≈ -sin(theta); subtract that off and the residual
  // is real translational acceleration. Lets the operator distinguish
  // between "gyro is picking up vibration" vs "chassis is actually
  // translating" during a wobble.
  std::atomic<float>    accelX{0.0f};
  // Yaw-axis gyro reading [rad/s], post bias subtraction, NO sign flip
  // (raw chip Z axis). Diagnostic — used to investigate yaw-rate
  // bleed into the pitch (Y) channel during a turn (mounting
  // misalignment / cross-axis sensitivity). If this trace tracks
  // thetaDot during a pure-yaw spin (bot held off the floor and rotated
  // about vertical, no real pitch motion), there's cross-coupling — and
  // the params::current.gyroYawXTalk compensation in imu.cpp is the
  // knob to dial it out (see params.h for procedure). Sign convention:
  // positive = rotating CCW seen from above (right-hand rule about chip
  // Z).
  std::atomic<float>    gyroZ{0.0f};

  // ---- Controller outputs ----------------------------------------------
  // Writer: controlTask. Readers: motors (within controlTask), telemetryTask.
  std::atomic<float>    thetaSet{0.0f};     // [rad] outer-loop output
  std::atomic<float>    vWheelCmd{0.0f};    // [m/s] inner-loop output
                                            //       (common-mode, pre-turn-split)
  // Per-wheel commanded velocities AFTER the steering differential is
  // applied and AFTER the per-side ±vMaxWheel saturation in
  // controller.cpp — i.e. the values that are passed to
  // motors::setWheelVelocity() this tick. Cart frame (positive = forward),
  // mounting invert NOT applied (motors:: handles that downstream).
  // Useful for debugging asymmetric behaviour (one wheel saturating
  // while the other doesn't, or steering bias) that the common-mode
  // vWheelCmd above hides by averaging. Resets to 0 on disarm / !imuOk
  // alongside the other controller outputs so telemetry doesn't display
  // stale per-side values while the wheels are stopped.
  std::atomic<float>    vWheelCmdL{0.0f};
  std::atomic<float>    vWheelCmdR{0.0f};
  std::atomic<float>    xDotEst{0.0f};      // [m/s] estimated cart velocity
  std::atomic<float>    outerP{0.0f};
  std::atomic<float>    outerI{0.0f};
  std::atomic<float>    outerD{0.0f};

  // ---- Motor actuals ----------------------------------------------------
  // Writer: telemetryTask (calls motors::wheelActualMps{,L,R}()). Readers:
  // telemetryTask (same, as part of frame). "Actual" here is computed
  // from the LEDC peripheral's *achieved* step frequency (lastLedcGotL/R
  // in motors.cpp), not the *commanded* rate — so silent LEDC failures
  // (e.g. requested freq exceeds what 8-bit duty resolution can divide
  // to) show up as actual=0 even when the controller asked for nonzero.
  // Sign-corrected for the mounting invert. Compare against vWheelCmd
  // to see if commands are surviving the LEDC layer.
  // wheelActualMps is the L+R average for legacy single-line charts;
  // wheelActualMps{L,R} are per-side for the cmd-vs-actual L/R chart.
  std::atomic<float>    wheelActualMps{0.0f};

  // ---- Battery ---------------------------------------------------------
  // Writer: batteryTask (smoothed copy of vBus). Readers: safetyTask,
  // telemetryTask.
  std::atomic<float>    vBat{0.0f};         // [V] smoothed pack voltage

  // ---- INA226 (high-side power monitor) -------------------------------
  // Writer: ina226Task at ~50 Hz. Readers: batteryTask, telemetryTask.
  // vBus is the raw bus-voltage reading; batteryTask smooths it into
  // vBat above. Both 0 until the INA226 task finishes its first valid
  // read; after that they hold the latest sample (no explicit "stale"
  // indicator — telemetry shows a flat trace if I²C goes away).
  std::atomic<float>    vBus{0.0f};         // [V] bus voltage
  std::atomic<float>    iBus{0.0f};         // [A] current (positive = drawn)

  // ---- Status ----------------------------------------------------------
  // Writers: safetyTask, joystickTask, netTask (each owns specific bits;
  // see Flag).  Use setFlag()/clearFlag() to keep writers from racing on
  // unrelated bits.
  std::atomic<uint16_t> flags{0};

  // ---- Telemetry sequence ---------------------------------------------
  // Writer: telemetryTask (monotonic on each frame send).
  std::atomic<uint32_t> seq{0};
};

// Process-wide singleton. Defined in shared_state.cpp.
extern SharedState g;

// Atomic flag helpers. Implemented as compare-exchange loops; lock-free.
void setFlag(Flag f, bool on);
bool getFlag(Flag f);

// Convenience: human-readable flag dump (no newline).
void printFlags(uint16_t bits, ::Print& out);

} // namespace shared
