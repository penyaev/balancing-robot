// params.h — runtime-tunable controller parameters.
//
// Mirrors the schema in PLAN.md §6. Fields are kept POD so the struct can be
// copied around freely between tasks. Persistence is via ESP32 NVS using
// the Arduino `Preferences` library (one key per field, namespace "bb").
//
// Mutation API:
//   - setByPath(name, value): used by the WS control channel. Returns true on
//     a recognised name.
// Serialization API:
//   - toJson / fromJson: mirror used by the WS "params" / "setParam"
//     messages. fromJson is permissive: missing fields are left untouched.

#pragma once

#include <stdint.h>

#include <ArduinoJson.h>

class Print;  // Arduino base output stream; forward-declared at file scope.

struct ControlParams {
  // Outer (velocity -> tilt setpoint) PID.
  float velKp;
  float velKi;
  float velKd;
  float velErrDeadband;   // [m/s] symmetric dead zone applied to the outer-
                          //     loop velocity error (err = targetV - xDotEst)
                          //     BEFORE any P/I/D computation. When
                          //     |err| < velErrDeadband, err is clamped to 0
                          //     for this tick — so P contributes nothing,
                          //     the integrator doesn't accumulate, and the
                          //     dErr term sees no change. 0 = disabled
                          //     (default, preserves the original behaviour
                          //     where every microscopic xDotEst quantum
                          //     leaks into thetaSet). Use to stop the
                          //     endless tiny corrections the outer loop
                          //     otherwise generates when the bot is
                          //     essentially stationary but xDotEst flickers
                          //     between adjacent LEDC step rates. Typical
                          //     tuning starts around 0.02–0.05 m/s; raise
                          //     until the bot stops twitching, lower until
                          //     genuine velocity tracking still feels
                          //     responsive. Side note: the deadband is
                          //     applied as a hard "round to zero" rather
                          //     than the smoother "subtract band edge"
                          //     form, so there is a small dErr spike at
                          //     the band boundary — usually invisible
                          //     because velKd is small or zero, but worth
                          //     knowing.
  float velIClamp;        // [rad]
  float maxAngleSetpoint; // [rad]
  float velDAlpha;        // [-] 1-pole IIR mix coefficient on the outer
                          //     D term, applied AFTER multiplying by velKd
                          //     and BEFORE summing with P+I into thetaSet.
                          //     y[n] = α·x[n] + (1-α)·y[n-1]. 1.0 =
                          //     passthrough (default historical behaviour
                          //     was a Hz-cutoff LPF on err pre-diff; this
                          //     is the post-gain mirror of velPAlpha and
                          //     for an LTI 1-pole IIR the two
                          //     parametrisations are equivalent up to
                          //     constant gains). Lower = stronger
                          //     smoothing of the D contribution at the
                          //     cost of phase lag in the velocity-
                          //     tracking loop. Differentiation itself is
                          //     done on raw err (no pre-LPF): dErr =
                          //     (err - errPrev) / outerDt; the IIR on
                          //     post-gain D is what tames noise.
                          //
                          //     Replaced velDLowpassHz (Hz cutoff on
                          //     err pre-diff). Convert from the legacy
                          //     8 Hz default at outerDt=0.005 s via
                          //     α = dt/(rc+dt), rc=1/(2π·fc) ≈ 0.0199 →
                          //     α ≈ 0.20. Effective sample period is
                          //     outerDt = N·DT, so the time-domain
                          //     shape interprets at the outer rate
                          //     regardless of outerEveryN. Filter step
                          //     runs on outer-run ticks only; skip
                          //     ticks reuse heldOuterD verbatim. State
                          //     resets to 0 on disarm / !imuOk /
                          //     !outerOn alongside the rest of the
                          //     outer-loop reset convention. Clamped
                          //     defensively against bad setParam
                          //     values.
  float outerEnabled;     // [bool 0/1] master switch for the outer
                          //     velocity→tilt-setpoint PID. 1 = on
                          //     (default, normal balancing). 0 = the
                          //     outer loop is held in reset exactly the
                          //     same way it is during disarm: thetaSet,
                          //     P/I/D, integrator, dErr LPF, targetV
                          //     ramp filter and the outer-P alpha
                          //     filter all stay at 0, and
                          //     targetVUsed=0 so the inner-loop
                          //     velFF·targetV term contributes nothing.
                          //     The inner PD (Kth, KthDot) keeps running
                          //     as a pure tilt corrector around upright,
                          //     so the bot still balances — it just won't
                          //     try to track a commanded velocity. Useful
                          //     for isolating outer-loop tuning issues
                          //     from inner-loop ones with one click from
                          //     the web UI. Stored as a float to fit the
                          //     existing FLOAT_FIELDS persistence table
                          //     (same convention as autoArmEnabled).
  float velPAlpha;        // [-] 1-pole IIR mix coefficient on the outer
                          //     P term, applied AFTER multiplying by velKp
                          //     and BEFORE summing with I+D into thetaSet.
                          //     y[n] = α·x[n] + (1-α)·y[n-1]. 1.0 =
                          //     passthrough (default). Lower = smoother
                          //     P contribution at the cost of phase lag in
                          //     the velocity-tracking loop. Useful when
                          //     xDotEst quantisation (or any noise on the
                          //     velocity estimate) shows up as jitter on
                          //     thetaSet that the inner loop then chases.
                          //     Filter state is owned by the control task
                          //     and resets to 0 on disarm / !imuOk so each
                          //     arm starts the P term from rest, mirroring
                          //     the integrator and the LPF reset above.
  float outerAlpha;       // [-] 1-pole IIR mix coefficient on the WHOLE
                          //     outer-loop output thetaSet, applied AFTER
                          //     summing P+I+D and AFTER the maxAngleSetpoint
                          //     clamp, BEFORE handing the value to the inner
                          //     loop and BEFORE latching into heldThetaSet
                          //     for the publish / downsample-hold paths.
                          //     y[n] = α·x[n] + (1-α)·y[n-1]. 1.0 =
                          //     passthrough (default). Lower = smoother
                          //     thetaSet trace at the cost of phase lag in
                          //     the velocity-tracking loop.
                          //
                          //     Distinct from velPAlpha (which filters
                          //     only the P term): this one smooths the
                          //     COMPOSITE setpoint, so quantisation /
                          //     noise on any of P, I, D — and any inter-
                          //     action between them — is attenuated in
                          //     one place. Useful in particular at
                          //     outerEveryN > 1 where the outer block
                          //     re-evaluates rarely and a noisy run-tick
                          //     value would otherwise be held verbatim
                          //     for N-1 inner-loop ticks. Filtering
                          //     AFTER the clamp means the filter state
                          //     can never exceed ±maxAngleSetpoint, so
                          //     the inner loop is still bounded the
                          //     same way it was before.
                          //
                          //     Filter step is taken on outer-run ticks
                          //     only; on skip ticks the held value just
                          //     carries through (zero-order hold), which
                          //     is the natural behaviour for an IIR with
                          //     no new sample to consume. The effective
                          //     sample period of the recurrence is
                          //     therefore outerDt = N·DT — same as the
                          //     integrator and the dErr LPF — so the
                          //     time-domain shape is interpreted at the
                          //     outer rate regardless of N. State is
                          //     held by the control task and reset to 0
                          //     on disarm / !imuOk / !outerOn alongside
                          //     the rest of the outer-loop reset
                          //     convention. Clamped defensively against
                          //     bad setParam values.
  float outerEveryN;      // [ticks, ≥1, stored as float] outer-loop
                          //     downsample factor. The outer (velocity →
                          //     tilt-setpoint) PID re-evaluates only every
                          //     N control ticks; the inner PD continues
                          //     to run every tick at CONTROL_LOOP_HZ. N=1
                          //     (default) = outer runs every tick — same
                          //     rate as inner, identical to the legacy
                          //     behaviour. N=4 at CONTROL_LOOP_HZ=200 →
                          //     outer at 50 Hz, inner at 200 Hz. Useful
                          //     because the cart-velocity dynamics are
                          //     much slower than the tilt dynamics, and
                          //     re-evaluating the outer loop at 200 Hz
                          //     mostly amplifies xDotEst quantisation
                          //     noise straight into thetaSet (which the
                          //     inner loop then chases). Downsampling
                          //     also gives the velPAlpha + velDAlpha
                          //     filters a longer effective sample period,
                          //     i.e. more smoothing per outer step.
                          //
                          //     Mechanics: between runs, the published
                          //     thetaSet / outerP / outerI / outerD and
                          //     the consumed targetVUsed are held at the
                          //     last computed value so the inner loop
                          //     sees a smooth zero-order-hold setpoint
                          //     (no glitches at the downsample boundary).
                          //     On a run tick, the integrator step and
                          //     the dErr divisor use outerDt = N·DT so
                          //     velKi / velKd keep their tuned meaning
                          //     regardless of N (raising N does not
                          //     silently weaken Ki).
                          //
                          //     Reset semantics: the held outputs and the
                          //     downsample counter are zeroed on disarm,
                          //     on !imuOk, and on outerEnabled=0, so the
                          //     first armed-and-enabled tick always runs
                          //     the outer block immediately (no random
                          //     phase relative to the operator's arm
                          //     click). Stored as a float to fit the
                          //     existing FLOAT_FIELDS persistence table
                          //     (same convention as autoArmEnabled);
                          //     clamped to ≥1 inside the controller via
                          //     lroundf so bad setParam values can't stall
                          //     the outer loop indefinitely.

  // Inner (tilt -> wheel velocity) PD + feed-forward.
  float innerEnabled;     // [bool 0/1] master switch for the inner
                          //     tilt→wheel-velocity PD. 1 = on (default,
                          //     normal balancing). 0 = the Kth·thetaErr
                          //     and KthDot·thDot terms are dropped from
                          //     the wheel-command formula; only the
                          //     velFF·targetVUsed feed-forward survives,
                          //     so the bot becomes an open-loop velocity
                          //     drive controlled by the joystick (with
                          //     the steering differential, vWheelAlpha
                          //     filter and per-wheel saturation still
                          //     applied as usual). The bot will NOT
                          //     balance with this off — the inner PD is
                          //     the tilt corrector — but it lets you
                          //     drive the wheels with one click for
                          //     bench tests (step-skip diagnosis,
                          //     steering symmetry, deadband sweeps,
                          //     resonance hunts) without rebuilding
                          //     firmware. Symmetric mirror of
                          //     outerEnabled above. Stored as a float to
                          //     fit the existing FLOAT_FIELDS persistence
                          //     table (same convention as outerEnabled
                          //     and autoArmEnabled). NOTE: auto-arm will
                          //     happily fire even with the inner loop
                          //     off — the FSM only knows about tilt /
                          //     dwell, not about which controller paths
                          //     are active. Disarm or disable auto-arm
                          //     before flipping this on a bot that's
                          //     near upright.
  float Kth;              // [m/s per rad of tilt]
  float KthDot;           // [m/s per (rad/s) of tilt rate]
  float velFF;            // [unitless] target_v feed-forward gain
  float targetVAlpha;     // [-] 1-pole IIR mix coefficient on the raw
                          //     shared::g.targetV before the controller
                          //     consumes it. Both outer-loop err and the
                          //     inner-loop velFF·targetV term see the
                          //     filtered value. y[n] = α·x[n] +
                          //     (1-α)·y[n-1]. 1.0 = passthrough (default).
                          //     Lower = the bot ramps gradually into a
                          //     new commanded velocity instead of
                          //     stepping, suppressing the inner-loop
                          //     transient when the operator pushes the
                          //     stick. Filter state resets to 0 on
                          //     disarm and !imuOk so every arm ramps
                          //     from rest toward the live joystick
                          //     value, even if the stick was already
                          //     pushed before arming. Trade-off: lower
                          //     α = smoother but slower joystick
                          //     response.
  float thetaTrim;        // [rad] static tilt offset for chassis CG
  float targetTurnAlpha;  // [-] 1-pole IIR mix coefficient on the raw
                          //     shared::g.targetTurn before the controller
                          //     applies the steering differential. Same
                          //     recurrence as targetVAlpha. 1.0 =
                          //     passthrough (default). Lower = the bot
                          //     ramps gradually into a new commanded turn
                          //     instead of stepping. Filter state resets
                          //     to 0 on disarm and !imuOk so every arm
                          //     starts with no differential and ramps in.
                          //     Independent of targetVAlpha so steering
                          //     and forward velocity can have different
                          //     response shapes.

  // Stick response curve. Applied in joystick.cpp (UART receiver path)
  // BEFORE the value lands in shared::g.targetV / targetTurn, so the
  // outer loop sees a curved+deadbanded version of the raw stick. The
  // same curve is used for both the cart-velocity axis (leftY) and the
  // steering axis (rightX) so the operator gets consistent feel between
  // them.
  //
  // Pipeline per axis: int16 from wire (±1000) → normalize to [-1,+1]
  //                  → apply translated deadband (smooth, not "round
  //                    to zero": outputs 0 inside ±deadband, and the
  //                    just-past-the-edge value starts at 0 rather
  //                    than stepping to ±deadband)
  //                  → power-law curve y = sign(x)·|x|^expo for expo>1
  //                    that gives a "slow at low values, fast at high
  //                    values" feel (standard RC expo curve)
  //                  → scale by vMaxCart / vMaxTurn.
  //
  // Both knobs are independent of targetVAlpha / targetTurnAlpha
  // (which run later, inside the controller) — the curve shapes the
  // operator's stick input, the alphas smooth changes over time.
  float stickDeadband;    // [-] normalized half-width of the dead zone,
                          //     0..0.95 clamped. 0 = no deadband (raw
                          //     stick passes through the curve). 0.04 =
                          //     ~4% — large enough to swallow DualSense
                          //     at-rest jitter without feeling sluggish.
                          //     Inside ±this, output is exactly 0;
                          //     outside, the residual is remapped to
                          //     [0..1] before the expo curve. Default
                          //     0.05.
  float stickExpo;        // [-] response-curve exponent, >=0.1 clamped.
                          //     1.0 = linear (no curve). 2.0 = gentle
                          //     expo: tiny stick movement at low end
                          //     produces tiny velocity, full stick at
                          //     high end produces full velocity, with
                          //     accelerating sensitivity in between.
                          //     3.0 = aggressive expo. Default 2.0.

  // Limits / safety.
  float vMaxCart;         // [m/s] joystick scale
  float vMaxWheel;        // [m/s] saturation on inner-loop output
  float vMaxTurn;         // [m/s] saturation on the steering differential
                          //       (shared::g.targetTurn). Clamps both the
                          //       setTurn WS path and the controller's
                          //       per-tick read so a wild client can't
                          //       drive |vL-vR|/2 past this value. Default
                          //       0.5 m/s. Should be < vMaxWheel.
  float aMaxWheel;        // [m/s^2] dV/dt cap enforced in motors::setWheelVelocity;
                          //         0 disables (passthrough). See motors.cpp for
                          //         the rationale (overcurrent + closed-loop
                          //         positive-feedback chatter).
  float fallAngle;        // [rad] safety cutoff
  float vBatCutoff;       // [V]   low-battery cutoff

  // Auto-arm.
  //
  // When the bot is in safety::READY (preconditions ok, not yet armed) and
  // the trim-corrected tilt |theta - thetaTrim| stays within autoArmAngle
  // for autoArmHoldMs continuous milliseconds, the safety FSM transitions
  // to ARMED automatically — same as if the operator had pressed Arm.
  //
  // To prevent immediate re-arm after a manual disarm or post-fall reset,
  // an inhibit flag is set on every entry to DISARMED and only cleared
  // once |theta - thetaTrim| > autoArmAngle is observed in READY. So the
  // user has to physically tilt the bot past the threshold once before
  // auto-arm engages again.
  float autoArmEnabled;   // [bool 0/1] master switch. 1 = on (default).
  float autoArmAngle;     // [rad] window half-width around thetaTrim.
                          //       Default ~0.01745 rad = 1°.
  float autoArmHoldMs;    // [ms] continuous dwell time required inside
                          //      the window before auto-arming. Default 500.

  // Driver.
  uint16_t microsteps;    // 8/16/32/...
  float    runCurrent;    // [A]
  float    holdCurrent;   // [A]

  // Filter.
  float compAlpha;        // [-] complementary-filter mix coefficient,
                          //     applied per IMU sample at SAMPLE_HZ. Per-tick
                          //     update is `theta = (1-α)·(theta+gyro·dt) + α·thetaAcc`,
                          //     so α≈0 trusts the gyro fully (drifts) and α≈1
                          //     follows the accel (noisy). Typical 0.005–0.02
                          //     at 200 Hz. Replaces the older compFilterTau,
                          //     which forced a per-tick `dt/(τ+dt)` divide and
                          //     conflated tuning with sample rate.
  float accAlpha;         // [-] 1-pole IIR mix coefficient on thetaAcc
                          //     BEFORE it enters the complementary filter:
                          //     thetaAccFilt = α·atan2(ax,az) + (1-α)·prev.
                          //     Lets you denoise the gravity-vector reference
                          //     without changing compAlpha (which controls
                          //     gyro-vs-accel trust crossover, a separate
                          //     concern). 1.0 = passthrough (raw atan2 fed
                          //     straight into the comp filter, the original
                          //     behaviour). Lower = stronger smoothing of the
                          //     accel path; stacks with compAlpha to give a
                          //     2-pole low-pass on the accel branch. Typical
                          //     0.05–0.3 if you want it; otherwise leave at 1.
                          //     Useful when chassis vibration shows up as
                          //     low-amplitude jitter in theta even after
                          //     comp-filter smoothing.
  float gyroAlpha;        // [-] 1-pole IIR mix coefficient on raw thetaDot,
                          //     applied per IMU sample. New = α·raw + (1-α)·prev.
                          //     0 disables (passthrough). Tames D-on-noise:
                          //     with KthDot>0, raw MPU6050 gyro noise feeds
                          //     directly into vWheelCmd, manifesting as a
                          //     5–10 Hz limit cycle the moment wheels gain
                          //     traction. Replaces gyroLpfHz; convert via
                          //     α = dt / (1/(2π·fc) + dt).
  float notchFc;          // [Hz] biquad notch center frequency applied to
                          //      thetaDot AFTER the gyroAlpha LPF, BEFORE
                          //      storing to shared::g.thetaDot. Targets a
                          //      known structural-resonance / limit-cycle
                          //      peak (we observed ~7 Hz on the bench). The
                          //      controller's KthDot term sees the notched
                          //      signal so a narrow noise band no longer
                          //      feeds back into vWheelCmd. 0 = disabled
                          //      (passthrough). Phase response is
                          //      well-behaved away from fc.
  float notchQ;           // [-] notch quality factor. Larger Q = narrower
                          //     notch. Typical 3–10. Too high = brittle
                          //     against drift in the resonant frequency;
                          //     too low = swallows useful bandwidth.
                          //     Ignored when notchFc <= 0.
  // Gyro yaw→pitch cross-talk compensation.
  //
  // PROBLEM: a pure rotation about the chassis Z (yaw) axis shows up as a
  // small fake pitch rate on the gyro Y axis, so the controller thinks
  // the bot is tipping while it's actually just turning in place. Two
  // physical sources, indistinguishable from the controller's point of
  // view:
  //   1. Mounting yaw misalignment — if the IMU is rotated even a few
  //      degrees about Z relative to the chassis, the chassis-Y rotation
  //      vector projects onto BOTH the chip's Y and (slightly) X/Z axes;
  //      conversely a chassis-Z rotation projects onto chip-Y. A 5°
  //      mounting tilt gives ~9% bleed (sin 5°).
  //   2. MPU6050 datasheet cross-axis sensitivity (±2% typical).
  // Both look identical to thetaDot: a transient that tracks the YAW
  // rate (= shared::g.gyroZ). With KthDot>0 the inner loop reacts to
  // this fake thetaDot by spinning the wheels — exactly what we don't
  // want during an in-place rotation.
  //
  // FIX: subtract a scaled copy of the (bias-corrected, raw-axis) gyro
  // Z reading from the (bias-corrected, raw-axis) gyro Y reading
  // BEFORE the IMU_GYRO_PITCH_SIGN flip and BEFORE all the gyroAlpha /
  // notch filtering. Both quantities are in dps, so k is dimensionless.
  //
  // CALIBRATION (manual, no auto-fit yet):
  //   1. Disarm. Lift the bot off the ground (so wheels can't react).
  //   2. Open the web UI; watch the thetaDot trace and the gyroZ trace
  //      side by side (they're both on the rates chart, in deg/s).
  //   3. Rotate the bot bodily about its vertical axis at a steady rate
  //      (~30–90 deg/s is plenty). Don't tilt — pure yaw.
  //   4. With k=0 you'll see thetaDot wobble in correlation with gyroZ.
  //      Increase / decrease gyroYawXTalk in small steps (±0.01) until
  //      thetaDot stays flat during the spin. Sign matters: if positive
  //      gyroZ produces positive thetaDot bleed, k > 0 cancels it.
  // Typical magnitudes: |k| < 0.1 for reasonable mountings; if you need
  // > 0.2 the IMU is probably skewed badly enough to remount.
  // 0 = disabled (default, original behaviour).
  float gyroYawXTalk;     // [-] dimensionless gyZ→gyY bleed coefficient.

  float vWheelAlpha;      // [-] 1-pole IIR mix coefficient on the inner-loop
                          //     output vWheel, applied per control tick AFTER
                          //     saturation, BEFORE setWheelVelocity / the
                          //     vWheelCmd telemetry store. y[n] = α·x[n] +
                          //     (1-α)·y[n-1]. 1.0 = passthrough (default).
                          //     Lower = stronger smoothing of the wheel
                          //     command. Useful when the inner-loop
                          //     KthDot path still injects high-frequency
                          //     content that the upstream gyroAlpha/notch
                          //     filters didn't catch (e.g. quantisation
                          //     from theta integration interacting with
                          //     velFF·targetV). Note: filtering the
                          //     actuator command adds phase lag inside
                          //     the balancing loop, so keep α as close to
                          //     1 as you can get away with — drop only as
                          //     far as needed to kill audible/visible
                          //     stepper chatter.

  // Gyro bias (calibration result).
  //
  // These are not user-tunable in the usual sense — they're populated by
  // imu::calibrateGyro() (triggered from the web UI's "Calibrate IMU"
  // button) and persist via NVS like any other param. Defaults are zero,
  // which means "never calibrated yet" — the bot will work but drift
  // slightly until the user runs a calibration. We deliberately do NOT
  // auto-calibrate at boot: stale-but-known bias from NVS is preferable
  // to a 2 s startup window that fails silently if the chassis is moving.
  //
  // Units: degrees per second (dps), in raw IMU axes (no IMU_GYRO_PITCH_SIGN
  // applied). Subtracted from the raw gyro reading before the sign flip in
  // imu.cpp::imuTask. Only gyroBiasY is currently consumed (pitch axis).
  // X/Z are stored for forward compatibility (future yaw / roll use).
  float gyroBiasX;        // [dps]
  float gyroBiasY;        // [dps]  consumed by the pitch fusion
  float gyroBiasZ;        // [dps]
};

namespace params {

// Returns the compile-time default parameter set (PLAN.md §6 + §13).
ControlParams defaults();

// NVS persistence. Namespace "bb". loadFromNvs() leaves untouched any field
// not present in storage and returns the number of fields actually loaded
// (so the caller can detect a fresh device).
int  loadFromNvs(ControlParams& p);
void saveToNvs(const ControlParams& p);
void clearNvs(); // wipe all keys in the "bb" namespace

// Set a single field by string path. The path is the unqualified field name
// (e.g. "velKp", not "control.velKp"). Returns true if the name was
// recognised and the value applied.
bool setByPath(ControlParams& p, const char* name, float value);

// Serialize the full struct into the given JsonObject. Caller owns the
// document.
void toJson(const ControlParams& p, JsonObject obj);

// Apply any recognised fields from `obj` to `p`. Unknown keys are ignored.
// Returns the number of fields applied.
int fromJson(ControlParams& p, JsonObjectConst obj);

// Format a one-line human summary on the given Print stream (for serial
// banners and debug dumps). Does not append a newline.
void printSummary(const ControlParams& p, ::Print& out);

// Process-wide, mutable, current parameter set. Loaded with defaults() at
// startup; main() then overlays NVS values, and the WS control channel
// mutates fields in place via setByPath() / fromJson(). Concurrent readers
// (controller, IMU filter, motors, telemetry) read individual fields
// without synchronisation — each field is a 32-bit aligned scalar so reads
// are atomic at the hardware level on Xtensa.
extern ControlParams current;

} // namespace params
