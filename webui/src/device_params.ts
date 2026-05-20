// Mirror of the firmware's ControlParams (firmware/src/params.cpp::defaults).
//
// Why a separate type from the simulator's ControlParams:
//   The simulator was designed for a torque-actuated plant (inner PID on
//   angle -> torque + outer PID on velocity -> angle setpoint). The
//   firmware drives steppers, so the actuator IS velocity; the inner loop
//   is a direct (Kth, KthDot, velFF) blend rather than a PID. The two
//   schemas overlap in only five fields (vel{Kp,Ki,Kd,IClamp},
//   maxAngleSetpoint), so trying to share a single TypeScript type would
//   either lose fields or invent fictional ones on each side.
//
//   In Device mode the UI swaps to this schema, populated from the
//   {type:"params"} message the firmware pushes on every WS connect.
//
// Defaults below mirror firmware/src/params.cpp::defaults() so the panel
// renders sensibly *before* the device replies (and is visually identical
// to a freshly-flashed device).

import { GroupSpec } from "./ui";

export interface DeviceParams {
  // Outer (vel -> angle setpoint)
  velKp: number;
  velKi: number;
  velKd: number;
  velErrDeadband: number;  // [m/s] symmetric dead zone on outer-loop err; |err|<this → err=0 for the tick
  velIClamp: number;
  maxAngleSetpoint: number;
  velDAlpha: number;     // [-] 1-pole IIR alpha on outer D term (post-velKd), 0..1; 1 = passthrough
  outerEnabled: number;  // [bool 0/1] master switch for the outer (vel→angle) PID; 1 = on (default), 0 = held in reset
  velPAlpha: number;     // [-] 1-pole IIR alpha on outer P term (post-velKp), 0..1; 1 = passthrough
  outerAlpha: number;    // [-] 1-pole IIR alpha on composite thetaSet (post P+I+D + clamp), 0..1; 1 = passthrough
  outerEveryN: number;   // [ticks ≥1] outer-loop downsample factor; 1 = run every tick (inner rate), 4 → outer at 50 Hz vs inner at 200 Hz
  // Inner (angle, angleDot -> wheel velocity command)
  innerEnabled: number;  // [bool 0/1] master switch for the inner (tilt→vWheel) PD; 1 = on (default), 0 = drops Kth/KthDot tilt-feedback (velFF survives)
  Kth: number;
  KthDot: number;
  velFF: number;
  targetVAlpha: number;  // [-] 1-pole IIR alpha on raw shared targetV before controller, 0..1; 1 = passthrough
  targetTurnAlpha: number;  // [-] 1-pole IIR alpha on raw shared targetTurn before controller, 0..1; 1 = passthrough
  // Stick response curve (applied to UART-coproc joystick input in
  // firmware/src/joystick.cpp). Shared between leftY → targetV and
  // rightX → targetTurn paths so both axes feel consistent.
  stickDeadband: number;  // [-] normalized half-width 0..0.95 of the dead zone
  stickExpo: number;      // [-] response-curve exponent; 1=linear, 2+=expo feel
  thetaTrim: number;
  // Limits
  vMaxCart: number;
  vMaxWheel: number;
  vMaxTurn: number;     // m/s; cap on steering differential
  aMaxWheel: number;
  fallAngle: number;     // rad
  vBatCutoff: number;    // V
  // Auto-arm
  autoArmEnabled: number;  // [bool 0/1]
  autoArmAngle: number;    // [rad] window half-width around thetaTrim
  autoArmHoldMs: number;   // [ms] dwell time inside the window before auto-arm
  // Driver
  microsteps: number;
  runCurrent: number;    // A
  holdCurrent: number;   // A
  // Filter
  compAlpha: number;     // [-] complementary filter blend, 0..1
  accAlpha: number;      // [-] 1-pole IIR alpha on thetaAcc before comp filter, 0..1; 1 = passthrough
  gyroAlpha: number;     // [-] 1-pole IIR alpha on gyro->thetaDot, 0..1; 0 = passthrough
  notchFc: number;       // Hz; biquad notch on thetaDot. 0 = disabled.
  notchQ: number;        // notch quality factor (3–10 typical).
  vWheelAlpha: number;   // [-] 1-pole IIR alpha on vWheel command, 0..1; 1 = passthrough
  // Gyro yaw→pitch cross-talk compensation. Subtracts k * gyroZ from
  // the raw gyroY reading inside the IMU task to cancel the small fake
  // pitch rate that mounting yaw misalignment + datasheet ±2%
  // cross-axis sensitivity bleed when the bot rotates in place. 0
  // disables (default). See params.h::gyroYawXTalk for the bench
  // calibration procedure.
  gyroYawXTalk: number;
  // Gyro bias (calibration result). Populated by the device's
  // {type:"calibrate"} handler and persisted to NVS like any other
  // param. NOT rendered in the UI — see deviceGroups below — because
  // it's a calibration result, not a tunable. Kept in the type so the
  // {type:"params"} broadcast assigns cleanly.
  gyroBiasX: number;     // dps
  gyroBiasY: number;     // dps (only the pitch axis is consumed by the firmware)
  gyroBiasZ: number;     // dps
}

export function defaultDeviceParams(): DeviceParams {
  return {
    velKp: 0.10,
    velKi: 0.05,
    velKd: 0.00,
    velErrDeadband: 0.0,
    velIClamp: 0.15,
    maxAngleSetpoint: 0.25,
    velDAlpha: 0.20,
    outerEnabled: 1.0,
    velPAlpha: 1.0,
    outerAlpha: 1.0,
    outerEveryN: 1.0,
    innerEnabled: 1.0,
    Kth: 0.0,
    KthDot: 0.0,
    velFF: 1.0,
    targetVAlpha: 1.0,
    targetTurnAlpha: 1.0,
    stickDeadband: 0.05,
    stickExpo: 2.0,
    thetaTrim: 0.0,
    vMaxCart: 0.8,
    vMaxWheel: 1.5,
    vMaxTurn: 0.5,
    aMaxWheel: 5.0,
    fallAngle: 0.61,
    vBatCutoff: 13.2,
    autoArmEnabled: 1.0,
    autoArmAngle: 0.01745,
    autoArmHoldMs: 500,
    microsteps: 16,
    runCurrent: 1.4,
    holdCurrent: 0.7,
    compAlpha: 0.01,
    accAlpha: 1.0,
    gyroAlpha: 0.44,
    notchFc: 0.0,
    notchQ: 5.0,
    vWheelAlpha: 1.0,
    gyroYawXTalk: 0.0,
    gyroBiasX: 0.0,
    gyroBiasY: 0.0,
    gyroBiasZ: 0.0,
  };
}

const DEG = 180 / Math.PI;

// Group schema for the device panel. Paths are bare field names because
// DeviceParams is flat (unlike the simulator's nested Params). The shared
// row renderer (ui.ts::buildPanelGroups) walks the path with split(".") so
// a single segment Just Works.
export const deviceGroups: GroupSpec[] = [
  {
    title: "Outer (vel → angle)",
    rows: [
      { kind: "bool", label: "enabled", path: "outerEnabled", numeric: true,
        tooltip: "Master switch for the outer velocity→angle-setpoint PID. ON (default) = normal cascaded balancing: the outer loop converts the commanded velocity into a tilt setpoint that the inner PD chases. OFF = the outer loop is held in reset (thetaSet=0, P/I/D=0, integrator and ramp filters all zeroed) and targetVUsed=0 so the velFF·targetV term contributes nothing either; the inner PD continues to run as a pure tilt corrector around upright. The bot will still balance with this off — it just won't try to track a commanded velocity. Useful for one-click isolation of inner-vs-outer behaviour while tuning. Stored as a float on the firmware (1=on, 0=off)." },
      { kind: "num", label: "Kp",        path: "velKp",            opts: { min: 0, max: 2,    step: 0.005, decimals: 3 },
        tooltip: "Outer P gain. Velocity error → angle setpoint. Higher = bot leans harder to chase the commanded velocity. Too high → oscillation in target tracking." },
      { kind: "num", label: "Ki",        path: "velKi",            opts: { min: 0, max: 2,    step: 0.005, decimals: 3 },
        tooltip: "Outer I gain. Integrates velocity error to bias the angle setpoint, removing steady-state offset (slope, asymmetric trim). Too high → slow oscillation, wind-up." },
      { kind: "num", label: "Kd",        path: "velKd",            opts: { min: -1, max: 1,    step: 0.0001, decimals: 4 },
        tooltip: "Outer D gain. Velocity error rate → angle setpoint. Usually 0 — outer loop runs slowly enough that D rarely helps. Negative values allowed for sign-flip experiments." },
      { kind: "num", label: "err deadband", path: "velErrDeadband", opts: { min: 0, max: 0.5, step: 0.005, unit: "m/s", decimals: 3 },
        tooltip: "Symmetric dead zone on the outer-loop velocity error (err = targetV - xDotEst). When |err| < this, err is rounded to 0 BEFORE any P/I/D computation, so P contributes nothing, the integrator step (velKi·err·outerDt) is zero and dErr derives from a zeroed err. Use to kill the endless tiny corrections the outer loop otherwise produces when xDotEst flickers between adjacent LEDC step rates in standstill. 0 = disabled (default, original behaviour). Typical tuning starts around 0.02–0.05 m/s — raise until the bot stops twitching, lower until genuine velocity tracking still feels responsive. Note: dead zone is a hard 'round to zero' rather than the smoother 'subtract band edge' form, so there's a small dErr spike at the band boundary — usually invisible because velKd is small or zero, but worth knowing." },
      { kind: "num", label: "I clamp",   path: "velIClamp",        opts: { min: 0, max: 30,   step: 0.25, unit: "°", decimals: 2, displayMul: DEG },
        tooltip: "Saturation on the outer integrator's contribution to the angle setpoint (degrees of lean). Prevents wind-up." },
      { kind: "num", label: "Max set θ", path: "maxAngleSetpoint", opts: { min: 0.5, max: 35, step: 0.25, unit: "°", decimals: 2, displayMul: DEG },
        tooltip: "Hard cap on the commanded lean angle. Limits how aggressively the outer loop can demand a tilt." },
      { kind: "num", label: "D α",       path: "velDAlpha",        opts: { min: 0, max: 1, step: 0.01, decimals: 3 },
        tooltip: "1-pole IIR on the outer D term, applied AFTER multiplying by velKd and BEFORE summing with P+I into thetaSet. 1.0 = passthrough. Lower = stronger smoothing of the D contribution at the cost of phase lag in the velocity-tracking loop. Mirrors the velPAlpha shape on P (one knob per term, same units, same reset semantics). Differentiation itself is on raw err: dErr = (err - errPrev)/outerDt; this post-gain IIR is what tames noise. Replaces the old velDLowpassHz Hz-cutoff form (8 Hz default at outerDt=0.005 s ≈ α=0.20, which is the new default here so behaviour matches on flash). Effective sample period of the recurrence is outerDt = N·DT (filter step runs only on outer-run ticks; skip ticks reuse heldOuterD). Filter state resets to 0 on disarm / !imuOk / outerEnabled=0." },
      { kind: "num", label: "P α",       path: "velPAlpha",        opts: { min: 0, max: 1, step: 0.01, decimals: 3 },
        tooltip: "1-pole IIR on the outer P term, applied AFTER multiplying by velKp and BEFORE summing with I+D into thetaSet. 1.0 = passthrough (default). Lower = smoother P contribution to thetaSet at the cost of phase lag in the velocity-tracking loop. Useful when xDotEst quantisation (or any noise on the velocity estimate) shows up as jitter on thetaSet that the inner loop then chases. Filter state resets to 0 on each arm." },
      { kind: "num", label: "outer α",   path: "outerAlpha",       opts: { min: 0, max: 1, step: 0.01, decimals: 3 },
        tooltip: "1-pole IIR on the COMPOSITE outer-loop output thetaSet, applied AFTER summing P+I+D and AFTER the maxAngleSetpoint clamp, BEFORE handing the value to the inner loop and BEFORE latching into heldThetaSet. 1.0 = passthrough (default). Lower = smoother thetaSet trace in one place — covers quantisation/noise on any of P, I, D and any interaction between them, distinct from velPAlpha which only filters P. Particularly useful at outerEveryN > 1 where the outer block re-evaluates rarely and a noisy run-tick value would otherwise be held verbatim for N-1 inner ticks. Filtering AFTER the clamp keeps the filter state bounded to ±maxAngleSetpoint. Filter step runs only on outer-run ticks; skip ticks carry the held value through naturally, so effective sample period is outerDt = N·DT. State resets to 0 on disarm / !imuOk / outerEnabled=0." },
      { kind: "num", label: "every N",   path: "outerEveryN",      opts: { min: 1, max: 50, step: 1, decimals: 0 },
        tooltip: "Outer-loop downsample factor (ticks). The outer (velocity→angle-setpoint) PID re-evaluates only every N control ticks; the inner PD continues to run every tick at CONTROL_LOOP_HZ=200. N=1 (default) = outer at 200 Hz, same rate as inner — legacy behaviour. N=4 → outer at 50 Hz, inner at 200 Hz, which is the typical cascaded arrangement when the outer dynamics (cart velocity) are slower than the inner (tilt) — and which prevents xDotEst quantisation noise from being amplified into thetaSet at the full inner rate. Between runs, the last computed thetaSet (and the published outerP/I/D) are held so the inner loop sees a clean zero-order-hold setpoint. The integrator step (velKi·err·dt) and the dErr divisor automatically scale to dt = N·DT so velKi/velKd keep their tuned meaning regardless of N. State and counter reset on disarm / outerEnabled flip, so the first armed-and-enabled tick always runs the outer block immediately." },
    ],
  },
  {
    title: "Inner (θ → vWheel)",
    rows: [
      { kind: "bool", label: "enabled", path: "innerEnabled", numeric: true,
        tooltip: "Master switch for the inner tilt→wheel-velocity PD. ON (default) = normal balancing: Kth·θerr + KthDot·θ̇ + velFF·targetV → wheel command. OFF = the Kth and KthDot tilt-feedback terms are dropped; only the velFF·targetV feed-forward survives, so the bot becomes an open-loop velocity drive controlled by the joystick (steering split, vWheelAlpha filter and per-wheel saturation still applied as usual). The bot will NOT balance with this off — the inner PD is the tilt corrector. This is a bench-test mode for driving the wheels directly to diagnose motor / steering / step-skip / resonance behaviour without rebuilding firmware. Symmetric mirror of the outer 'enabled' toggle. WARNING: auto-arm doesn't know about this flag — disarm or disable auto-arm before flipping it on a near-upright bot. Stored as a float on the firmware (1=on, 0=off)." },
      { kind: "num", label: "Kth",     path: "Kth",       opts: { min: 0, max: 50, step: 0.05, decimals: 2 },
        tooltip: "Inner P-equivalent: tilt error → wheel velocity (m/s per rad). The dominant balancing gain. Too low = bot falls; too high = high-frequency twitch." },
      { kind: "num", label: "KthDot",  path: "KthDot",    opts: { min: -10, max: 10, step: 0.01, decimals: 2 },
        tooltip: "Inner D-equivalent: tilt rate → wheel velocity (m/s per rad/s). Damps oscillation around upright. Increase if the bot wobbles when Kth alone is tuned. Negative values allowed for sign-flip experiments (will destabilise the slow pendulum mode if the firmware sign chain is correct — see the IMU_GYRO_PITCH_SIGN diagnostic)." },
      { kind: "num", label: "vel FF",  path: "velFF",     opts: { min: 0, max: 2,  step: 0.01, decimals: 2 },
        tooltip: "Velocity feedforward: passes the outer-loop targetV directly into the wheel command. 1.0 = full FF (recommended for steppers, where the actuator IS velocity)." },
      { kind: "num", label: "θ trim",  path: "thetaTrim", opts: { min: -10, max: 10, step: 0.05, unit: "°", decimals: 2, displayMul: DEG },
        tooltip: "IMU mounting offset. Reported θ = measured − trim. Use the 'Capture trim' button to set it automatically while holding the bot upright." },
    ],
  },
  {
    title: "Limits",
    rows: [
      { kind: "num", label: "v max cart",  path: "vMaxCart",   opts: { min: 0, max: 3,  step: 0.05, unit: "m/s", decimals: 2 },
        tooltip: "Soft cap on cart-frame velocity command (joystick / outer-loop output)." },
      { kind: "num", label: "v max wheel", path: "vMaxWheel",  opts: { min: 0, max: 5,  step: 0.05, unit: "m/s", decimals: 2 },
        tooltip: "Hard cap on per-wheel commanded velocity. Saturated inside the motors layer before reaching LEDC." },
      { kind: "num", label: "v max turn",  path: "vMaxTurn",   opts: { min: 0, max: 3,  step: 0.05, unit: "m/s", decimals: 2 },
        tooltip: "Cap on the steering differential added to/subtracted from each wheel after the vWheelAlpha filter. Positive shared targetTurn = left wheel faster = bot turns right. Should be < v max wheel so the differential can't swallow the entire wheel budget. Per-wheel total is still clamped to ±v max wheel as a final safety net." },
      { kind: "num", label: "a max wheel", path: "aMaxWheel",  opts: { min: 0, max: 30, step: 0.25, unit: "m/s²", decimals: 2 },
        tooltip: "Slew-rate limit on commanded wheel velocity, enforced in motors::setWheelVelocity. Caps |dV/dt| to bound peak current draw and prevent positive-feedback chatter from inner D / outer D loops. 0 disables." },
      { kind: "num", label: "fall angle",  path: "fallAngle",  opts: { min: 5, max: 60, step: 0.5,  unit: "°", decimals: 1, displayMul: DEG },
        tooltip: "Tilt magnitude that triggers a safety disarm. Beyond this, the FSM drops EN and stops the wheels." },
      { kind: "num", label: "vBat cutoff", path: "vBatCutoff", opts: { min: 0, max: 20, step: 0.1,  unit: "V", decimals: 1 },
        tooltip: "Battery low-voltage cutoff. Below this the safety FSM disarms to protect the LiPo." },
    ],
  },
  {
    title: "Auto-arm",
    rows: [
      { kind: "bool", label: "enabled",  path: "autoArmEnabled", numeric: true,
        tooltip: "Master switch for auto-arm. When on, the safety FSM transitions READY → ARMED automatically once the bot has been balanced near upright for the configured dwell time. The 'Arm' button still works either way. Stored as a float on the firmware (1=on, 0=off); the toggle writes 1/0." },
      { kind: "num", label: "angle",    path: "autoArmAngle", opts: { min: 0, max: 10, step: 0.05, unit: "°", decimals: 2, displayMul: DEG },
        tooltip: "Half-width of the upright window. Auto-arm requires |theta - thetaTrim| ≤ this angle for the full dwell time. Default 1°. Larger = arms more easily but tolerates more pre-arm tilt error." },
      { kind: "num", label: "hold",     path: "autoArmHoldMs", opts: { min: 0, max: 5000, step: 50, unit: "ms", decimals: 0 },
        tooltip: "Continuous dwell time the trim-corrected tilt must stay inside the window before auto-arming. Default 500 ms. Too short = arms during a stray hand-bump near upright; too long = clumsy to engage." },
    ],
  },
  {
    title: "Driver (TMC2209)",
    rows: [
      // Microsteps is an int on the firmware side; the JSON setParam
      // path coerces via lroundf so any integer (1, 2, 4, 8, 16, 32, …)
      // is accepted. We render it as a number with step=1.
      { kind: "num", label: "microsteps",  path: "microsteps",   opts: { min: 1, max: 256, step: 1, decimals: 0 },
        tooltip: "TMC2209 microsteps per full step (1, 2, 4, 8, 16, 32, 64, 128, 256). Higher = smoother motion but more STEP edges per m/s. 16 is a good default." },
      { kind: "num", label: "run current", path: "runCurrent",   opts: { min: 0, max: 2.5, step: 0.05, unit: "A", decimals: 2 },
        tooltip: "Motor RMS current while stepping (sets IRUN). Stay at or below the motor's rated current and watch for warmth." },
      { kind: "num", label: "hold curr.",  path: "holdCurrent",  opts: { min: 0, max: 2.5, step: 0.05, unit: "A", decimals: 2 },
        tooltip: "Motor RMS current at standstill (sets IHOLD via IHOLD/IRUN ratio). Lower = cooler when stationary; too low = the bot can't hold position." },
    ],
  },
  {
    title: "Filter",
    rows: [
      { kind: "num", label: "acc α",       path: "accAlpha", opts: { min: 0, max: 1, step: 0.01, decimals: 3 },
        tooltip: "Raw accelerometer theta filter. 1.0 = passthrough. Lower = more filter." },
      { kind: "num", label: "gyro α",      path: "gyroAlpha", opts: { min: 0, max: 1, step: 0.01, decimals: 3 },
        tooltip: "Raw gyro thetaDot filter. 1 = passthrough. Lower = more filter." },
      { kind: "num", label: "comp α",      path: "compAlpha", opts: { min: 0, max: 1, step: 0.001, decimals: 4 },
        tooltip: "Complementary-filter blend: 0 = more integrated from gyro; 1 = more directly from accelerometer" },
      { kind: "num", label: "notch fc",    path: "notchFc",      opts: { min: 0, max: 50, step: 0.5, unit: "Hz", decimals: 1 },
        tooltip: "Biquad notch centre frequency on thetaDot, applied AFTER the gyroAlpha LPF and BEFORE the controller sees it. Targets a known structural-resonance / limit-cycle peak (e.g. ~7 Hz observed on the bench). 0 disables. Phase response is well-behaved away from fc, so unlike a broad LPF this should NOT add meaningful lag at the controller's working frequencies." },
      { kind: "num", label: "notch Q",     path: "notchQ",       opts: { min: 0.5, max: 30, step: 0.5, decimals: 1 },
        tooltip: "Notch quality factor. Larger Q = narrower notch. Typical 3–10. Too high = brittle against drift in the resonant frequency; too low = swallows useful bandwidth. Ignored when notch fc ≤ 0." },
      { kind: "num", label: "vWheel α",    path: "vWheelAlpha",  opts: { min: 0, max: 1, step: 0.01, decimals: 3 },
        tooltip: "1-pole IIR on the inner-loop vWheel command, applied AFTER saturation and BEFORE the motors driver. 1.0 = passthrough (no filter). Lower = stronger smoothing of the actuator command, useful to kill audible/visible stepper chatter. WARNING: filtering the actuator output adds phase lag inside the balancing loop, which directly hurts stability margin — keep α as close to 1 as you can get away with." },
      { kind: "num", label: "yaw→pitch k", path: "gyroYawXTalk", opts: { min: -0.5, max: 0.5, step: 0.005, decimals: 4 },
        tooltip: "Cross-talk compensation: subtracts k·gyroZ from raw gyroY before all filtering, cancelling the fake thetaDot bleed when the bot rotates in place (mounting yaw misalignment + datasheet cross-axis sensitivity). 0 = disabled. CALIBRATE: lift the bot, rotate it bodily about its vertical axis at ~30–90 °/s without tilting, watch thetaDot vs ψ̇ on the rates chart, dial k in ±0.005 steps until thetaDot stops correlating with ψ̇. Sign matters: if positive ψ̇ produces positive thetaDot bleed, k > 0 cancels it. Typical |k| < 0.1; |k| > 0.2 means remount the IMU." },
    ],
  },
];
