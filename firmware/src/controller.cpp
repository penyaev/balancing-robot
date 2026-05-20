// controller.cpp — see controller.h.
//
// 200 Hz cascaded controller. The math mirrors PLAN.md §2; differences from
// the simulator (which uses torque) are called out inline.
//
// Layout per step:
//   1. Wait for the next 5 ms tick (vTaskDelayUntil — drift-free).
//   2. Snapshot inputs from shared::g (theta, thetaDot, targetV) and from
//      params::current (gains, limits). Both are read field-by-field; each
//      field is a 32-bit aligned scalar so reads are torn-write-free.
//   3. Outer PID:
//        err = target_v − x_dot_estimate  (x_dot = last commanded vWheel)
//        P = velKp · err
//        I += velKi · err · dt           (clamped to ±velIClamp)
//        dErr = (err − errPrev) / dt              (raw; velDAlpha LPF on D)
//        D = velKd · dErr
//        θ_set = clamp(P + I + D, ±maxAngleSetpoint)
//   4. Inner PD+FF (per PLAN.md §2):
//        v_wheel = velFF·target_v + Kth·(θ−θ_set−thetaTrim) + KthDot·θ̇
//      Saturate to ±vMaxWheel.
//   5. Push to motors driver (only when FLAG_MOTORS_ENABLED is set; else
//      stop motors and zero the integrator to prevent windup).
//   6. Publish controller outputs back to shared::g so telemetry (F11) can
//      stream them.
//
// Until imu::isReady() returns true (boot-time gyro bias calibration is
// still running), we hold all outputs at zero and do nothing else. The IMU
// task flips that flag after the first complementary-filter sample is out.

#include "controller.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "imu.h"
#include "motors.h"
#include "params.h"
#include "shared_state.h"

namespace controller {
namespace {

constexpr BaseType_t CORE_CONTROL = 1;
constexpr float DT = 1.0f / static_cast<float>(CONTROL_LOOP_HZ); // 0.005 s

bool running = false;

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline float sign(float v) {
  if (v < 0) {
    return -1.0f;
  }
  return 1.0f;
}

void controlTask(void* /*arg*/) {
  // Outer-PID state, owned exclusively by this task.
  float integ = 0.0f;       // velKi * sum(err)*dt; clamped per tick
  // Raw err differentiation state. Differentiation is unfiltered now
  // (the post-gain velDAlpha IIR below tames noise, mirroring the
  // velPAlpha shape on the P term). errPrev holds the previous run-
  // tick's err so dErr = (err - errPrev) / outerDt; errPrimed gates
  // the first run after reset so we don't compute dErr against a
  // bogus 0.
  float errPrev = 0.0f;
  bool  errPrimed = false;

  // 1-pole IIR state for velDAlpha. Filter is applied to the post-gain
  // D term (velKd · dErr), in the same shape and at the same point as
  // velPAlpha is applied to P. α=1 (default) collapses to passthrough.
  // Seed from 0 (not raw D) so each arm starts the D contribution at
  // rest — mirrors targetVFilt / pFilt / thetaSetFilt soft-start
  // pattern. Reset on disarm / !imuOk / !outerOn.
  float dFilt = 0.0f;
  bool  dSeed = false;

  // Cached x_dot estimate. Steppers don't slip, so commanded ≈ actual.
  float xDotEst = 0.0f;

  // 1-pole IIR state for vWheelAlpha. Filter is applied AFTER saturation,
  // so we can never integrate above ±vMaxWheel and a sudden disarm/arm
  // edge re-seeds from the post-clamp value below. seed flag mirrors the
  // pattern used in imu.cpp's pre-fusion filters: first sample bypasses
  // the recurrence so we don't ramp up from 0 on startup.
  float vWheelFilt = 0.0f;
  bool  vWheelSeed = false;

  // 1-pole IIR state for targetVAlpha. Filter is applied to the raw
  // shared::g.targetV before the controller consumes it (both outer-loop
  // err and inner-loop velFF·targetV see the filtered value). Reset to
  // 0 on disarm / !imuOk so every arm ramps from 0 toward the current
  // joystick value — even if the operator was already pushing the stick
  // before clicking arm, the bot starts from rest and accelerates
  // smoothly. α=1 (default) collapses the recurrence to passthrough.
  float targetVFilt = 0.0f;
  bool  targetVSeed = false;

  // 1-pole IIR state for targetTurnAlpha. Same shape and reset rules as
  // targetVFilt above, but applied to shared::g.targetTurn before the
  // steering differential is split across the wheels. Independent of the
  // forward-velocity ramp so steering and forward response are tunable
  // separately. α=1 (default) collapses to passthrough.
  float targetTurnFilt = 0.0f;
  bool  targetTurnSeed = false;

  // 1-pole IIR state for velPAlpha. Filter is applied to the outer-loop
  // P term AFTER multiplying by velKp and BEFORE summing with I+D into
  // thetaSet. Resets to 0 on disarm / !imuOk so each arm starts the P
  // contribution from rest, mirroring the integrator and dErr LPF
  // resets. α=1 (default) collapses to passthrough. Diagnostic intent:
  // xDotEst quantisation jitter feeds err → P → thetaSet → inner loop;
  // a small dose of smoothing on P alone lets you decouple that from
  // I/D without touching the outer P gain.
  float pFilt = 0.0f;
  bool  pSeed = false;

  // 1-pole IIR state for outerAlpha. Filter is applied to the COMPOSITE
  // outer-loop output thetaSet AFTER the P+I+D sum and AFTER the
  // maxAngleSetpoint clamp, BEFORE we latch heldThetaSet and hand the
  // value to the inner loop. α=1 (default) collapses to passthrough.
  // Seeding from 0 (not raw) mirrors the targetVFilt / pFilt soft-start
  // pattern: every arm begins with the composite setpoint at rest, which
  // auto-arm (only fires within ±autoArmAngle) handles cleanly. The
  // filter step is taken only on outer-run ticks; on skip ticks the
  // held value carries through unchanged via the heldThetaSet path, so
  // the effective sample period of the recurrence is outerDt = N·DT —
  // same as the integrator and the dErr LPF. State is reset to 0 on
  // disarm / !imuOk / !outerOn alongside the rest of the outer-loop
  // reset convention. See params.h::outerAlpha.
  float thetaSetFilt = 0.0f;
  bool  thetaSetSeed = false;

  // Outer-loop downsample state (see params.h::outerEveryN).
  //
  // outerTickCount counts how many control ticks have elapsed since the
  // last counter reset (disarm / !imuOk / !outerOn). The outer block
  // executes only on ticks where (outerTickCount % N) == 0, with N =
  // cp.outerEveryN clamped to ≥1; on all other ticks the previously-
  // computed outer outputs are held and fed to the inner loop unchanged.
  // Counter is reset to 0 on every "off" path so the first armed-and-
  // enabled tick always runs the outer block immediately (no random
  // phase relative to the operator's arm click).
  //
  // The held* variables are the post-clamp outer outputs from the most
  // recent run tick. They are what the inner loop sees and what the
  // controller publishes to shared::g.outerP/I/D and thetaSet on every
  // tick — both run and skip — so telemetry shows a clean zero-order-
  // hold trace instead of glitches at the downsample boundary. Reset
  // to 0 on disarm / !imuOk / !outerOn alongside the integrator and the
  // ramp/LPF filter states (consistent with the rest of this task's
  // outer-loop reset convention).
  uint32_t outerTickCount  = 0;
  float    heldOuterP      = 0.0f;
  float    heldOuterI      = 0.0f;
  float    heldOuterD      = 0.0f;
  float    heldThetaSet    = 0.0f;
  float    heldTargetVUsed = 0.0f;

  // Periodic log scaffolding. Heartbeat every 10 s; arm/disarm transitions
  // log immediately (edge-triggered below).
  uint32_t logCounter = 0;
  constexpr uint32_t LOG_PERIOD = 10 * CONTROL_LOOP_HZ; // 10 s
  bool prevArmed = false;

  TickType_t nextWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ));

    // -- Snapshot inputs ---------------------------------------------------
    const ControlParams& cp = params::current; // reads happen field-by-field
    const float targetV = shared::g.targetV.load();
    const float theta   = shared::g.theta.load();
    const float thDot   = shared::g.thetaDot.load();
    const bool  imuOk   = imu::isReady();
    const bool  armed   = shared::getFlag(shared::FLAG_MOTORS_ENABLED);
    // Outer-loop master switch. When 0, the outer PID is held in reset
    // exactly the way it is during disarm: thetaSet, P/I/D, integ, dErr
    // LPF, targetV ramp filter and the outer-P alpha filter all stay at
    // 0; targetVUsed=0 so the inner-loop velFF·targetV term contributes
    // nothing. The inner PD still runs and acts as a pure tilt
    // corrector around upright. Useful for one-click isolation of inner
    // vs outer behaviour from the web UI without a code change. Stored
    // as a float (mirrors autoArmEnabled); >0.5 = on.
    const bool  outerOn = cp.outerEnabled > 0.5f;
    // Inner-loop master switch. When 0, the Kth·thetaErr and KthDot·thDot
    // tilt-feedback terms are dropped from the wheel-command formula
    // below; only velFF·targetVUsed survives. Steering split,
    // vWheelAlpha filter and per-wheel saturation still apply, so the
    // bot becomes an open-loop velocity drive controlled by the
    // joystick — the natural mirror of outerOn (which keeps the inner
    // PD running as a pure tilt corrector). The bot will NOT balance
    // with this off; it's a bench-test mode for driving wheels
    // directly to diagnose motor / steering / step-skip behaviour
    // without the inner PD chasing tilt. There is no inner-PD
    // integrator to reset, and vWheelFilt continues from its current
    // value — toggling innerOn mid-arm ramps smoothly through the IIR.
    const bool  innerOn = cp.innerEnabled > 0.5f;

    if (!imuOk) {
      // IMU bias calibration still in progress (or IMU dropped out).
      // Hold outputs zero, keep integrator clean. Don't touch motors —
      // motors::start() left them stopped; we don't want to start spamming
      // setSpeedInHz with something nonsensical.
      shared::g.thetaSet.store(0.0f);
      shared::g.vWheelCmd.store(0.0f);
      shared::g.vWheelCmdL.store(0.0f);
      shared::g.vWheelCmdR.store(0.0f);
      shared::g.xDotEst.store(0.0f);
      shared::g.outerP.store(0.0f);
      shared::g.outerI.store(0.0f);
      shared::g.outerD.store(0.0f);
      // targetTurnUsed mirrors the held-in-reset state of the
      // controller's ramp filter while the IMU isn't ready; clear it
      // here so the UI doesn't show a stale post-filter turn value
      // during the calibration window.
      shared::g.targetTurnUsed.store(0.0f);
      integ = 0.0f;
      errPrev = 0.0f;
      errPrimed = false;
      dFilt = 0.0f;
      dSeed = false;
      // Reset the targetV ramp filter too: a calibration window can take
      // 2 s during which the joystick may have moved, and we don't want
      // the controller's first armed tick to inherit a stale ramp state.
      targetVFilt = 0.0f;
      targetVSeed = false;
      targetTurnFilt = 0.0f;
      targetTurnSeed = false;
      pFilt = 0.0f;
      pSeed = false;
      thetaSetFilt = 0.0f;
      thetaSetSeed = false;
      // Outer downsample state: zero held outputs and reset the counter
      // so the first armed tick after IMU readiness runs the outer block
      // immediately (matches the reset convention for every other piece
      // of outer-loop state in this branch).
      outerTickCount  = 0;
      heldOuterP      = 0.0f;
      heldOuterI      = 0.0f;
      heldOuterD      = 0.0f;
      heldThetaSet    = 0.0f;
      heldTargetVUsed = 0.0f;
      continue;
    }

    // -- Outer PID (vel → tilt setpoint) ----------------------------------
    // Only runs while armed. Reasoning: when the bot is sitting in the
    // hand or on the bench (disarmed), xDotEst is 0 by definition (motors
    // stopped) but targetV may be nonzero from a stick deflection or just
    // the operator brushing the joystick. A live outer PID would respond
    // to that with a tilt setpoint, which is then visible to the inner
    // loop and to telemetry — making it look like the bot "thinks it's
    // moving" even though nothing is driving the wheels. Holding the
    // outer loop in reset while disarmed keeps thetaSet at 0 so the inner
    // PD (which still runs) is purely a tilt corrector ready to engage
    // the moment auto-arm fires. Mirrors the !imuOk reset above.
    //
    // Downsample: the outer block re-evaluates every cp.outerEveryN
    // control ticks (≥1 enforced via lroundf+max). On non-run ticks the
    // most recent outer outputs are reused unchanged, so the inner loop
    // sees a clean zero-order-hold setpoint. outerDt = N·DT scales the
    // integrator step and the dErr divisor so velKi/velKd keep their
    // tuned meaning regardless of N. See params.h::outerEveryN.
    int outerEvery = static_cast<int>(lroundf(cp.outerEveryN));
    if (outerEvery < 1) outerEvery = 1;
    const bool  outerRunTick = (outerTickCount % static_cast<uint32_t>(outerEvery)) == 0;
    const float outerDt      = DT * static_cast<float>(outerEvery);
    outerTickCount++;

    // Per-tick consumer values default to the held outputs from the last
    // run tick, so a skip tick feeds the inner loop the same setpoint
    // and publishes the same outerP/I/D as the previous run tick. On a
    // run tick below these will be overwritten with freshly-computed
    // values and the held* state will be updated. On the disarm/!outerOn
    // paths the held state is zero, so these are zero too.
    float P = heldOuterP;
    float I = heldOuterI;
    float D = heldOuterD;
    float thetaSet = heldThetaSet;
    float targetVUsed = heldTargetVUsed;
    if (armed && outerOn && outerRunTick) {
      // Apply the targetV ramp filter BEFORE the loop sees it. α=1
      // (default) is passthrough; lower values make commanded velocity
      // changes ramp into the controller instead of stepping. The seed
      // branch initialises from 0 (not raw) so that a fresh arm always
      // ramps from rest — by construction targetVFilt is already 0 from
      // the disarm/!imuOk reset below, but the explicit seed=false makes
      // the first armed tick deterministic regardless of how we got here.
      float tvAlpha = cp.targetVAlpha;
      if (tvAlpha < 0.0f) tvAlpha = 0.0f;
      if (tvAlpha > 1.0f) tvAlpha = 1.0f;
      if (!targetVSeed) {
        targetVFilt = 0.0f;
        targetVSeed = true;
      }
      targetVFilt = tvAlpha * targetV + (1.0f - tvAlpha) * targetVFilt;
      targetVUsed = targetVFilt;

      float err = targetVUsed - xDotEst;
      // Outer-loop error deadband. When |err| is small, round it to zero
      // for this tick so the outer PID emits no correction at all: P=0,
      // the integrator step (velKi·err·outerDt) is zero, and dErr derives
      // from a zeroed err. This kills the endless micro-corrections the
      // outer loop otherwise produces when xDotEst flickers in standstill.
      // velErrDeadband=0 (default) is a no-op — same as before. We compare
      // against a clamped value defensively so a stray negative setParam
      // can't enable a "negative deadband" that would suppress nothing.
      const float dbThresh =
          (cp.velErrDeadband > 0.0f) ? cp.velErrDeadband : 0.0f;
      if (fabsf(err) < dbThresh) err = 0.0f;

      P = cp.velKp * err;

      // 1-pole IIR on the post-gain P term. α=1 (default) is
      // passthrough; lower α smooths xDotEst-quantisation jitter that
      // would otherwise pass straight through err·velKp into thetaSet.
      // Filter the *post-gain* P (not the err input) so what we publish
      // to shared::g.outerP and what we sum into thetaSet are the same
      // value the controller actually used. Seeding from 0 (not raw P)
      // mirrors the targetVFilt / vWheelFilt soft-start pattern: every
      // arm begins with the P contribution at rest, which auto-arm
      // (only fires within ±autoArmAngle) handles cleanly. Clamp α
      // defensively against bad setParam values.
      float vpAlpha = cp.velPAlpha;
      if (vpAlpha < 0.0f) vpAlpha = 0.0f;
      if (vpAlpha > 1.0f) vpAlpha = 1.0f;
      if (!pSeed) {
        pFilt = 0.0f;
        pSeed = true;
      }
      pFilt = vpAlpha * P + (1.0f - vpAlpha) * pFilt;
      P = pFilt;

      // Anti-windup: if the previous output saturated AND the new step
      // would drive further into saturation, freeze the integrator.
      // Otherwise accumulate and clamp. dt is outerDt (= N·DT), not
      // the per-tick DT, so raising N does not silently weaken velKi.
      integ += cp.velKi * err * outerDt;
      if (integ >  cp.velIClamp) integ =  cp.velIClamp;
      if (integ < -cp.velIClamp) integ = -cp.velIClamp;
      I = integ;

      // Differentiation on raw err (no pre-LPF). dErr = (err - errPrev)
      // / outerDt; errPrimed gates the very first run-tick after a
      // reset so we don't compute dErr against a bogus 0. The post-
      // gain D filter below (velDAlpha) is what tames noise — for an
      // LTI 1-pole IIR, filtering err pre-diff and filtering D post-
      // gain are equivalent up to constant gains, and the post-gain
      // form mirrors the velPAlpha shape on P (one knob per term,
      // same units, same reset semantics).
      float dErr = 0.0f;
      if (!errPrimed) {
        errPrev = err;
        errPrimed = true;
      } else {
        dErr = (err - errPrev) / outerDt;
        errPrev = err;
      }
      D = cp.velKd * dErr;

      // 1-pole IIR on the post-gain D term. Same shape as velPAlpha
      // on P: α=1 (default) passthrough; lower α = stronger smoothing
      // at the cost of phase lag. Seeding from 0 (not raw D) so each
      // arm starts the D contribution at rest, mirroring the targetV /
      // P / thetaSet ramps. Clamp α defensively against bad setParam
      // values. Effective sample period of the recurrence is outerDt
      // = N·DT (filter step runs only on outer-run ticks; skip ticks
      // reuse heldOuterD verbatim).
      float vdAlpha = cp.velDAlpha;
      if (vdAlpha < 0.0f) vdAlpha = 0.0f;
      if (vdAlpha > 1.0f) vdAlpha = 1.0f;
      if (!dSeed) {
        dFilt = 0.0f;
        dSeed = true;
      }
      dFilt = vdAlpha * D + (1.0f - vdAlpha) * dFilt;
      D = dFilt;

      thetaSet = P + I + D;
      if (thetaSet >  cp.maxAngleSetpoint) thetaSet =  cp.maxAngleSetpoint;
      if (thetaSet < -cp.maxAngleSetpoint) thetaSet = -cp.maxAngleSetpoint;

      // 1-pole IIR on the composite, post-clamp thetaSet. α=1 (default)
      // is passthrough; lower α smooths the whole outer-loop output in
      // one place — covers quantisation/noise on any of P, I, D and any
      // interaction between them. Filtering AFTER the clamp means the
      // filter state can never exceed ±maxAngleSetpoint, so what we
      // latch into heldThetaSet / publish / feed to the inner loop is
      // still bounded the same way. Seeding from 0 (not raw thetaSet)
      // matches the targetVFilt / pFilt soft-start convention — every
      // arm begins with the composite setpoint at rest. Clamp α
      // defensively against bad setParam values. NB: this step runs
      // only on outer-run ticks; skip ticks reuse heldThetaSet
      // verbatim, so the recurrence effectively samples at outerDt =
      // N·DT — same as the integrator and the dErr LPF.
      float oAlpha = cp.outerAlpha;
      if (oAlpha < 0.0f) oAlpha = 0.0f;
      if (oAlpha > 1.0f) oAlpha = 1.0f;
      if (!thetaSetSeed) {
        thetaSetFilt = 0.0f;
        thetaSetSeed = true;
      }
      thetaSetFilt = oAlpha * thetaSet + (1.0f - oAlpha) * thetaSetFilt;
      thetaSet = thetaSetFilt;

      // Latch the post-clamp values so the next (outerEvery-1) skip
      // ticks reuse them, both for the inner-loop feed and for the
      // shared::g publish at the bottom of this iteration.
      heldOuterP      = P;
      heldOuterI      = I;
      heldOuterD      = D;
      heldThetaSet    = thetaSet;
      heldTargetVUsed = targetVUsed;
    } else if (armed && outerOn && !outerRunTick) {
      // Skip tick: nothing to do. P/I/D/thetaSet/targetVUsed already
      // loaded from heldXxx above; the inner loop will consume them
      // unchanged and the publish at the bottom will republish them so
      // telemetry shows a clean zero-order-hold trace between outer
      // updates rather than a glitch back to 0.
    } else if (armed && !outerOn) {
      // Outer loop disabled by the user. Hold every piece of outer-PID
      // state in reset so a later flip to enabled starts from rest
      // (mirrors the disarm/!imuOk reset). thetaSet, P/I/D and
      // targetVUsed all stay at their 0 init above; the inner PD below
      // runs as a pure tilt corrector around upright. The cost of
      // re-zeroing every tick is a few stores; idempotent and cheap.
      integ = 0.0f;
      errPrimed = false;
      errPrev = 0.0f;
      dSeed = false;
      dFilt = 0.0f;
      targetVSeed = false;
      targetVFilt = 0.0f;
      pSeed = false;
      pFilt = 0.0f;
      thetaSetSeed = false;
      thetaSetFilt = 0.0f;
      // Held outer outputs reset too — and the downsample counter, so
      // the first tick after outerEnabled is flipped back on runs the
      // outer block immediately. Local P/I/D/thetaSet/targetVUsed
      // values reloaded from these for this tick's publish below.
      outerTickCount  = 0;
      heldOuterP      = 0.0f;
      heldOuterI      = 0.0f;
      heldOuterD      = 0.0f;
      heldThetaSet    = 0.0f;
      heldTargetVUsed = 0.0f;
      P = I = D = 0.0f;
      thetaSet = 0.0f;
      targetVUsed = 0.0f;
    }
    // else: disarmed — outer-PID state held in reset by the disarmed
    // branch below. P/I/D stay at their held (=zeroed-on-disarm) values,
    // thetaSet stays 0, targetVUsed stays 0 so velFF·targetVUsed
    // contributes nothing to the inner loop. The inner PD still runs
    // (see below) and acts as a pure tilt corrector priming vWheelFilt
    // for the moment armed flips true.

    // -- Inner PD + feed-forward (tilt → wheel velocity) ------------------
    // thetaTrim compensates for the chassis CG offset: a perfectly upright
    // bot reads theta ≈ thetaTrim from the IMU. Subtract it from theta
    // before feeding the PD so 0-tilt-error means physically vertical.
    //
    // innerOn gates ONLY the Kth/KthDot tilt-feedback terms. velFF still
    // contributes regardless so that with innerOn=false the bot is an
    // open-loop velocity drive (steering, vWheelAlpha and saturation
    // still applied below). thetaErr is computed unconditionally so the
    // expression is one-line and has no branch in the hot path; the
    // multiplications are by zero when innerOn is false. Cheap, and
    // keeps the published outerP/I/D and thetaSet/thetaErr semantics
    // identical between the two modes — only the wheel command differs.
    const float thetaErr = (theta - cp.thetaTrim) - thetaSet;
    const float kthEff    = innerOn ? cp.Kth    : 0.0f;
    const float kthDotEff = innerOn ? cp.KthDot : 0.0f;
    float vWheel = cp.velFF * targetVUsed
                 + kthEff    * thetaErr
                 + kthDotEff * thDot;

    // Saturate to vMaxWheel. The motors driver also saturates, but we
    // need a meaningful x_dot estimate locally.
    vWheel = clampf(vWheel, -cp.vMaxWheel, cp.vMaxWheel);

    // 1-pole IIR on the saturated wheel command. α=1 (default) is
    // passthrough — the recurrence collapses to vWheelFilt = vWheel and
    // there is zero phase lag. Drop α below 1 to smooth audible/visible
    // stepper chatter at the cost of phase lag inside the balancing
    // loop. Clamped defensively so a bad setParam can't blow up the
    // recurrence (mirrors the comp/gyro/acc alpha clamps in imu.cpp).
    //
    // Seeding policy: the seed branch sets vWheelFilt to ZERO (not raw
    // vWheel) so the very first armed tick after disarm/!imuOk produces
    // α·vWheel_raw (a small fraction) instead of stepping the wheels
    // straight to the raw PD output. This matches targetVFilt and
    // targetTurnFilt below — every ramp filter "soft starts" from rest.
    // Why: even at a benign sub-degree tilt the inner PD demands ~0.1
    // m/s; previously seeding to raw kicked the wheels by that full
    // amount on the first armed tick, the body whipped backward (inverted
    // pendulum reaction), and the heavy filter (low α) then dragged the
    // corrective response by ~τ ms of phase, growing the oscillation.
    // Soft-starting from 0 lets the wheels ramp up over the filter time
    // constant; auto-arm only fires within ±autoArmAngle so the bot is
    // near-vertical and has time for the gentle ramp to take effect.
    float vwAlpha = cp.vWheelAlpha;
    if (vwAlpha < 0.0f) vwAlpha = 0.0f;
    if (vwAlpha > 1.0f) vwAlpha = 1.0f;
    if (!vWheelSeed) {
      vWheelFilt = 0.0f;
      vWheelSeed = true;
    }
    vWheelFilt = vwAlpha * vWheel + (1.0f - vwAlpha) * vWheelFilt;
    vWheel = vWheelFilt;

    // -- Steering differential --------------------------------------------
    // Read the live targetTurn (set by joystick / WS), apply the
    // targetTurnAlpha 1-pole IIR for smooth ramping (independent of the
    // targetV ramp), then clamp against vMaxTurn defensively (the WS
    // handler also clamps but params can change underneath us). Split into
    // per-wheel commands:
    //   vL = vWheel + targetTurn
    //   vR = vWheel - targetTurn
    // Each clamped to ±vMaxWheel. Note: this is applied AFTER vWheelAlpha
    // so the common-mode filter doesn't drag the differential. xDotEst
    // stays equal to the common-mode vWheel because the cart-frame
    // velocity is the average of the two wheels — turn cancels.
    //
    // The filter only runs while armed: when disarmed we hold targetTurn
    // through to the (unused) per-wheel commands at zero anyway via
    // motors::stop(), but reset semantics matter for the first armed
    // tick — same lifecycle as targetVFilt above. We mirror the disarm
    // reset below by resetting targetTurnSeed only in that branch and the
    // !imuOk branch.
    float targetTurnRaw = shared::g.targetTurn.load();
    float ttAlpha = cp.targetTurnAlpha;
    if (ttAlpha < 0.0f) ttAlpha = 0.0f;
    if (ttAlpha > 1.0f) ttAlpha = 1.0f;
    if (!targetTurnSeed) {
      targetTurnFilt = 0.0f;
      targetTurnSeed = true;
    }
    targetTurnFilt = ttAlpha * targetTurnRaw + (1.0f - ttAlpha) * targetTurnFilt;
    float targetTurn = targetTurnFilt;
    const float turnLim = cp.vMaxTurn;
    if (turnLim > 0.0f) {
      if (targetTurn >  turnLim) targetTurn =  turnLim;
      if (targetTurn < -turnLim) targetTurn = -turnLim;
    }
    float vL = vWheel + targetTurn;
    float vR = vWheel - targetTurn;
    vL = clampf(vL, -cp.vMaxWheel, cp.vMaxWheel);
    vR = clampf(vR, -cp.vMaxWheel, cp.vMaxWheel);

    // Publish the post-filter / post-clamp turn value used THIS tick
    // for telemetry (separate from the raw shared::g.targetTurn the
    // operator wrote, so the UI can chart "commanded" vs "actually
    // used" side by side). Stored unconditionally here so the value
    // reflects the current armed-and-running tick; the disarm / !imuOk
    // branches force this back to 0 to match the held-in-reset
    // semantics of the ramp filter itself.
    shared::g.targetTurnUsed.store(targetTurn);

    // -- Drive output ------------------------------------------------------
    if (armed) {
      motors::setWheelVelocity(vL, vR);
      // xDotEst is the outer-loop velocity feedback. Use the *actual*
      // wheel velocity reported by motors:: (computed from LEDC's
      // achieved frequency, i.e. lastLedcGotL/R), not vWheel which is
      // the *commanded* setpoint. Most of the time these track each
      // other exactly. The case that matters is when the LEDC
      // peripheral can't divide its clock finely enough for a
      // requested frequency and silently idles the channel: vWheel
      // would still report the (rejected) command, the outer loop
      // would see zero velocity error, and the bot would tip while
      // the wheels stood still. Reading from motors:: closes that
      // hole — when LEDC fails, xDotEst correctly drops to 0 and
      // the loop sees the resulting velocity error.
      xDotEst = motors::wheelActualMps();
    } else {
      // Disarmed: hold motors stopped and force EVERY piece of controller
      // state to zero so the first armed tick after auto-arm starts from
      // a clean slate. Earlier we left vWheelFilt running (so the inner PD
      // could "warm up" and avoid a startup transient on arm), but in
      // practice that meant auto-arm would inherit a non-zero filter value
      // computed from a tilt setpoint that no longer applies, jolting the
      // wheels at the moment of arm. With this reset the bot is guaranteed
      // to begin balancing from rest, and any startup transient is bounded
      // by the ramps (targetVAlpha, targetTurnAlpha, vWheelAlpha) instead
      // of by stale filter contents.
      //
      // Resets here:
      //  - integ=0 stops the integrator carrying over.
      //  - lpfPrimed=false re-primes the dErr LPF on the first armed tick.
      //  - targetVFilt / targetTurnFilt seed=false so each arm ramps from
      //    rest toward the live joystick values (see params.h targetVAlpha
      //    / targetTurnAlpha).
      //  - vWheelFilt seed=false so the post-saturation IIR also re-seeds
      //    from zero (the first armed tick's vWheel becomes the seed).
      //  - xDotEst=0 so the outer PID's first armed tick sees a true
      //    "stopped cart" feedback, matching reality.
      //  - All published shared::g controller outputs zeroed explicitly
      //    so telemetry / UI don't display the last armed values frozen
      //    in place while disarmed (used to skip publish via continue,
      //    leaving stale numbers visible).
      motors::stop();
      integ = 0.0f;
      errPrimed = false;
      errPrev = 0.0f;
      dSeed = false;
      dFilt = 0.0f;
      xDotEst = 0.0f;
      targetVSeed = false;
      targetVFilt = 0.0f;
      targetTurnSeed = false;
      targetTurnFilt = 0.0f;
      vWheelSeed = false;
      vWheelFilt = 0.0f;
      pSeed = false;
      pFilt = 0.0f;
      thetaSetSeed = false;
      thetaSetFilt = 0.0f;
      // Outer downsample state: zero held outputs and reset the counter
      // so the first armed tick after disarm always runs the outer
      // block immediately (same reset convention as the !imuOk and
      // !outerOn branches above).
      outerTickCount  = 0;
      heldOuterP      = 0.0f;
      heldOuterI      = 0.0f;
      heldOuterD      = 0.0f;
      heldThetaSet    = 0.0f;
      heldTargetVUsed = 0.0f;
      shared::g.thetaSet.store(0.0f);
      shared::g.vWheelCmd.store(0.0f);
      shared::g.vWheelCmdL.store(0.0f);
      shared::g.vWheelCmdR.store(0.0f);
      shared::g.xDotEst.store(0.0f);
      shared::g.outerP.store(0.0f);
      shared::g.outerI.store(0.0f);
      shared::g.outerD.store(0.0f);
      // targetTurnUsed mirrors the held-in-reset state of the ramp
      // filter while disarmed; clear it so the UI shows 0 (matching
      // what motors::stop() above just enforced) instead of the last
      // armed value frozen in place.
      shared::g.targetTurnUsed.store(0.0f);
      continue;
    }

    // -- Publish -----------------------------------------------------------
    shared::g.thetaSet.store(thetaSet);
    shared::g.vWheelCmd.store(vWheel);
    // Per-side post-split, post-saturation values that were just handed
    // to motors::setWheelVelocity. Telemetered separately from vWheelCmd
    // so the UI can plot L vs R commands and detect asymmetric
    // saturation / steering bias the average would hide.
    shared::g.vWheelCmdL.store(vL);
    shared::g.vWheelCmdR.store(vR);
    shared::g.xDotEst.store(xDotEst);
    shared::g.outerP.store(P);
    shared::g.outerI.store(I);
    shared::g.outerD.store(D);

    // -- 0.1 Hz human log + edge-triggered arm/disarm --------------------
    if (armed != prevArmed) {
      Serial.print(F("ctrl: "));
      Serial.print(armed ? F("ARMED") : F("disarmed"));
      Serial.print(F(" (th="));   Serial.print(theta, 3);
      Serial.print(F(" tgtV=")); Serial.print(targetV, 3);
      Serial.println(F(")"));
      prevArmed = armed;
    }
    if (++logCounter >= LOG_PERIOD) {
      logCounter = 0;
      Serial.print(F("ctrl: tgtV="));
      Serial.print(targetV, 3);
      Serial.print(F(" th="));
      Serial.print(theta, 3);
      Serial.print(F(" thDot="));
      Serial.print(thDot, 3);
      Serial.print(F(" thSet="));
      Serial.print(thetaSet, 3);
      Serial.print(F(" vW="));
      Serial.print(vWheel, 3);
      Serial.print(F(" I="));
      Serial.print(I, 3);
      Serial.print(F(armed ? " ARMED" : " disarmed"));
      Serial.println();
    }
  }
}

} // namespace

void start() {
  if (running) return;
  // Stack: 4096 B is comfortably above measured worst-case for this kind
  // of FP-heavy loop on Xtensa with no library calls per tick (Serial
  // prints once per second only).
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr,
                          /*priority=*/4, nullptr, CORE_CONTROL);
  running = true;
}

bool isRunning() { return running; }

} // namespace controller
