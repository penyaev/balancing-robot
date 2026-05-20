// motors.cpp — F6 wheel-velocity driver using ESP32 LEDC for STEP pulse
// generation. TMC2209 control plane (UART/probe/watchdog/HTML) lives
// separately in motors_drv.{h,cpp} and is shared with anything else
// that needs to talk to the chips.
//
// Why LEDC and not FastAccelStepper:
//   FAS layers an acceleration ramp + look-ahead command queue on top of
//   the hardware step generator. For a balancer, both are
//   counterproductive:
//     - The controller already produces a smoothed velocity command
//       (Kth*θerr + KthDot*θ̇ + velFF*targetV), so an extra accel ramp
//       just adds lag between control authority and wheel response.
//     - The queue caused a wake-from-rest pathology: at low step rates
//       each queue entry is long-duration and a fresh fast command can't
//       replan ahead of the in-flight slow entries (FAS fill_queue()
//       plans 20 ms ahead OR ≥2 entries, whichever longer; with Kth*θerr
//       noise at ±1 step/sec near upright, that is multiple seconds of
//       stale plan).
//   LEDC collapses command-to-step into a single register write per
//   control tick: setStepFreq(channel, target_hz) → ledcSetup +
//   ledcWrite. Zero queue, zero accel — what you ask for is what
//   comes out the STEP pin (modulo the LEDC peripheral's
//   max-frequency cap; see setStepFreq for details on why we don't
//   use the stock ledcWriteTone wrapper).
//
// Pin/UART layout: see config.h. STEP_L/R go to two LEDC channels; DIR_L/R
// are plain digital outputs.

#include "motors.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "motors_drv.h"
#include "params.h"

namespace motors {
namespace {

// Two LEDC channels, one per STEP pin. Channels 0 and 1 are free
// (status LED uses no LEDC; IMU/UART use no LEDC). Resolution is duty
// resolution; for a square-wave tone we always sit at 50% duty so 8
// bits is plenty. ledcSetup chooses the timer divider to hit the
// requested frequency given the resolution.
//
// CRITICAL: pick channels that map to *different* LEDC timers.
// arduino-esp32's LEDC HAL assigns each channel to a hardware timer
// using `timer = (channel / 2) % 4`, so channels 0+1 share timer 0,
// channels 2+3 share timer 1, and so on. ledcSetup() (which we call
// every tick from setStepFreq) reconfigures the underlying timer's
// frequency — meaning if both STEP pins live on the same timer pair,
// the latest call instantaneously stomps the *other* channel's
// frequency too. Symptom: asymmetric L/R commands (i.e. any turn)
// produce wheel jumps and chatter; symmetric commands look fine
// because both writes ask for the same freq. (We hit this for real
// during turn-tuning; the LEDC req/got chart didn't show it because
// each write succeeds in isolation — the damage is to the *other*
// channel's already-set timer.) Putting L on CH 0 (timer 0) and R
// on CH 2 (timer 1) gives each side its own independent timer.
constexpr uint8_t LEDC_CH_L = 0;
constexpr uint8_t LEDC_CH_R = 2;
constexpr uint8_t LEDC_RES_BITS = 8;
constexpr uint32_t LEDC_INIT_FREQ = 1000;  // arbitrary nonzero, will be overwritten

// Cache of the last commanded step rate per side (signed; negative
// means DIR was set for reverse). Used to decide whether we need to
// poke the LEDC channel — *not* used to report wheelActualMps()
// anymore (see lastLedcGotL/R below for why).
int32_t lastCmdStepsL = 0;
int32_t lastCmdStepsR = 0;

// Cache of the most recent (req, got) pair last passed to / returned
// from ledcWriteTone() per side. Signed: sign reflects the commanded
// direction at the time of the call (positive = forward, negative =
// reverse, 0 = channel idle / below deadband / stopped). Updated only
// inside applyToChannel() (the sole ledcWriteTone caller for an active
// side) and inside stop() (which forces both pairs to 0). Surfaced via
// the ledcReqStepsL/R / ledcGotStepsL/R getters declared in motors.h
// so telemetry can chart req-vs-got per channel — divergence between
// the two would be the smoking gun for an LEDC frequency-rounding bug
// (e.g. caused by a future change to LEDC_RES_BITS shrinking the
// achievable-frequency set). Today they should track exactly.
int32_t lastLedcReqL = 0;
int32_t lastLedcGotL = 0;
int32_t lastLedcReqR = 0;
int32_t lastLedcGotR = 0;

// Cached conversion: m/s → steps/sec. Recomputed when microsteps change.
float stepsPerMeter = 0.0f;

void recomputeStepsPerMeter(uint16_t microsteps) {
  // steps_per_rev = full_steps * microsteps; meters_per_rev = 2π * R.
  const float stepsPerRev = static_cast<float>(STEPPER_FULLSTEPS) *
                            static_cast<float>(microsteps);
  const float metersPerRev = 2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_M;
  stepsPerMeter = stepsPerRev / metersPerRev;
}

// Set a STEP-pin frequency on one LEDC channel, returning the actual
// frequency the hardware managed to set (0 == channel idled).
//
// Why we don't use ledcWriteTone(): arduino-esp32's ledcWriteTone()
// silently calls ledcSetup(channel, freq, /*resolution=*/10) under
// the hood, hard-coding the duty resolution to 10 bits regardless of
// what we configured at start(). With APB at 80 MHz, 10-bit resolution
// caps the achievable frequency at 80e6 / 2^10 ≈ 78 125 Hz: above
// that, the LEDC peripheral can't divide its clock finely enough and
// ledcSetup returns 0 (== "no can do"), which ledcWriteTone propagates
// back. We saw exactly this in a debug dump: as the controller asked
// for ≥86 kHz step rates the LEDC silently went idle on both channels
// while wheelActualMps()-from-cmd kept lying that everything was fine,
// the controller saw zero velocity error, and the bot tipped over.
//
// Bypassing ledcWriteTone and calling ledcSetup ourselves with our
// configured LEDC_RES_BITS=8 raises the cap to 80e6 / 2^8 ≈ 312 500 Hz
// (~1.92 m/s wheel velocity at microsteps=256), which is well past
// vMaxWheel for any realistic config. After ledcSetup we manually
// write 50% duty (1<<(RES-1)) to produce the square-wave tone that
// ledcWriteTone would have written for us.
//
// Returns the frequency the hardware actually achieved (0 if the
// channel was asked to idle, or >0 if ledcSetup succeeded). Caller
// uses this to populate the lastLedcGot diagnostic cache and — via
// wheelActualMps{,L,R}() — the controller's xDotEst feedback.
uint32_t setStepFreq(uint8_t ledcCh, uint32_t freq) {
  if (freq == 0) {
    // Idle the channel. Writing duty=0 stops the edges; we do NOT call
    // ledcSetup with freq=0 because the divider math goes pathological.
    ledcWrite(ledcCh, 0);
    return 0;
  }
  const uint32_t got = ledcSetup(ledcCh, freq, LEDC_RES_BITS);
  if (got > 0) {
    // 50% duty for a square-wave STEP signal. At 8-bit resolution the
    // half-scale duty value is 1<<7 = 128.
    ledcWrite(ledcCh, 1u << (LEDC_RES_BITS - 1));
  }
  // got == 0 means the requested frequency was unachievable at our
  // resolution. The channel is now in an undefined state from
  // ledcSetup's POV; explicitly idle it so we don't leave a stale
  // tone pulsing at the previous frequency.
  if (got == 0) {
    ledcWrite(ledcCh, 0);
  }
  return got;
}

// Convert signed velocity [m/s] into a signed step rate, applying the
// per-side mounting invert. Round, not truncate, so small velocities
// don't get quantised to 0 unevenly between the two wheels.
int32_t velocityToSteps(float v, bool invert) {
  if (invert) v = -v;
  return static_cast<int32_t>(lroundf(v * stepsPerMeter));
}

// Apply a signed step-rate command to one wheel:
//   - Below the deadband (config.h::MOTOR_DEADBAND_STEPS) we collapse to
//     0 → ledcWriteTone(0) → STEP idle. This is what fixed the
//     wake-from-rest lag back when FAS was the backend (the FAS queue
//     no longer stuffed itself with multi-second slow ramps from
//     Kth*θerr noise). With LEDC the rationale is gentler — just
//     "don't bother emitting <50 Hz garbage edges that the chip will
//     stretch over 20 ms anyway" — but the hysteresis is still useful.
//   - Otherwise: set DIR pin from sign, then ledcWriteTone(magnitude).
//
// We change DIR before the STEP frequency. The TMC2209 needs only 20 ns
// of DIR setup, easily met by the GPIO write being orders of magnitude
// slower than that, but doing it in this order keeps the behaviour
// obviously correct without inserting delays.
void applyToChannel(uint8_t ledcCh, uint8_t dirPin,
                    int32_t signedSteps,
                    int32_t& lastSteps,
                    int32_t& lastReq,
                    int32_t& lastGot,
                    const char* tag) {
  const int32_t mag32 = signedSteps < 0 ? -signedSteps : signedSteps;
  if (mag32 < MOTOR_DEADBAND_STEPS) {
    if (lastSteps != 0) {
      setStepFreq(ledcCh, 0);
      lastSteps = 0;
    }
    // Always update the diagnostic cache when we hit this branch — even
    // when lastSteps was already 0 we want telemetry to show "channel
    // idle this tick" rather than the stale value from the last active
    // call. Both fields zero, no direction sign (we're not driving).
    lastReq = 0;
    lastGot = 0;
    return;
  }
  // Update DIR if the sign changed (or we were stopped — DIR state is
  // unknown after the initial pinMode in start()).
  const bool wantForward = signedSteps > 0;
  const bool wasForward  = lastSteps  > 0;
  if (lastSteps == 0 || wantForward != wasForward) {
    digitalWrite(dirPin, wantForward ? HIGH : LOW);
  }
  // setStepFreq() returns the actual frequency the LEDC peripheral
  // managed to set; if that diverges from requested (most likely 0
  // because the requested frequency exceeds what our resolution allows
  // — see setStepFreq comment), log it. The (req, got) pair is also
  // exposed via telemetry so the UI charts the divergence directly.
  const uint32_t req = static_cast<uint32_t>(mag32);
  const uint32_t got = setStepFreq(ledcCh, req);
  // Cache (req, got) with the commanded direction sign reapplied so the
  // telemetry chart shows the same sign convention as cmd/actual lines
  // (positive = forward at the LEDC pin's intent, negative = reverse).
  // Magnitude is the raw STEP-pin frequency in Hz, not converted to
  // m/s — the whole point of these series is to show LEDC quantisation
  // behaviour in its native units.
  const int32_t signMul = wantForward ? 1 : -1;
  lastReq = signMul * static_cast<int32_t>(req);
  lastGot = signMul * static_cast<int32_t>(got);
  if (got != req) {
    Serial.print(F("LEDC "));
    Serial.print(tag);
    Serial.print(F(" req="));
    Serial.print(req);
    Serial.print(F(" got="));
    Serial.println(got);
  }
  lastSteps = signedSteps;
}

}  // namespace

bool start() {
  // 1) TMC2209 control plane: UART + probe + GCONF watchdog.
  const ControlParams& cp = params::current;
  const bool drvOk = drv::start(cp);

  // 2) Step generator: LEDC channels on the two STEP pins, DIR pins as
  //    plain digital outputs. We always set up the LEDC even if the
  //    drivers didn't ack — the step pulses cost nothing if the chip
  //    isn't listening, and it keeps later wake/teardown paths simple.
  recomputeStepsPerMeter(cp.microsteps);

  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  digitalWrite(PIN_DIR_L, LOW);
  digitalWrite(PIN_DIR_R, LOW);

  ledcSetup(LEDC_CH_L, LEDC_INIT_FREQ, LEDC_RES_BITS);
  ledcSetup(LEDC_CH_R, LEDC_INIT_FREQ, LEDC_RES_BITS);
  ledcAttachPin(PIN_STEP_L, LEDC_CH_L);
  ledcAttachPin(PIN_STEP_R, LEDC_CH_R);
  // Start with channels stopped (zero duty = no edges).
  setStepFreq(LEDC_CH_L, 0);
  setStepFreq(LEDC_CH_R, 0);

  Serial.print(F("motors: stepsPerMeter="));
  Serial.println(stepsPerMeter, 1);

  return drvOk;
}

bool isReady()        { return drv::isReadyL() && drv::isReadyR(); }
bool isDriverReadyL() { return drv::isReadyL(); }
bool isDriverReadyR() { return drv::isReadyR(); }

void setWheelVelocity(float vLeft, float vRight) {
  // Saturate to vMaxWheel on both sides. Defensive — F7 should also limit,
  // but a buggy controller shouldn't be able to ask for an impossible rate.
  const float vmax = params::current.vMaxWheel;
  if (vmax > 0.0f) {
    if (vLeft  >  vmax) vLeft  =  vmax;
    if (vLeft  < -vmax) vLeft  = -vmax;
    if (vRight >  vmax) vRight =  vmax;
    if (vRight < -vmax) vRight = -vmax;
  }

  // Slew-rate limit. The controller can produce a commanded velocity
  // step of ~vMaxWheel within one tick if the inner law saturates (e.g.
  // a sudden disturbance with large Kth*err). On the LEDC backend that
  // step shows up as an instantaneous LEDC frequency change → the
  // stepper has to reverse direction in a single control period (~5 ms).
  // Two consequences we observed during tuning:
  //   1. Peak current draw spikes on every reversal, tripping the
  //      battery-side overcurrent protection.
  //   2. Closed-loop positive-feedback paths (KthDot via raw gyro,
  //      velKd via xD derivative) chatter the wheels at frequencies
  //      where each ground impulse re-excites the loop. Without a
  //      bound on dV/dt the loop can ring up arbitrarily.
  // Cap |dV/dt| at params::current.aMaxWheel [m/s²] using actual
  // elapsed time since last call (robust to controller tick jitter).
  // aMaxWheel <= 0 disables the limiter (passthrough) for A/B testing.
  const float aMax = params::current.aMaxWheel;
  if (aMax > 0.0f) {
    static float    lastVL = 0.0f;
    static float    lastVR = 0.0f;
    static uint32_t lastSlewUs = 0;
    const uint32_t  nowUs = micros();
    float dt = (lastSlewUs == 0) ? 0.005f
                                 : (nowUs - lastSlewUs) * 1e-6f;
    // Clamp dt: if the controller stalls for >100 ms (e.g. paused, or
    // first call after start()), don't let an unbounded step accumulate
    // into "any velocity is reachable in one tick". Treat it as a
    // first-call reset.
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;
    const float dvMax = aMax * dt;
    auto slew = [&](float target, float& last) {
      const float dv = target - last;
      if      (dv >  dvMax) last = last + dvMax;
      else if (dv < -dvMax) last = last - dvMax;
      else                  last = target;
      return last;
    };
    vLeft  = slew(vLeft,  lastVL);
    vRight = slew(vRight, lastVR);
    lastSlewUs = nowUs;
  }

  const int32_t cmdL = velocityToSteps(vLeft,  MOTOR_INVERT_L);
  const int32_t cmdR = velocityToSteps(vRight, MOTOR_INVERT_R);

  if (cmdL != lastCmdStepsL) {
    applyToChannel(LEDC_CH_L, PIN_DIR_L, cmdL, lastCmdStepsL,
                   lastLedcReqL, lastLedcGotL, "L");
  }
  if (cmdR != lastCmdStepsR) {
    applyToChannel(LEDC_CH_R, PIN_DIR_R, cmdR, lastCmdStepsR,
                   lastLedcReqR, lastLedcGotR, "R");
  }
}

void stop() {
  setStepFreq(LEDC_CH_L, 0);
  setStepFreq(LEDC_CH_R, 0);
  lastCmdStepsL = 0;
  lastCmdStepsR = 0;
  // Mirror the cmd cache: stop() is "channels forced idle", so the
  // diagnostic req/got pair must also reflect that immediately. Without
  // this, a Disarm right after a fast spin would leave telemetry
  // showing the last active req/got values forever (the next active
  // applyToChannel call only updates them when the controller next
  // crosses out of the deadband).
  lastLedcReqL = 0;
  lastLedcGotL = 0;
  lastLedcReqR = 0;
  lastLedcGotR = 0;
}

float wheelActualMps() {
  if (stepsPerMeter <= 0.0f) return 0.0f;
  // Source of truth is lastLedcGotL/R — the *actual* frequency the
  // LEDC peripheral managed to set, not the *commanded* step rate
  // (lastCmdStepsL/R). They diverge whenever a requested frequency
  // exceeds what our 8-bit LEDC resolution can divide to (got=0;
  // see setStepFreq). Reading "got" makes silent LEDC failures
  // visible to the controller's xDotEst feedback path: if the wheels
  // are actually idle because LEDC refused the freq, the outer loop
  // sees zero forward velocity (matching reality) and the inner
  // tilt-keeper isn't lied to. Reading "cmd" instead would let the
  // controller spiral: it would think the wheels were running, see
  // no velocity error, and never grow the command enough to recover
  // (we observed exactly this fall mode).
  //
  // We still don't know if the *motor* missed steps (no encoder),
  // but at least we know what the STEP pin actually emitted.
  // Average L and R, undo per-side mounting invert so result is in
  // cart frame. lastLedcGotL/R already carry the commanded sign.
  const float lMps =
      static_cast<float>(lastLedcGotL) / stepsPerMeter * (MOTOR_INVERT_L ? -1.0f : 1.0f);
  const float rMps =
      static_cast<float>(lastLedcGotR) / stepsPerMeter * (MOTOR_INVERT_R ? -1.0f : 1.0f);
  return 0.5f * (lMps + rMps);
}

// Per-side variants. Same conversion as wheelActualMps() above but
// without the L+R average — surfaced so telemetry can chart
// per-wheel cmd-vs-actual lines. Same "got, not cmd" rationale.
float wheelActualMpsL() {
  if (stepsPerMeter <= 0.0f) return 0.0f;
  return static_cast<float>(lastLedcGotL) / stepsPerMeter *
         (MOTOR_INVERT_L ? -1.0f : 1.0f);
}
float wheelActualMpsR() {
  if (stepsPerMeter <= 0.0f) return 0.0f;
  return static_cast<float>(lastLedcGotR) / stepsPerMeter *
         (MOTOR_INVERT_R ? -1.0f : 1.0f);
}

// Diagnostic accessors: most recent (req, got) signed step rate per
// LEDC channel. See motors.h for full semantics. These are direct
// reads of cache values written by applyToChannel() / stop(); no
// computation, no UART traffic — safe to call from telemetryTask
// every tick.
int32_t ledcReqStepsL() { return lastLedcReqL; }
int32_t ledcGotStepsL() { return lastLedcGotL; }
int32_t ledcReqStepsR() { return lastLedcReqR; }
int32_t ledcGotStepsR() { return lastLedcGotR; }

void applyParamsLive() {
  const ControlParams& cp = params::current;
  // Microsteps may have changed → step-rate scaling needs to follow.
  // Do this BEFORE the driver reconfig so an in-flight step rate is
  // already using the new conversion by the time the chip switches
  // its CHOPCONF.MRES on the next UART write.
  recomputeStepsPerMeter(cp.microsteps);
  drv::applyParamsLive(cp);
}

void printStatus(::Print& out) {
  out.print(F("motors: stepsPerMeter="));
  out.println(stepsPerMeter, 1);
  drv::printStatus(out);
}

void printDriverDetailsHtml(::Print& out, char side) {
  drv::printDriverDetailsHtml(out, side);
}

void readDriverSnapshot(char side, DriverSnapshot& out) {
  drv::readDriverSnapshot(side, out);
}

}  // namespace motors
