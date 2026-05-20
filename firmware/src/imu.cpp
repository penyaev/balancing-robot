// imu.cpp — see imu.h.

#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <atomic>

#include <MPU6050.h>

#include "config.h"
#include "params.h"
#include "shared_state.h"

namespace imu {

namespace {

constexpr int  SAMPLE_HZ        = CONTROL_LOOP_HZ;       // 200 Hz
constexpr int  SAMPLE_PERIOD_MS = 1000 / SAMPLE_HZ;
constexpr int  LOG_PERIOD_MS    = 10000;
constexpr BaseType_t CORE_IMU   = 1;

// Sensitivity (LSB/unit) for the ranges we configure below. ±500 dps gyro,
// ±4 g accel, per the MPU6050 datasheet.
constexpr float GYRO_LSB_PER_DPS  = 65.5f;
constexpr float ACCEL_LSB_PER_G   = 8192.0f;
constexpr float DEG_TO_RAD_F      = 0.0174532925f;

// Stand-still detector for gyro-bias calibration.
//
// Window/threshold rationale:
//   - 2 s window at 5 ms/sample = up to ~400 samples.
//   - 5°/s threshold easily clears any real stillness (clone chips
//     typically sit at ≤ 1.5°/s gyro noise + bias) but rejects any
//     deliberate bump or the bot being picked up.
//   - We only check the GYRO. Earlier we also gated on |accel|≈1g, but
//     no-name MPU-6500 clones routinely have 5–8% accel scale-factor
//     error before factory calibration, so |aMag-1| can sit at 0.06
//     when perfectly still and reject every sample. Motion is already
//     detected by the gyro check directly — adding accel just made
//     calibration brittle without adding signal.
//   - On a rejected sample we just SKIP it (don't accumulate); we do
//     NOT reset the accumulator. The previous behaviour required N
//     consecutive still samples and so a single bump wiped 1 s of
//     accumulated data — completely unrealistic during bringup.
constexpr int   CAL_WINDOW_MS     = 2000;
constexpr int   CAL_MIN_SAMPLES   = 100;     // ~0.5 s effective integration
constexpr float CAL_MAX_GYRO_DPS  = 5.0f;

// Instantiated after the bus scan picks an address (0x68 or 0x69), so the
// MPU6050 lib's devAddr field is set correctly via its constructor.
MPU6050* g_mpu = nullptr;
volatile bool g_ready = false;
float g_biasGx = 0, g_biasGy = 0, g_biasGz = 0;

// Calibration request handshake. Written by net.cpp (requestCalibration,
// clearCalibrationState) on the AsyncTCP task; transitioned through
// PENDING → RUNNING → DONE_{OK,ERR} by imuTask. uint8_t fits in a single
// instruction's atomic operations on Xtensa.
std::atomic<uint8_t> g_calState{static_cast<uint8_t>(CalState::IDLE)};

// Derive pitch from the accelerometer. Convention: when the chassis is
// upright, accel_x ≈ 0 and accel_z ≈ +1 g, so atan2(ax, az) = 0. A tilt
// toward +x makes ax positive, so theta becomes positive — matching
// PLAN.md. Caller passes accel components in g.
inline float thetaFromAccel(float ax, float az) {
  return atan2f(ax, az);
}

bool calibrateGyro() {
  Serial.println(F("imu: calibrating gyro bias (hold still ~2 s)..."));
  const uint32_t deadline = millis() + (uint32_t)CAL_WINDOW_MS;
  int samples = 0, total = 0, rejected = 0;
  double sx = 0, sy = 0, sz = 0;
  // Track the actual noise envelope so a failed calibration tells us
  // *what* moved (gyro noise too high vs nothing being seen at all),
  // and a successful one gives us a noise estimate "for free".
  float maxAbsG = 0.0f;
  while (millis() < deadline) {
    int16_t ax, ay, az, gx, gy, gz;
    g_mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    const float gxDps = gx / GYRO_LSB_PER_DPS;
    const float gyDps = gy / GYRO_LSB_PER_DPS;
    const float gzDps = gz / GYRO_LSB_PER_DPS;

    const float aMax = fmaxf(fmaxf(fabsf(gxDps), fabsf(gyDps)), fabsf(gzDps));
    if (aMax > maxAbsG) maxAbsG = aMax;
    ++total;

    if (aMax < CAL_MAX_GYRO_DPS) {
      sx += gxDps; sy += gyDps; sz += gzDps;
      ++samples;
    } else {
      // Skip this single sample only — keep what we've already got.
      // A transient bump no longer wipes the accumulator.
      ++rejected;
    }
    delay(5);
  }

  if (samples < CAL_MIN_SAMPLES) {
    Serial.print(F("imu: calibration FAILED — only "));
    Serial.print(samples);
    Serial.print(F("/"));
    Serial.print(total);
    Serial.print(F(" samples passed (rejected "));
    Serial.print(rejected);
    Serial.print(F(", max|gyro|="));
    Serial.print(maxAbsG, 2);
    Serial.println(F("°/s). Bias = 0; will drift until recalibrated."));
    g_biasGx = g_biasGy = g_biasGz = 0;
    return false;
  }
  g_biasGx = (float)(sx / samples);
  g_biasGy = (float)(sy / samples);
  g_biasGz = (float)(sz / samples);
  Serial.print(F("imu: gyro bias dps gx="));
  Serial.print(g_biasGx, 3);
  Serial.print(F(" gy="));
  Serial.print(g_biasGy, 3);
  Serial.print(F(" gz="));
  Serial.print(g_biasGz, 3);
  Serial.print(F(" ("));
  Serial.print(samples);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.print(F(" samples, max|gyro|="));
  Serial.print(maxAbsG, 2);
  Serial.println(F("°/s)"));
  return true;
}

void imuTask(void* /*arg*/) {
  // Adopt the persisted gyro bias before producing any samples. Defaults
  // are zero (see params::defaults()), which means a fresh device runs
  // with raw bias until the user clicks "Calibrate IMU" — explicit and
  // visible, by design (see imu.h header comment).
  g_biasGx = params::current.gyroBiasX;
  g_biasGy = params::current.gyroBiasY;
  g_biasGz = params::current.gyroBiasZ;

  // Seed theta from a fresh accel reading so we don't ramp up from zero.
  {
    int16_t ax, ay, az, gx, gy, gz;
    g_mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    const float axG = ax / ACCEL_LSB_PER_G;
    const float azG = az / ACCEL_LSB_PER_G;
    shared::g.theta.store(thetaFromAccel(axG, azG), std::memory_order_relaxed);
    shared::g.thetaDot.store(0.0f, std::memory_order_relaxed);
  }
  g_ready = true;

  TickType_t last = xTaskGetTickCount();
  uint32_t lastLogMs = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

    // Service a pending calibration request before sampling. We hold the
    // I²C bus exclusively in this task, so calibration MUST run here
    // (the net handler can't poke the MPU directly without a lock). The
    // 2 s window blocks normal sampling — that's why the WS handler
    // refuses to start a calibration while motors are armed (would lose
    // theta/thetaDot updates and drop the bot). On completion we update
    // both the module-static bias (used by the inner gyro path below)
    // and params::current (so subsequent boots load it from NVS), then
    // request the net layer save + broadcast.
    if (g_calState.load(std::memory_order_acquire) ==
        static_cast<uint8_t>(CalState::PENDING)) {
      g_calState.store(static_cast<uint8_t>(CalState::RUNNING),
                       std::memory_order_release);
      const bool ok = calibrateGyro();
      // Mirror result into ControlParams. Float writes are atomic on
      // 32-bit-aligned scalars; concurrent readers (this task on
      // subsequent ticks, controller, etc.) see either old or new
      // bias, never a torn value.
      params::current.gyroBiasX = g_biasGx;
      params::current.gyroBiasY = g_biasGy;
      params::current.gyroBiasZ = g_biasGz;
      g_calState.store(static_cast<uint8_t>(
                         ok ? CalState::DONE_OK : CalState::DONE_ERR),
                       std::memory_order_release);
      // Reset our delay-until baseline; the 2 s blocking call has long
      // pushed `last` into the past and a flood of catch-up ticks would
      // be useless work.
      last = xTaskGetTickCount();
    }

    int16_t ax, ay, az, gx, gy, gz;
    g_mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert to physical units. Pitch axis = body Y (PLAN.md §7).
    // Gyro sign is flipped per IMU_GYRO_PITCH_SIGN to match the accel
    // convention (positive theta = tipping toward +x). Without this,
    // a fast tilt produces a brief dip in the wrong direction before
    // the accel-side complementary correction drags theta back —
    // see config.h for the full diagnostic signature.
    const float axG    = ax / ACCEL_LSB_PER_G;
    const float azG    = az / ACCEL_LSB_PER_G;
    // Bias-subtract both axes BEFORE applying any cross-talk
    // compensation. The Z DC bias is irrelevant to pitch and would
    // otherwise add a constant pitch-rate offset (= k * biasZ) to gy
    // when k != 0 — small but exactly the kind of "why does theta drift
    // since I added the compensation?" footgun we want to avoid.
    const float gyDpsBiased = gy / GYRO_LSB_PER_DPS - g_biasGy;
    const float gzDps       = gz / GYRO_LSB_PER_DPS - g_biasGz;
    // Yaw→pitch cross-talk compensation (mounting yaw misalignment +
    // datasheet ±2% cross-axis sensitivity). Subtract a scaled copy of
    // the bias-corrected raw gyro Z from the bias-corrected raw gyro Y
    // BEFORE the IMU_GYRO_PITCH_SIGN flip, so k is in the chip's own
    // axis frame and survives any future change to the mounting sign
    // convention. Disabled by default (k=0). See params.h::gyroYawXTalk
    // for the full calibration procedure.
    const float kYXT = params::current.gyroYawXTalk;
    const float gyDpsRaw = gyDpsBiased - kYXT * gzDps;
    const float gyDps    = IMU_GYRO_PITCH_SIGN * gyDpsRaw;
    const float thetaDotRaw = gyDps * DEG_TO_RAD_F; // rad/s, unfiltered
    // Yaw-axis (chip Z) reading, bias-corrected, no sign flip. Stored
    // raw to shared::g.gyroZ for diagnostics — see the field's comment
    // in shared_state.h. Same gzDps used above for the cross-talk
    // compensation, so the chart trace and the compensation source are
    // guaranteed to be the same number.
    shared::g.gyroZ.store(gzDps * DEG_TO_RAD_F, std::memory_order_relaxed);

    // ---- Pre-fusion filtering -------------------------------------------
    //
    // Both raw signals (thetaDot from the gyro, thetaAcc from atan2 of the
    // gravity vector) get their own optional 1-pole IIR BEFORE anything
    // else uses them. This keeps the pipeline easy to reason about: every
    // downstream consumer (comp filter, notch, controller) sees a single
    // canonical "filtered" version, not a mix of raw and processed signals.
    //
    // Trade-off note: filtering thetaDot before the comp-filter integration
    // adds a tiny bit of phase lag to the gyro path. With compAlpha=0.01
    // the comp filter itself is already a ~0.32 Hz low-pass on theta, so
    // an extra pole at gyroAlpha=0.44 (≈25 Hz cutoff) is dominated by the
    // comp filter and effectively invisible. If you crank compAlpha closer
    // to 1 (very accel-trusting) the gyro-path lag matters more — at that
    // point you're not really running a comp filter anyway.

    // 1-pole IIR on thetaDot. Direct mix coefficient α = gyroAlpha.
    // y[n] = α·x[n] + (1-α)·y[n-1]. The on-chip MPU6050 DLPF is set to
    // 42 Hz (start()), so this stacks an extra pole on top — well
    // separated from the body's <3 Hz dynamics. α<=0 disables (pass-through)
    // for A/B testing; α>=1 means no smoothing (raw).
    static float thetaDotFilt = 0.0f;
    static bool  thetaDotSeed = false;
    float gAlpha = params::current.gyroAlpha;
    if (gAlpha < 0.0f) gAlpha = 0.0f;
    if (gAlpha > 1.0f) gAlpha = 1.0f;
    if (!thetaDotSeed) {
      thetaDotFilt = thetaDotRaw;
      thetaDotSeed = true;
    } else if (gAlpha > 0.0f) {
      thetaDotFilt = gAlpha * thetaDotRaw + (1.0f - gAlpha) * thetaDotFilt;
    } else {
      thetaDotFilt = thetaDotRaw; // passthrough
    }

    // 1-pole IIR on thetaAcc. accAlpha=1 = passthrough (default). Lower
    // to denoise the gravity-vector reference itself, independently of
    // compAlpha (which controls how much weight the comp filter gives
    // it). Stacks with compAlpha for a 2-pole low-pass on the accel
    // branch when both are below 1.
    const float thetaAccRaw = thetaFromAccel(axG, azG);
    static float thetaAccFilt = 0.0f;
    static bool  thetaAccSeed = false;
    float aAlpha = params::current.accAlpha;
    if (aAlpha < 0.0f) aAlpha = 0.0f;
    if (aAlpha > 1.0f) aAlpha = 1.0f;
    if (!thetaAccSeed) {
      thetaAccFilt = thetaAccRaw;
      thetaAccSeed = true;
    } else {
      thetaAccFilt = aAlpha * thetaAccRaw + (1.0f - aAlpha) * thetaAccFilt;
    }

    // ---- Complementary filter (operates on the FILTERED inputs) ---------
    const float dt = SAMPLE_PERIOD_MS * 0.001f;
    // Direct alpha from params (replaces the old compFilterTau →
    // dt/(τ+dt) conversion). Clamp to [0,1] defensively; α<0 or α>1 from a
    // bad setParam would otherwise blow up the recurrence below.
    float alpha = params::current.compAlpha;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    float theta = shared::g.theta.load(std::memory_order_relaxed);
    theta += thetaDotFilt * dt;
    theta  = (1.0f - alpha) * theta + alpha * thetaAccFilt;
    (void)params::current.thetaTrim; // applied downstream by controller

    // Single authoritative store. The previous "thetaLpfHz on top of comp
    // filter" stage is gone — the complementary filter alone is now the
    // smoothing path. Lower α if more smoothing is needed; that adds
    // accel-following lag rather than the dedicated phase-lag of a second
    // LPF in series.
    shared::g.theta.store(theta, std::memory_order_relaxed);

    // Body-frame X accel. Diagnostic only: a stationary tilted bot reads
    // ax ≈ -sin(theta), so subtracting -sin(theta) leaves real
    // translational accel. Useful to tell whether a wobble is the chassis
    // physically translating (=> mechanical / wheel-coupling problem) or
    // just gyro picking up structural vibration without bulk motion (=>
    // signal / mounting problem).
    shared::g.accelX.store(axG, std::memory_order_relaxed);

    // Notch on thetaDotFilt, applied BEFORE storing to shared::g.thetaDot
    // for the controller. Targets a known structural-resonance peak
    // (e.g. ~7 Hz limit cycle observed on the bench) without nuking the
    // bandwidth a low-pass at the same depth would.
    //
    // Direct Form I, RBJ cookbook coefficients:
    //   ω0 = 2π fc / fs,  α = sin(ω0)/(2Q)
    //   b0 = 1, b1 = -2cos(ω0), b2 = 1
    //   a0 = 1+α, a1 = -2cos(ω0), a2 = 1-α
    // Normalised so a0=1. Coefficients are recomputed only when fc/Q
    // change, so the steady-state path is just six multiplies + four
    // adds per sample.
    //
    // notchFc <= 0 disables (passthrough). Phase response is well-behaved
    // away from fc (only the narrow band around the centre is rotated),
    // so unlike thetaLpfHz this should NOT add meaningful phase lag to
    // the inner loop at the controller's working frequencies (<3 Hz).
    static float nb0 = 1.0f, nb1 = 0.0f, nb2 = 0.0f;
    static float na1 = 0.0f, na2 = 0.0f;
    static float nx1 = 0.0f, nx2 = 0.0f, ny1 = 0.0f, ny2 = 0.0f;
    static float lastFc = -1.0f, lastQ = -1.0f;
    const float nFc = params::current.notchFc;
    const float nQ  = params::current.notchQ;
    float thetaDotOut = thetaDotFilt;
    if (nFc > 0.0f && nQ > 0.0f) {
      if (nFc != lastFc || nQ != lastQ) {
        const float fs    = (float)SAMPLE_HZ;
        const float w0    = 2.0f * 3.14159265f * nFc / fs;
        const float cw0   = cosf(w0);
        const float alpha = sinf(w0) / (2.0f * nQ);
        const float a0    = 1.0f + alpha;
        nb0 =  1.0f / a0;
        nb1 = -2.0f * cw0 / a0;
        nb2 =  1.0f / a0;
        na1 = -2.0f * cw0 / a0;
        na2 = (1.0f - alpha) / a0;
        lastFc = nFc;
        lastQ  = nQ;
        // Don't reset state on coefficient change; momentary glitch is
        // less harmful than a click during live tuning.
      }
      const float x = thetaDotFilt;
      const float y = nb0 * x + nb1 * nx1 + nb2 * nx2 - na1 * ny1 - na2 * ny2;
      nx2 = nx1; nx1 = x;
      ny2 = ny1; ny1 = y;
      thetaDotOut = y;
    } else {
      // Keep state coherent with input when disabled, so re-enabling
      // doesn't kick from zero.
      nx2 = nx1; nx1 = thetaDotFilt;
      ny2 = ny1; ny1 = thetaDotFilt;
      lastFc = -1.0f; // force recompute on next enable
    }
    shared::g.thetaDot.store(thetaDotOut, std::memory_order_relaxed);

    const uint32_t nowMs = millis();
    if (nowMs - lastLogMs >= (uint32_t)LOG_PERIOD_MS) {
      lastLogMs = nowMs;
      Serial.print(F("imu: theta="));
      Serial.print(theta * (180.0f / 3.14159265f), 2);
      Serial.print(F("° thetaDot="));
      Serial.print(thetaDotRaw * (180.0f / 3.14159265f), 1);
      Serial.print(F("°/s (acc="));
      Serial.print(thetaAccRaw * (180.0f / 3.14159265f), 2);
      Serial.println(F("°)"));
    }
  }
}

} // namespace

bool start() {
  static bool started = false;
  if (started) return true;

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  // Bus scan: many GY-521 clones tie AD0 high and live at 0x69 instead of
  // the library default 0x68. Logging every responder also distinguishes
  // "wrong address" from "nothing on the bus" (wiring / pull-ups / power).
  Serial.println(F("imu: scanning I2C bus..."));
  bool found68 = false, found69 = false;
  int  nFound  = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("imu: I2C device @ 0x"));
      Serial.println(addr, HEX);
      ++nFound;
      if (addr == 0x68) found68 = true;
      if (addr == 0x69) found69 = true;
    }
  }
  if (nFound == 0) {
    Serial.println(F("imu: no I2C devices found — check SDA/SCL wiring, "
                     "VCC/GND, and pull-ups (most modules have them)"));
    return false;
  }

  // Pick whichever MPU-class address answered. Prefer 0x68 (AD0 low, default).
  // Construct the lib instance with the right address — there is no setter
  // post-construction; devAddr is captured in the ctor.
  uint8_t mpuAddr = 0x68;
  if (found69 && !found68) mpuAddr = 0x69;
  if (!found68 && !found69) {
    Serial.println(F("imu: I2C devices found but none at 0x68/0x69 — "
                     "is the MPU6050 actually wired?"));
    return false;
  }
  Serial.print(F("imu: using MPU6050 @ 0x"));
  Serial.println(mpuAddr, HEX);
  g_mpu = new MPU6050(mpuAddr);

  // Read WHO_AM_I (reg 0x75) directly. The MPU6050 lib's testConnection()
  // expects exactly 0x68 (genuine InvenSense MPU-6050). Many cheap GY-521
  // boards actually carry an MPU-6500 (returns 0x70/0x72), MPU-9250 (0x71),
  // or other pin-compatible clones whose register map for accel/gyro is
  // identical for our purposes. We accept any plausible ID and only bail on
  // 0x00/0xFF (bus stuck low/high — wiring fault).
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(mpuAddr, (uint8_t)1);
  uint8_t whoAmI = Wire.available() ? Wire.read() : 0xFF;
  Serial.print(F("imu: WHO_AM_I=0x"));
  Serial.println(whoAmI, HEX);
  if (whoAmI == 0x00 || whoAmI == 0xFF) {
    Serial.println(F("imu: invalid WHO_AM_I — bus stuck, giving up"));
    return false;
  }
  if (whoAmI != 0x68) {
    Serial.println(F("imu: not a genuine MPU-6050 (likely MPU-6500/9250 "
                     "clone); accel/gyro register map is compatible, "
                     "proceeding"));
  }

  g_mpu->initialize();

  // Configure ranges + on-chip DLPF. DLPF_BW_42 corresponds to ~42 Hz cutoff
  // on the gyro and accel; well above our control bandwidth, well below
  // structural resonances on a small chassis.
  g_mpu->setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  g_mpu->setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  g_mpu->setDLPFMode(MPU6050_DLPF_BW_42);
  // Sample rate divider: internal sample rate = 1 kHz / (1+div). div=4 → 200 Hz.
  g_mpu->setRate(4);

  // No auto-calibration. Bias is loaded from params::current at imuTask
  // entry; user triggers a fresh calibration via the web UI when needed.

  started = true;
  xTaskCreatePinnedToCore(
    imuTask, "imu", 4096, nullptr, 4, nullptr, CORE_IMU);
  return true;
}

bool isReady() { return g_ready; }

bool requestCalibration() {
  if (!g_ready) return false;
  // Only IDLE → PENDING is a legal transition from outside imuTask. If
  // we're mid-calibration or holding a result the caller hasn't consumed,
  // refuse rather than silently clobbering it. compare_exchange_strong
  // makes this race-free against a concurrent state read.
  uint8_t expected = static_cast<uint8_t>(CalState::IDLE);
  return g_calState.compare_exchange_strong(
      expected, static_cast<uint8_t>(CalState::PENDING),
      std::memory_order_acq_rel, std::memory_order_acquire);
}

CalState calibrationState() {
  return static_cast<CalState>(g_calState.load(std::memory_order_acquire));
}

void clearCalibrationState() {
  // Caller is responsible for only doing this after observing a terminal
  // state. We don't enforce it because the only consumer is net.cpp and
  // the misuse would only re-arm a fresh request, which is safe.
  g_calState.store(static_cast<uint8_t>(CalState::IDLE),
                   std::memory_order_release);
}

} // namespace imu
