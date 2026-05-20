// params.cpp — see params.h.

#include "params.h"

#include <Arduino.h>
#include <Preferences.h>
#include <stddef.h>
#include <string.h>

namespace params {

namespace {

constexpr const char* NVS_NAMESPACE = "bb";

// Field tables. NVS keys are limited to 15 characters; a couple of long names
// get abbreviated for storage but keep their full identifiers in the API and
// JSON. The tables drive setByPath, toJson, fromJson, and NVS persistence so
// adding a parameter means touching one place.

struct FloatField {
  const char* name;     // identifier (also JSON key)
  const char* nvsKey;   // NVS key (<=15 chars)
  size_t      offset;   // offsetof(ControlParams, ...)
};

struct U16Field {
  const char* name;
  const char* nvsKey;
  size_t      offset;
};

#define BB_FIELD(name, nvs) { #name, nvs, offsetof(ControlParams, name) }

const FloatField FLOAT_FIELDS[] = {
  BB_FIELD(velKp,            "velKp"),
  BB_FIELD(velKi,            "velKi"),
  BB_FIELD(velKd,            "velKd"),
  BB_FIELD(velErrDeadband,   "velErrDB"),
  BB_FIELD(velIClamp,        "velIClamp"),
  BB_FIELD(maxAngleSetpoint, "maxAngSet"),
  BB_FIELD(velDAlpha,        "velDAlpha"),
  BB_FIELD(outerEnabled,     "outerEn"),
  BB_FIELD(velPAlpha,        "velPAlpha"),
  BB_FIELD(outerAlpha,       "outerAlpha"),
  BB_FIELD(outerEveryN,      "outerEvN"),
  BB_FIELD(innerEnabled,     "innerEn"),
  BB_FIELD(Kth,              "Kth"),
  BB_FIELD(KthDot,           "KthDot"),
  BB_FIELD(velFF,            "velFF"),
  BB_FIELD(thetaTrim,        "thetaTrim"),
  BB_FIELD(vMaxCart,         "vMaxCart"),
  BB_FIELD(vMaxWheel,        "vMaxWheel"),
  BB_FIELD(vMaxTurn,         "vMaxTurn"),
  BB_FIELD(aMaxWheel,        "aMaxWheel"),
  BB_FIELD(fallAngle,        "fallAngle"),
  BB_FIELD(vBatCutoff,       "vBatCutoff"),
  BB_FIELD(autoArmEnabled,   "autoArmEn"),
  BB_FIELD(autoArmAngle,     "autoArmAng"),
  BB_FIELD(autoArmHoldMs,    "autoArmHold"),
  BB_FIELD(runCurrent,       "runCurrent"),
  BB_FIELD(holdCurrent,      "holdCurrent"),
  BB_FIELD(compAlpha,        "compAlpha"),
  BB_FIELD(accAlpha,         "accAlpha"),
  BB_FIELD(gyroAlpha,        "gyroAlpha"),
  BB_FIELD(notchFc,          "notchFc"),
  BB_FIELD(notchQ,           "notchQ"),
  BB_FIELD(vWheelAlpha,      "vWheelAlpha"),
  BB_FIELD(gyroYawXTalk,     "gyroYXT"),
  BB_FIELD(targetVAlpha,     "targetVAlpha"),
  BB_FIELD(targetTurnAlpha,  "tgtTurnAlpha"),
  BB_FIELD(stickDeadband,    "stickDB"),
  BB_FIELD(stickExpo,        "stickExpo"),
  BB_FIELD(gyroBiasX,        "gyroBiasX"),
  BB_FIELD(gyroBiasY,        "gyroBiasY"),
  BB_FIELD(gyroBiasZ,        "gyroBiasZ"),
};

const U16Field U16_FIELDS[] = {
  BB_FIELD(microsteps,       "microsteps"),
};

#undef BB_FIELD

constexpr size_t NUM_FLOAT_FIELDS = sizeof(FLOAT_FIELDS) / sizeof(FLOAT_FIELDS[0]);
constexpr size_t NUM_U16_FIELDS   = sizeof(U16_FIELDS)   / sizeof(U16_FIELDS[0]);

inline float& floatRef(ControlParams& p, size_t off) {
  return *reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(&p) + off);
}
inline const float& floatRef(const ControlParams& p, size_t off) {
  return *reinterpret_cast<const float*>(reinterpret_cast<const uint8_t*>(&p) + off);
}
inline uint16_t& u16Ref(ControlParams& p, size_t off) {
  return *reinterpret_cast<uint16_t*>(reinterpret_cast<uint8_t*>(&p) + off);
}
inline const uint16_t& u16Ref(const ControlParams& p, size_t off) {
  return *reinterpret_cast<const uint16_t*>(reinterpret_cast<const uint8_t*>(&p) + off);
}

} // namespace

ControlParams current = defaults();

ControlParams defaults() {
  ControlParams p{};
  // Outer
  p.velKp            = 0.10f;
  p.velKi            = 0.05f;
  p.velKd            = 0.00f;
  // velErrDeadband=0 = disabled, original behaviour. Set non-zero from
  // the web UI to round small outer-loop velocity errors to zero (P=0,
  // integrator frozen, dErr=0 within the band). Suppresses the endless
  // tiny corrections the outer loop otherwise produces when xDotEst
  // jitters around 0 in standstill. See params.h for the trade-off.
  p.velErrDeadband   = 0.0f;
  p.velIClamp        = 0.15f;
  p.maxAngleSetpoint = 0.25f;
  // velDAlpha=1 = passthrough on the outer D term (post-velKd). Drop
  // below 1 to smooth differentiation noise; the legacy Hz-cutoff
  // form (velDLowpassHz=8 Hz on err pre-diff) is equivalent at α≈0.20
  // for outerDt=0.005 s — match that here so the field's tuning
  // semantics on a freshly-flashed bot mirror the previous behaviour.
  // State is reset to 0 on disarm/!imuOk/!outerOn by the controller;
  // see params.h::velDAlpha and controller.cpp.
  p.velDAlpha        = 0.20f;
  // outerEnabled=1 = outer PID active (normal balancing). Set to 0 from
  // the web UI to suppress the outer loop without code changes — handy
  // for separating "the bot can stand up" (inner PD) from "the bot can
  // track a commanded velocity" (outer PID) when tuning. Reset semantics
  // and zeroing of the outer-loop filter state live in controller.cpp.
  p.outerEnabled     = 1.0f;
  // velPAlpha=1 = passthrough on the outer P term (default behaviour).
  // Drop below 1 to smooth jitter from xDotEst quantisation through P
  // before it reaches thetaSet. Filter state is reset to 0 on each arm
  // by the control task; see params.h and controller.cpp.
  p.velPAlpha        = 1.0f;
  // outerAlpha=1 = passthrough on the composite thetaSet output (default
  // behaviour). Drop below 1 for a smoother setpoint trace; effective
  // sample period is outerDt = N·DT so the time-domain shape interprets
  // at the outer rate regardless of outerEveryN. State is held in the
  // control task and reset to 0 on disarm/!imuOk/!outerOn — same
  // convention as every other outer-loop ramp/LPF here.
  p.outerAlpha       = 1.0f;
  // outerEveryN=1 = outer PID re-evaluates every control tick (200 Hz),
  // legacy behaviour identical to before this knob existed. Raise to 4
  // for a 50 Hz outer / 200 Hz inner split — the typical cascaded-PID
  // arrangement when the outer dynamics (cart velocity) are slower than
  // the inner ones (tilt). See params.h::outerEveryN for the held-output
  // and dt-scaling mechanics that keep velKi/velKd meaning intact across
  // N values, and controller.cpp for the implementation. Reset to 1.0
  // on loadDefaults so a typo in the field can't permanently stall the
  // outer loop.
  p.outerEveryN      = 1.0f;
  // innerEnabled=1 = inner PD active (normal balancing). Set to 0 from
  // the web UI to drop the Kth·thetaErr and KthDot·thDot tilt-feedback
  // terms; only velFF·targetVUsed survives in the wheel command, so the
  // bot becomes an open-loop velocity drive (steering, vWheelAlpha,
  // saturation still applied). The bot WILL fall over without the
  // inner loop — this is a bench-test mode for driving the wheels
  // directly from the joystick (step-skip diagnosis, steering symmetry,
  // resonance hunts) without touching firmware. Symmetric mirror of
  // outerEnabled. Reset semantics: nothing extra to clear (no inner-PD
  // integrator); vWheelFilt continues from its existing value, which
  // smoothly ramps through any toggle thanks to the IIR.
  p.innerEnabled     = 1.0f;
  // Inner
  p.Kth              = 0.0f;   // tune empirically
  p.KthDot           = 0.0f;   // tune empirically
  p.velFF            = 1.0f;
  p.thetaTrim        = 0.0f;
  // Limits
  p.vMaxCart         = 0.8f;
  p.vMaxWheel        = 1.5f;
  p.vMaxTurn         = 0.5f;  // steering differential cap (m/s)
  p.aMaxWheel        = 30.0f;
  p.fallAngle        = 0.61f;  // ~35°
  p.vBatCutoff       = 12.0f;  // 4S LiPo low cutoff
  // Auto-arm: enabled, ±1° window, 500 ms dwell. See params.h for the
  // re-arm-inhibit rationale and safety.cpp for where it's applied.
  p.autoArmEnabled   = 1.0f;
  p.autoArmAngle     = 0.01745f;  // 1° in rad
  p.autoArmHoldMs    = 500.0f;
  // Driver
  p.microsteps       = 16;
  p.runCurrent       = 1.4f;
  p.holdCurrent      = 0.7f;
  // Filter
  // compAlpha and gyroAlpha are direct 1-pole IIR mix coefficients; the
  // controller no longer derives them from a frequency/τ at runtime, so the
  // tuning is decoupled from SAMPLE_HZ. Convert from the legacy
  // (compFilterTau=0.5 s, gyroLpfHz=25 Hz) defaults at SAMPLE_HZ=200:
  //   compAlpha = dt/(τ+dt) = 0.005/(0.505) ≈ 0.0099
  //   gyroAlpha = dt/(rc+dt), rc=1/(2π·25)=6.37 ms → 0.005/(0.01137) ≈ 0.44
  p.compAlpha        = 0.01f;
  // accAlpha=1 = passthrough (raw thetaAcc straight into the comp filter,
  // matches the original behaviour). Drop to 0.05–0.3 to denoise the accel
  // path independently of compAlpha. See params.h for the full rationale.
  p.accAlpha         = 1.0f;
  p.gyroAlpha        = 0.44f;
  p.notchFc          = 0.0f;   // disabled by default. Set to the observed
                               // limit-cycle frequency (e.g. 7 Hz) to
                               // suppress structural resonance feeding back
                               // through KthDot. See params.h.
  p.notchQ           = 5.0f;   // narrow-ish; tune in 3–10 range.
  // vWheelAlpha=1 = passthrough (raw inner-loop output goes straight to
  // the motors). Drop below 1 to smooth the actuator command at the cost
  // of phase lag inside the balancing loop. See params.h for guidance.
  p.vWheelAlpha      = 1.0f;
  // gyroYawXTalk=0 = original behaviour (no compensation). User tunes
  // empirically against an in-place spin; see params.h for procedure.
  p.gyroYawXTalk     = 0.0f;
  // targetVAlpha=1 = passthrough. Drop below 1 to ramp the commanded
  // velocity into the controller (smoother stick response, slower
  // tracking). Filter state is held in the controller task and reset
  // to 0 on disarm/!imuOk; see controller.cpp.
  p.targetVAlpha     = 1.0f;
  // targetTurnAlpha=1 = passthrough on the steering differential. Drop
  // below 1 to ramp turn rate; same lifecycle/reset semantics as
  // targetVAlpha. Independent so the operator can have a snappy turn
  // and a smooth forward ramp (or vice versa).
  p.targetTurnAlpha  = 1.0f;
  // Stick response curve. Defaults sized to absorb DualSense at-rest
  // jitter (~5% deadband) and add a moderate expo feel (^2) so small
  // stick deflections give finely controllable low-speed motion while
  // full stick gives full vMax{Cart,Turn}. Set stickExpo=1 for linear.
  // See params.h::stickDeadband / stickExpo for the pipeline.
  p.stickDeadband    = 0.05f;
  p.stickExpo        = 2.0f;
  // Gyro bias: zero by default. The IMU loads these at start-up; the user
  // populates them via the "Calibrate IMU" button (handleCalibrate in
  // net.cpp). Until then the bot will drift slightly with the chip's
  // factory bias offset, but it WILL run.
  p.gyroBiasX        = 0.0f;
  p.gyroBiasY        = 0.0f;
  p.gyroBiasZ        = 0.0f;
  return p;
}

int loadFromNvs(ControlParams& p) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/true)) {
    return 0;
  }
  int n = 0;
  for (const auto& f : FLOAT_FIELDS) {
    if (prefs.isKey(f.nvsKey)) {
      floatRef(p, f.offset) = prefs.getFloat(f.nvsKey, floatRef(p, f.offset));
      ++n;
    }
  }
  for (const auto& f : U16_FIELDS) {
    if (prefs.isKey(f.nvsKey)) {
      u16Ref(p, f.offset) = prefs.getUShort(f.nvsKey, u16Ref(p, f.offset));
      ++n;
    }
  }
  prefs.end();
  return n;
}

void saveToNvs(const ControlParams& p) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) {
    return;
  }
  for (const auto& f : FLOAT_FIELDS) {
    prefs.putFloat(f.nvsKey, floatRef(p, f.offset));
  }
  for (const auto& f : U16_FIELDS) {
    prefs.putUShort(f.nvsKey, u16Ref(p, f.offset));
  }
  prefs.end();
}

void clearNvs() {
  Preferences prefs;
  if (prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) {
    prefs.clear();
    prefs.end();
  }
}

bool setByPath(ControlParams& p, const char* name, float value) {
  if (!name) return false;
  for (const auto& f : FLOAT_FIELDS) {
    if (strcmp(name, f.name) == 0) {
      floatRef(p, f.offset) = value;
      return true;
    }
  }
  for (const auto& f : U16_FIELDS) {
    if (strcmp(name, f.name) == 0) {
      // Round-to-nearest, clamp to uint16 range.
      long iv = lroundf(value);
      if (iv < 0) iv = 0;
      if (iv > 65535) iv = 65535;
      u16Ref(p, f.offset) = static_cast<uint16_t>(iv);
      return true;
    }
  }
  return false;
}

void toJson(const ControlParams& p, JsonObject obj) {
  for (const auto& f : FLOAT_FIELDS) {
    obj[f.name] = floatRef(p, f.offset);
  }
  for (const auto& f : U16_FIELDS) {
    obj[f.name] = u16Ref(p, f.offset);
  }
}

int fromJson(ControlParams& p, JsonObjectConst obj) {
  int n = 0;
  for (const auto& f : FLOAT_FIELDS) {
    JsonVariantConst v = obj[f.name];
    if (!v.isNull()) {
      floatRef(p, f.offset) = v.as<float>();
      ++n;
    }
  }
  for (const auto& f : U16_FIELDS) {
    JsonVariantConst v = obj[f.name];
    if (!v.isNull()) {
      u16Ref(p, f.offset) = v.as<uint16_t>();
      ++n;
    }
  }
  return n;
}

void printSummary(const ControlParams& p, Print& out) {
  out.print(F("vel{Kp=")); out.print(p.velKp, 3);
  out.print(F(",Ki=")); out.print(p.velKi, 3);
  out.print(F(",Kd=")); out.print(p.velKd, 3);
  out.print(F(",")); out.print(p.outerEnabled > 0.5f ? F("on") : F("off"));
  out.print(F(",N=")); out.print(p.outerEveryN, 0);
  out.print(F(",oA=")); out.print(p.outerAlpha, 3);
  out.print(F("} tilt{Kth=")); out.print(p.Kth, 3);
  out.print(F(",KthDot=")); out.print(p.KthDot, 3);
  out.print(F(",FF=")); out.print(p.velFF, 2);
  out.print(F(",")); out.print(p.innerEnabled > 0.5f ? F("on") : F("off"));
  out.print(F(",tgtVA=")); out.print(p.targetVAlpha, 4);
  out.print(F("} maxSet=")); out.print(p.maxAngleSetpoint, 3);
  out.print(F(" fall=")); out.print(p.fallAngle, 2);
  out.print(F(" vBatCut=")); out.print(p.vBatCutoff, 2);
  out.print(F(" us=")); out.print(p.microsteps);
  out.print(F(" run=")); out.print(p.runCurrent, 2);
  out.print(F("A hold=")); out.print(p.holdCurrent, 2);
  out.print(F("A cAlpha=")); out.print(p.compAlpha, 4);
  out.print(F(" aAlpha=")); out.print(p.accAlpha, 4);
  out.print(F(" gAlpha=")); out.print(p.gyroAlpha, 4);
  out.print(F(" notch=")); out.print(p.notchFc, 1);
  out.print(F("Hz/Q")); out.print(p.notchQ, 1);
  out.print(F(" vWAlpha=")); out.print(p.vWheelAlpha, 4);
  out.print(F(" gYXT=")); out.print(p.gyroYawXTalk, 4);
  out.print(F(" autoArm{")); out.print(p.autoArmEnabled > 0.5f ? F("on") : F("off"));
  out.print(F(",ang=")); out.print(p.autoArmAngle, 4);
  out.print(F(",hold=")); out.print(p.autoArmHoldMs, 0);
  out.print(F("ms}"));
  out.print(F(" gBias[")); out.print(p.gyroBiasX, 3);
  out.print(F(",")); out.print(p.gyroBiasY, 3);
  out.print(F(",")); out.print(p.gyroBiasZ, 3);
  out.print(F("]dps"));
}

} // namespace params
