// safety.cpp — see safety.h.
//
// FSM evaluation runs at SAFETY_LOOP_HZ on core 1 with priority above the
// controller. The actual GPIO write to PIN_DRV_EN happens only on state
// *transitions*, never on every tick — that keeps the EN line clean and
// avoids inadvertent pulses from a glitching loop.
//
// Fall debounce: |theta| > fallAngle must hold for FALL_DEBOUNCE_MS before
// FALLEN is latched. We track the moment the threshold was first crossed
// and check elapsed time on every tick. If the threshold un-crosses, the
// counter resets immediately.
//
// Note: low-battery is fully owned by battery.cpp (FLAG_LOW_BAT, with
// hysteresis + 2 s debounce). We just consume the flag here. That keeps
// the cutoff policy in one place.

#include "safety.h"

#include <Arduino.h>
#include <atomic>
#include <math.h>

#include "config.h"
#include "imu.h"
#include "motors.h"
#include "params.h"
#include "shared_state.h"

namespace safety {
namespace {

constexpr BaseType_t CORE_SAFETY = 1;
constexpr int SAFETY_PERIOD_MS = 1000 / SAFETY_LOOP_HZ; // 10 ms

// Public state. Single-writer (safetyTask), many readers (telemetry, log).
std::atomic<State> g_state{State::DISARMED};

// One-shot transition requests. cleared by the task when consumed.
std::atomic<bool> req_arm{false};
std::atomic<bool> req_disarm{false};
std::atomic<bool> req_reset{false};

bool started = false;

// Drive the EN line and FLAG_MOTORS_ENABLED together so they can never get
// out of sync. EN is active-low: HIGH = drivers OFF, LOW = drivers ON.
void setMotorsEnabled(bool on) {
  digitalWrite(PIN_DRV_EN, on ? LOW : HIGH);
  shared::setFlag(shared::FLAG_MOTORS_ENABLED, on);
}

// Common transition helper: log + apply motor gate + commit state.
void enterState(State next, const char* reason) {
  const State prev = g_state.load();
  if (prev == next) return;

  // Motors enabled iff target is ARMED. Disable on every other transition.
  // Doing this BEFORE the state switch closes a race where a controller
  // tick reading FLAG_MOTORS_ENABLED+ARMED could push step rates after we
  // logically left ARMED.
  if (next != State::ARMED) {
    setMotorsEnabled(false);
    motors::stop();
  }

  g_state.store(next);

  if (next == State::ARMED) {
    setMotorsEnabled(true);
  }
  if (next == State::FALLEN) {
    shared::setFlag(shared::FLAG_FALLEN, true);
  }
  if (next == State::DISARMED) {
    shared::setFlag(shared::FLAG_FALLEN, false);
  }

  Serial.print(F("safety: "));
  Serial.print(stateName(prev));
  Serial.print(F(" -> "));
  Serial.print(stateName(next));
  Serial.print(F(" ("));
  Serial.print(reason);
  Serial.println(F(")"));
}

// READY preconditions per PLAN.md §8: stand-still, vBat ok, IMU healthy.
// We approximate "stand-still" by combining imu::isReady() (the IMU task
// only flips that after its 2 s stand-still bias-cal completed) with a
// |theta| < fallAngle*0.5 sanity check, so that DISARMED → READY only
// happens with the bot upright. After a fall, the user has to physically
// right the bot before it can arm again.
bool readyPreconditions() {
  if (!imu::isReady()) return false;
  if (shared::getFlag(shared::FLAG_LOW_BAT)) return false;
  const float th = shared::g.theta.load();
  const float lim = 0.5f * params::current.fallAngle;
  if (fabsf(th) > lim) return false;
  return true;
}

void safetyTask(void* /*arg*/) {
  // Belt and braces: even though main.cpp already drove EN HIGH at boot,
  // assert it here so the state machine owns the pin from now on.
  pinMode(PIN_DRV_EN, OUTPUT);
  setMotorsEnabled(false);

  TickType_t nextWake = xTaskGetTickCount();

  // Time at which |theta| first exceeded fallAngle in the current ARMED
  // session. 0 means "currently within bounds".
  uint32_t fallSinceMs = 0;

  // Auto-arm dwell tracking. autoArmInWindowSinceMs holds the millis()
  // timestamp at which |theta - thetaTrim| first entered the auto-arm
  // window in the current READY session; 0 means "currently outside the
  // window or auto-arm not actively dwelling". autoArmInhibited is set on
  // every entry to DISARMED (manual disarm or post-fall reset), which
  // requires the user to physically tilt the bot past autoArmAngle once
  // before auto-arm can engage again — otherwise a manual disarm while
  // balanced would re-arm autoArmHoldMs later.
  uint32_t autoArmInWindowSinceMs = 0;
  bool     autoArmInhibited       = true;

  // Tracks last-seen FSM state so we can react to *transitions* (not just
  // residency) — used here to re-arm the auto-arm inhibit on every entry
  // to DISARMED.
  State prevState = g_state.load();

  for (;;) {
    vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(SAFETY_PERIOD_MS));

    const State s = g_state.load();
    const uint32_t nowMs = millis();

    // Consume requests up-front. Each is a one-shot; clearing it now means
    // a duplicate request a tick later still gets one shot.
    const bool wantArm    = req_arm.exchange(false);
    const bool wantDisarm = req_disarm.exchange(false);
    const bool wantReset  = req_reset.exchange(false);

    // Read once per tick.
    const float theta = shared::g.theta.load();
    const float fallLim = params::current.fallAngle;
    const bool  lowBat  = shared::getFlag(shared::FLAG_LOW_BAT);

    switch (s) {
      case State::DISARMED:
        if (readyPreconditions()) {
          enterState(State::READY, "preconditions ok");
        }
        break;

      case State::READY:
        // Slip back to DISARMED if any precondition fails (IMU drops out,
        // user tilts the bot past the upright sanity band, low-bat raised).
        if (!readyPreconditions()) {
          enterState(State::DISARMED, "preconditions lost");
          break;
        }
        if (wantArm) {
          fallSinceMs = 0;
          autoArmInWindowSinceMs = 0;
          enterState(State::ARMED, "armed");
          break;
        }
        // Auto-arm. Triggers a READY -> ARMED transition automatically
        // when the trim-corrected tilt has stayed inside ±autoArmAngle
        // for autoArmHoldMs continuous milliseconds. The inhibit flag
        // (set on every entry to DISARMED) requires the user to physically
        // tilt the bot past the threshold once before re-engaging, so a
        // manual disarm while balanced doesn't immediately re-arm. Disabled
        // entirely when params.autoArmEnabled <= 0.5.
        if (params::current.autoArmEnabled > 0.5f) {
          const float effTheta = theta - params::current.thetaTrim;
          const float aaAng    = params::current.autoArmAngle;
          if (fabsf(effTheta) <= aaAng) {
            if (!autoArmInhibited) {
              if (autoArmInWindowSinceMs == 0) autoArmInWindowSinceMs = nowMs;
              const uint32_t holdMs =
                  (uint32_t)params::current.autoArmHoldMs;
              if ((nowMs - autoArmInWindowSinceMs) >= holdMs) {
                fallSinceMs = 0;
                autoArmInWindowSinceMs = 0;
                enterState(State::ARMED, "auto-armed");
                break;
              }
            }
            // else: inhibited — do nothing, wait for the user to tilt
            // past autoArmAngle once.
          } else {
            // Outside the window: drop dwell, and (importantly) clear
            // the inhibit so the *next* time we enter the window the
            // dwell counter starts accumulating.
            autoArmInWindowSinceMs = 0;
            autoArmInhibited = false;
          }
        } else {
          // Auto-arm disabled by param: don't carry stale dwell state.
          autoArmInWindowSinceMs = 0;
        }
        break;

      case State::ARMED: {
        // Battery cutoff: trust the battery module's already-debounced flag.
        if (lowBat) {
          enterState(State::LOW_BAT, "vBat below cutoff");
          break;
        }
        // Tilt cutoff with debounce.
        if (fabsf(theta) > fallLim) {
          if (fallSinceMs == 0) fallSinceMs = nowMs;
          if ((nowMs - fallSinceMs) >= (uint32_t)FALL_DEBOUNCE_MS) {
            enterState(State::FALLEN, "tilt cutoff");
            break;
          }
        } else {
          fallSinceMs = 0;
        }
        if (wantDisarm) {
          enterState(State::DISARMED, "disarmed");
        }
        break;
      }

      case State::FALLEN:
      case State::LOW_BAT:
        // Stay latched until the user explicitly resets. This is the
        // hard-fail bucket — don't auto-recover even if conditions
        // improve, because a fall typically means the user has to right
        // the bot manually anyway, and a low-battery pack briefly
        // recovering above 12.5 V doesn't mean it's safe to keep going.
        if (wantReset) {
          enterState(State::DISARMED, "reset");
        }
        break;
    }

    // Edge-detect state transitions to maintain auto-arm bookkeeping.
    // Re-arm the inhibit on every entry to DISARMED so a manual disarm
    // (or post-fall reset) doesn't immediately auto-rearm; the user has
    // to physically tilt the bot past autoArmAngle once more before
    // dwell tracking will resume in READY.
    const State curState = g_state.load();
    if (curState != prevState) {
      if (curState == State::DISARMED) {
        autoArmInhibited = true;
        autoArmInWindowSinceMs = 0;
      }
      prevState = curState;
    }
  }
}

} // namespace

void start() {
  if (started) return;
  started = true;
  // Highest priority among control-side tasks (controller is 4); safety
  // must always run on time.
  xTaskCreatePinnedToCore(safetyTask, "safety", 3072, nullptr,
                          /*priority=*/6, nullptr, CORE_SAFETY);
}

State state() { return g_state.load(); }

const char* stateName(State s) {
  switch (s) {
    case State::DISARMED: return "DISARMED";
    case State::READY:    return "READY";
    case State::ARMED:    return "ARMED";
    case State::FALLEN:   return "FALLEN";
    case State::LOW_BAT:  return "LOW_BAT";
  }
  return "?";
}

void requestArm()    { req_arm.store(true); }
void requestDisarm() { req_disarm.store(true); }
void requestReset()  { req_reset.store(true); }

} // namespace safety
