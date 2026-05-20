// safety.h — arming state machine and motor enable gate. F8.
//
// PLAN.md §8:
//
//   DISARMED ──(stand-still + vBat ok + IMU ready)──→ READY
//   READY    ──(requestArm)──────────────────────────→ ARMED
//   ARMED    ──(|θ| > fallAngle for 100 ms)──────────→ FALLEN
//   ARMED    ──(FLAG_LOW_BAT raised)─────────────────→ LOW_BAT
//   ARMED    ──(requestDisarm)───────────────────────→ DISARMED
//   FALLEN/LOW_BAT ──(requestReset)──────────────────→ DISARMED
//
// safetyTask is the only writer of:
//   - the EN pin (PIN_DRV_EN, active-low). HIGH (disabled) in any state
//     other than ARMED. Driven LOW only on entry to ARMED.
//   - shared::FLAG_MOTORS_ENABLED. Mirrors EN: set on entry to ARMED,
//     cleared on every exit.
//   - shared::FLAG_FALLEN. Set on transition to FALLEN, cleared on RESET.
//
// External actors (WS / joystick / serial) request transitions via
// requestArm() / requestDisarm() / requestReset(). These are tiny atomic
// flags consumed by the task; they don't drive transitions directly so the
// state machine can apply its precondition checks (e.g. won't arm if IMU
// isn't ready or battery is low).
//
// Runs at 100 Hz on core 1 (PLAN.md §3, highest priority among control-side
// tasks). The tilt cutoff debounce is implemented inside this loop, so the
// 100 Hz rate gives ~10 ms granularity which comfortably resolves the
// configured 100 ms FALL_DEBOUNCE_MS.

#pragma once

#include <stdint.h>

class Print; // Arduino base output stream; forward-declared at file scope.

namespace safety {

enum class State : uint8_t {
  DISARMED = 0,
  READY    = 1,
  ARMED    = 2,
  FALLEN   = 3,
  LOW_BAT  = 4,
};

// One-time init: spawns safetyTask on core 1. Idempotent.
void start();

// Current FSM state. Cheap; reads an atomic.
State state();

// Single-letter text for log lines / telemetry. "D/R/A/F/L".
const char* stateName(State s);

// Transition requests. Each is a one-shot bool the task consumes; calling
// the same one twice in quick succession is a no-op. Safe to call from any
// task (joystick, WS, serial console, ...).
void requestArm();
void requestDisarm();
void requestReset(); // FALLEN/LOW_BAT → DISARMED

} // namespace safety
