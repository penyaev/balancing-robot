// controller.h — cascaded balancing controller. F7.
//
// Architecture (PLAN.md §2):
//
//   target_v ─► outer PID(velKp/Ki/Kd) ─► θ_set (clamped ±maxAngleSetpoint)
//                                          │
//   θ, θ̇ (from imu) ───────────────────────┼──► inner PD+FF
//                                          │
//        v_wheel = velFF·target_v + Kθ·(θ−θ_set) + Kθ̇·θ̇
//
// `x_dot_estimate` for the outer PID is the previous-tick commanded wheel
// velocity (steppers don't slip below pull-out torque, so commanded ≈
// actual). `v_wheel` is saturated to ±vMaxWheel and pushed to the motors
// driver — same value to both wheels (no yaw control yet).
//
// Spawns a single 200 Hz task on core 1 (`controlTask`) per the FreeRTOS
// layout in PLAN.md §3. While FLAG_MOTORS_ENABLED is clear (i.e. before F8
// arms the bot, or any time safety dropped EN), the controller still runs
// the math and publishes its outputs for debugging, but holds the
// integrator at zero and commands the motors driver to stop. That way the
// telemetry stream (F11) shows live controller behaviour without needing
// motion.

#pragma once

namespace controller {

// One-time init: spawns the 200 Hz controlTask pinned to core 1. Idempotent.
void start();

// True iff start() has been called and the task is running.
bool isRunning();

} // namespace controller
