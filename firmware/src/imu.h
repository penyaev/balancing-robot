// imu.h — MPU6050 sampling + complementary filter (PLAN.md §7).
//
// Owns its own task on core 1 sampling at CONTROL_LOOP_HZ (200 Hz). Outputs
// fused tilt and tilt rate to shared::g.theta / g.thetaDot. Sign convention
// follows PLAN.md: +theta = body tipping toward +x. The actual mapping from
// IMU axes to "x" depends on how the IMU is mounted in the chassis; if the
// sign comes out wrong, flip the corresponding gain sign during F7/F9
// tuning rather than re-spinning this module.
//
// Calibration: gyro bias is NO LONGER auto-zeroed at boot. The bias values
// live in ControlParams (gyroBias{X,Y,Z}) and are persisted to NVS like any
// other tunable; on start() the IMU loads them from params::current. The
// user explicitly recalibrates via the web UI, which sends {type:"calibrate"}
// — see net.cpp::handleCalibrate. Rationale: a stale-but-known bias from a
// previous calibration is preferable to the old behaviour of silently
// re-running the 2 s window every boot (which fails if the chassis is
// moving and produced a 1–3°/s drift surprise on the bench).

#pragma once

#include <stdint.h>

namespace imu {

// Configures I²C, brings up the MPU6050, loads the persisted gyro bias
// from params::current, and starts the sampling task. Idempotent. Returns
// false on connection error (caller should still proceed; the task won't
// be started but the rest of the firmware can run).
bool start();

// True after the IMU task has come up and produced its first sample.
bool isReady();

// Calibration request API. The actual calibration runs inside imuTask
// (it owns the I²C bus); this API is a thread-safe handshake usable from
// any task — typically the WS handler.
//
// State machine values returned by calibrationState():
//   IDLE     — no calibration pending or running.
//   PENDING  — request accepted, imuTask hasn't picked it up yet.
//   RUNNING  — imuTask is sampling; theta/thetaDot are NOT being updated.
//   DONE_OK  — finished; gyroBias{X,Y,Z} in params::current have been
//              updated. Caller should consume this state (call clear())
//              before the next request.
//   DONE_ERR — finished but failed (chassis moved too much, etc.); bias
//              has been zeroed. Same consumption requirement.
enum class CalState : uint8_t {
  IDLE     = 0,
  PENDING  = 1,
  RUNNING  = 2,
  DONE_OK  = 3,
  DONE_ERR = 4,
};

// Request a calibration. Returns false if a calibration is already in
// progress (state != IDLE) or if the IMU isn't ready. On true, the caller
// should poll calibrationState() until it reaches DONE_OK / DONE_ERR.
bool requestCalibration();

CalState calibrationState();

// Reset state to IDLE after observing a terminal state. Safe to call from
// the same task that consumed the result.
void clearCalibrationState();

} // namespace imu
