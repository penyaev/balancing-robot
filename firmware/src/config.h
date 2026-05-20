// config.h — build-time constants and pin map for the balancing-bot firmware.
//
// Pin assignments come from PLAN.md §4. Keep this file dependency-free
// (only standard headers + Arduino macros) so any module can include it.

#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// Firmware version
// ---------------------------------------------------------------------------
#ifndef BB_FW_VERSION
#define BB_FW_VERSION "0.0.0-dev"
#endif

// ---------------------------------------------------------------------------
// Pin map (ESP32-WROOM-32). See PLAN.md §4.
//
// Avoid GPIO 0/2/12/15 for outputs (strapping pins). GPIO 34/35/36/39 are
// input-only.
// ---------------------------------------------------------------------------

// I²C — MPU6050 + INA226 (shared bus)
constexpr int PIN_I2C_SDA   = 21;
constexpr int PIN_I2C_SCL   = 22;
constexpr int PIN_IMU_INT   = 19; // optional, for interrupt-driven sampling

// INA226 high-side power monitor. Default I²C address with A0=A1=GND.
// Shunt resistance is module-dependent — confirm from your board's
// silkscreen before flashing; the calibration register depends on it.
// Module type: 2 mΩ (R002, high-current variant) — gives ±16.384 A
// full-scale at 0.5 mA/LSB.
constexpr uint8_t INA226_ADDR        = 0x40;
constexpr float   INA226_SHUNT_OHMS  = 0.002f;

// IMU axis sign for the pitch (= body Y) gyro, relative to the
// accelerometer-derived theta convention (theta>0 = body tipping toward +x;
// see imu.cpp::thetaFromAccel). The accel sets the convention because
// gravity gives an unambiguous physical reference; the gyro is a rate
// sensor whose sign depends on which way the chip is bolted down.
//
// Symptom this guards against: on a fast manual tilt to +30°, the filtered
// theta first dips to ~-10° (gyro integration pushing the wrong way) and
// then over ~tau seconds the accel pulls it up to +30°. That shape is the
// signature of a gyro/accel sign mismatch on the pitch channel — flip
// this constant and the dip disappears.
//
// On the current bench mounting we observed exactly that signature, so
// the gyro pitch reading is negated. If you remount the IMU rotated
// 180° about the axle, set this back to +1.
constexpr float IMU_GYRO_PITCH_SIGN = -1.0f;

// Stepper drivers (TMC2209)
constexpr int PIN_STEP_L    = 25;
constexpr int PIN_DIR_L     = 26;
constexpr int PIN_STEP_R    = 27;
constexpr int PIN_DIR_R     = 14;
constexpr int PIN_DRV_EN    = 33; // active-low; held high (disabled) at boot
constexpr int PIN_DRV_UART_TX = 17; // UART2 TX → driver RX (shared bus)
constexpr int PIN_DRV_UART_RX = 16; // UART2 RX ← driver TX

// TMC2209 UART addresses (set on hardware via MS1/MS2 strap pins). The two
// drivers share one half-duplex UART; the address byte in each datagram
// selects which one responds. PLAN.md §4.
constexpr uint8_t DRV_ADDR_L = 0; // MS1=0, MS2=0
constexpr uint8_t DRV_ADDR_R = 1; // MS1=1, MS2=0

// TMC2209 sense-resistor value. The BIGTREETECH / FYSETC clones used in
// this build all ship with 0.11 Ω; verify on your specific board.
constexpr float DRV_RSENSE_OHM = 0.11f;

// UART baud for the driver bus. 115200 is the conservative default that
// works without external pull-ups on most cabling; TMCStepper supports
// higher rates if needed.
constexpr uint32_t DRV_UART_BAUD = 115200;

// Wheel mounting: depending on which side the motor body faces, "forward"
// for the cart may correspond to opposite physical rotation directions on
// the two wheels. These flags get applied in motors::setWheelVelocity().
// Decided empirically on first bench test (F6) and refined during the
// first balancing attempt: with the IMU sign convention where +θ means
// the bot is tipping toward +x, we need a positive vWheel command to
// drive the wheels in the +x direction (under the falling body) so the
// inner law's `Kth·θerr` term restores rather than amplifies the tilt.
constexpr bool MOTOR_INVERT_L = false;
constexpr bool MOTOR_INVERT_R = true;

// Wheel-velocity deadband applied in the LEDC step generator. Any
// per-wheel command with magnitude < MOTOR_DEADBAND_STEPS (steps/sec)
// is collapsed to 0 (ledcWriteTone(0) → STEP idle, no edges). This
// suppresses the small Kth*θerr noise that hovers around upright,
// which would otherwise emit sub-50 Hz STEP edges that the chip
// stretches over ~20 ms anyway. Originally introduced as a workaround
// for a FastAccelStepper queue-stuffing pathology — see git log; the
// hysteresis is still useful with the LEDC backend. 50 steps/s ≈ 5 mm/s
// at the wheel (with default microsteps), well below any meaningful
// command and well above IMU-noise jitter. Tunable for experiments.
constexpr int32_t MOTOR_DEADBAND_STEPS = 0;

// TMC2209 chopper mode at runtime.
//   true  → spreadCycle: instant torque from rest, audible whine,
//           higher current draw at standstill. Recommended for a
//           balancer where every ms of wake-from-rest matters.
//   false → stealthChop: silent, gentler current ramp, slight torque
//           lag at low speeds. Useful for quiet bring-up / debugging.
// Changing this also flips bit 2 of EXPECTED_GCONF in motors_drv.cpp,
// which the driver-drift watchdog uses to detect post-brownout chips.
constexpr bool MOTOR_USE_SPREADCYCLE = false;

// Status LED — onboard blue LED on most WROOM dev boards
constexpr int PIN_STATUS_LED = 2;

// Reserved for future encoder support; not used yet.
constexpr int PIN_RESERVED_A = 32;
constexpr int PIN_RESERVED_B = 35;

// UART1 to the XIAO ESP32-S3 coprocessor (now the WiFi/HTTP/WS relay;
// see firmware/src/uartbus.cpp and coproc/main/sketch.cpp). Both pins
// are general-purpose GPIO on the WROOM-32 with no strapping
// requirement; UART1 is otherwise unused (UART0 is the serial
// console, UART2 is the TMC2209 driver bus). The wire format is in
// wire_proto.h.
//
// Direction is full-duplex: main TX (telemetry/status/params) → coproc
// RX; main RX ← coproc TX (forwarded WS commands).
//
// Baud bumped 115200 → 460800 because the new direction (main→coproc)
// carries the full 60 Hz telemetry stream that used to go over WS;
// ~8 KB/s of traffic doesn't fit comfortably in 11 KB/s of 115200
// capacity.
constexpr int PIN_COPROC_UART_TX = 4;   // → coproc UART RX
constexpr int PIN_COPROC_UART_RX = 13;  // ← coproc UART TX
constexpr uint32_t COPROC_UART_BAUD = 460800;

// ---------------------------------------------------------------------------
// Mechanical / drivetrain constants
// ---------------------------------------------------------------------------
constexpr float WHEEL_RADIUS_M       = 0.0515f;  // 103 mm diameter wheels
constexpr int   STEPPER_FULLSTEPS    = 200;      // 1.8°/step
constexpr int   DEFAULT_MICROSTEPS   = 16;

// ---------------------------------------------------------------------------
// Loop rates (Hz)
// ---------------------------------------------------------------------------
constexpr int CONTROL_LOOP_HZ    = 200;
constexpr int TELEMETRY_HZ       = 60;
constexpr int SAFETY_LOOP_HZ     = 100;

// ---------------------------------------------------------------------------
// Battery monitoring (4S LiPo)
// ---------------------------------------------------------------------------
constexpr float VBAT_CUTOFF_V      = 13.2f;
constexpr float VBAT_RECOVER_V     = 13.7f;   // hysteresis

// ---------------------------------------------------------------------------
// Safety
// ---------------------------------------------------------------------------
constexpr float FALL_ANGLE_RAD     = 0.61f;   // ~35°
constexpr int   FALL_DEBOUNCE_MS   = 100;
constexpr int   LOWBAT_DEBOUNCE_MS = 2000;
