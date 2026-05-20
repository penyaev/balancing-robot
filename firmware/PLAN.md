# PLAN.md — ESP32 firmware for the 2-wheel self-balancing robot

Sister project to the simulator under `simulator/`. The firmware runs the full
control loop on-device and exposes a network interface so the simulator can
plot live telemetry and tune parameters remotely.

This document is the single source of truth for the firmware effort. It locks
in design decisions, captures the architecture, and lists the implementation
TODO so a future session (with no prior context) can pick up cleanly.

---

## 1. Locked-in decisions

| #  | Decision                | Choice                                                                                              | Rationale                                                                       |
|----|-------------------------|-----------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------|
| 1  | Architecture            | **A: Standalone bot**                                                                               | WiFi-out-of-loop; bot keeps balancing if comms drop.                            |
| 2  | MCU                     | **ESP32-WROOM-32** (plain)                                                                          | Has BR/EDR + BLE; PS5 controller works cleanly via Bluepad32.                   |
| 3  | Build system            | **PlatformIO**, framework = arduino, board = `esp32dev`                                             | Multi-file C++; lib mgmt; CI-friendly.                                          |
| 4  | TMC2209 control         | **STEP/DIR + UART** (single shared UART bus, drivers addressed 0/1)                                 | Allows runtime current/microstep/stealth tuning over network.                   |
| 5  | Step generation         | **FastAccelStepper**                                                                                | RMT/MCPWM-backed, smooth accel ramps, non-blocking.                             |
| 6  | PS5 stack               | **Bluepad32**                                                                                       | Mature, well-documented, supports DualSense LEDs/rumble.                        |
| 7  | Pin map                 | Proposed below (§4).                                                                                | None pre-existing.                                                              |
| 8  | Drivetrain              | Direct drive, **wheel diameter = 103 mm** (r = 0.0515 m, circ. = 0.324 m)                           | User-confirmed.                                                                 |
| 9  | Power                   | **4S LiPo** (~14.8 V nom, 16.8 V full, 12.0 V cutoff). Stepper supply direct; **buck to 5 V** logic | 4S is well within TMC2209's 4.75–29 V range and gives the steppers headroom.    |
| 10 | Safety cutoff           | `\|θ\| > 35°` for >100 ms → motors off. `V_batt < 12.0 V` for >2 s → motors off + low-bat flag.     | Standard.                                                                       |
| 11 | Param persistence       | **NVS** via `Preferences` lib                                                                       | Built-in, atomic, no FS overhead.                                               |
| 12 | Telemetry format        | **Binary little-endian struct over WS binary frames** at 100 Hz; **JSON** WS text for control       | Efficient hot path, debuggable cold path.                                       |
| 13 | Steering                | None for now; both wheels in sync.                                                                  | User-confirmed.                                                                 |
| 14 | First milestone         | **Skeleton only**: project structure, pin map, FreeRTOS scaffolding, stubbed subsystems, LED blink. | Lock the architecture before filling in subsystems.                             |

---

## 2. Control architecture (differs from simulator!)

Steppers in STEP/DIR mode are **velocity sources**, not torque sources. The
inner loop becomes a wheel-velocity command rather than a torque command.

```
        target_v (from PS5 left stick Y, scaled to [-v_max, +v_max])
            │
            ▼
   ┌─ outer (vel) PID ──→  θ_set   (clamped to ±θ_max, e.g. 0.25 rad)
   │
   │   inputs: target_v, x_dot_estimate
   │   gains:  velKp, velKi, velKd  (with anti-windup, optional LPF on dErr)
   │
   ▼
   ┌─ inner (tilt) PD ──→  v_wheel_setpoint  (m/s linear; same to both wheels)
   │
   │   v_wheel = velFF * target_v + Kθ * (θ − θ_set) + Kθ̇ * θ̇
   │
   │   - velFF: feed-forward so a step in target_v immediately commands wheels.
   │   - Kθ, Kθ̇: PD on tilt. Sign chosen so a forward tilt commands wheels
   │     forward (cart drives toward fall direction).
   ▼
   step_rate = v_wheel / (2π·r) · steps_per_rev · microsteps    [Hz]
```

`x_dot_estimate` comes from the commanded wheel velocity (steppers don't slip
below pull-out torque). Optionally cross-check against IMU's accelerometer-
derived linear accel.

This is intentionally **simpler** than the simulator's torque-mode controller.
We may later add a "stepper mode" to the simulator for like-for-like tuning;
that is a separate effort.

---

## 3. FreeRTOS task layout

| Task            | Core | Prio    | Period             | Job                                                                       |
|-----------------|------|---------|--------------------|---------------------------------------------------------------------------|
| `controlTask`   | 1    | high    | 5 ms (200 Hz)      | Read IMU, run filter, run cascaded controller, set step rates.            |
| `joystickTask`  | 0    | mid     | event-driven       | Bluepad32 callbacks → `target_v`, `target_yaw` (unused for now).          |
| `wsServerTask`  | 0    | mid     | async              | AsyncWebSocket: accept connections, dispatch JSON control messages.       |
| `telemetryTask` | 0    | low     | 10 ms (100 Hz)     | Pack binary telemetry frame, broadcast to all WS clients.                 |
| `safetyTask`    | 1    | highest | 10 ms              | Tilt cutoff, battery cutoff, watchdog feeder. Owns motor enable.          |

Inter-task data: a single `SharedState` struct in a header, accessed under a
`portMUX_TYPE` spinlock. Hot-path fields (`target_v`, current `θ`, `θ̇`,
`v_wheel_cmd`, `vbat`) use `std::atomic<float>` to keep the control loop
lock-free.

---

## 4. Pin map (proposed)

Pins chosen to avoid input-only / strapping conflicts (GPIO 0/2/12/15) for outputs.

| Function                         | Pin     | Notes                                                            |
|----------------------------------|---------|------------------------------------------------------------------|
| MPU6050 SDA                      | GPIO 21 | I²C0 default                                                     |
| MPU6050 SCL                      | GPIO 22 | I²C0 default                                                     |
| MPU6050 INT                      | GPIO 19 | optional, for interrupt-driven sampling                          |
| TMC2209 left STEP                | GPIO 25 |                                                                  |
| TMC2209 left DIR                 | GPIO 26 |                                                                  |
| TMC2209 right STEP               | GPIO 27 |                                                                  |
| TMC2209 right DIR                | GPIO 14 |                                                                  |
| TMC2209 EN (shared)              | GPIO 33 | active-low; pulled high (disabled) at boot                       |
| TMC2209 UART TX (to drivers RX)  | GPIO 17 | UART2 TX                                                         |
| TMC2209 UART RX (from drivers TX)| GPIO 16 | UART2 RX, 1 kΩ in series per driver, MS1/MS2 set addresses       |
| Battery V_sense                  | GPIO 34 | input-only; voltage divider 100 k / 22 k → ~3.0 V at 16.8 V pack |
| Status LED                       | GPIO 2  | onboard blue LED                                                 |
| Reserved (future encoder A/B)    | 32, 35  |                                                                  |

Drivers addressed 0 (left) and 1 (right) via MS1/MS2 strapping. Microstepping
default 16×; configured at runtime over UART.

---

## 5. Wire-level protocols

### 5.1 Telemetry frame (binary, little-endian, fixed size = 52 bytes)

```c
struct TelemetryFrame {
  uint32_t magic;        // 0xB0B0B0B0
  uint32_t seq;          // monotonic
  float    t;            // [s] uptime
  float    theta;        // [rad]
  float    thetaDot;     // [rad/s]
  float    xDot;         // [m/s] estimated
  float    thetaSet;     // [rad] outer-loop output
  float    vWheelCmd;    // [m/s] inner-loop output
  float    outerP, outerI, outerD;   // [rad]
  float    vBat;         // [V]
  uint16_t flags;        // bit0 fallover, bit1 lowBat, bit2 motorsEnabled, bit3 psConnected
  uint16_t reserved;
}; // 13 × 4 bytes
```

### 5.2 Control messages (JSON over WS text frames)

Client → server:

```json
{ "type": "setParam", "path": "control.angleKp", "value": 4.2 }
{ "type": "setTarget", "v": 0.4 }
{ "type": "enableMotors", "value": true }
{ "type": "reset" }
{ "type": "getParams" }
{ "type": "saveParams" }
{ "type": "loadDefaults" }
```

Server → client (control channel only):

```json
{ "type": "params", "params": { ...full ControlParams... } }
{ "type": "ack", "of": "saveParams" }
{ "type": "error", "msg": "..." }
```

Discovery: ESP32 advertises mDNS `balancebot.local` so the simulator can
connect without knowing the IP.

---

## 6. Parameter set on firmware (mirrors `ControlParams` shape)

```c
struct ControlParams {
  // Outer (vel -> tilt setpoint)
  float velKp, velKi, velKd;
  float velIClamp;        // [rad]
  float maxAngleSetpoint; // [rad]
  float velDLowpassHz;    // optional, planned

  // Inner (tilt -> wheel velocity)
  float Kth, KthDot;      // PD on tilt
  float velFF;            // feed-forward target_v -> v_wheel
  float thetaTrim;        // [rad] static tilt offset for chassis CG

  // Limits
  float vMaxCart;         // [m/s] target_v scale from joystick
  float vMaxWheel;        // [m/s] saturation on inner-loop output
  float aMaxWheel;        // [m/s²] acceleration ramp passed to FastAccelStepper
  float fallAngle;        // [rad] e.g. 35° → 0.61
  float vBatCutoff;       // [V]   12.0

  // Driver
  uint16_t microsteps;    // 16
  float    runCurrent;    // [A] e.g. 1.4
  float    holdCurrent;   // [A] e.g. 0.7

  // Filter
  float compAlpha;        // [-] complementary filter blend (0..1)
};
```

Persisted to NVS namespace `bb` with keys per field. `Preferences` lib handles
atomic writes.

---

## 7. Sensor fusion

MPU6050 at 1 kHz raw, decimated/filtered to 200 Hz. **Complementary filter**
for first cut (`compAlpha ≈ 0.01`); single-knob, easy to reason about. Madgwick
available behind a build flag if drift becomes an issue.

Pitch axis is the balancing axis; we use `accel_x, accel_z` for the gravity
reference and gyro Y for rate. Gyro bias auto-calibrated at boot with a 2 s
"stand still" detector — require `|gyro|` and `|accel - 1g|` below thresholds
for 2 s before arming motors.

---

## 8. Safety logic (`safetyTask`)

State machine:

```
DISARMED ──(wait for: stand-still detected, vBat ok, IMU healthy)──→ READY
READY    ──(client sends enableMotors=true OR PS5 "PS" button)─────→ ARMED
ARMED    ──(|θ| > fallAngle for 100 ms)────────────────────────────→ FALLEN
ARMED    ──(vBat < vBatCutoff for 2 s)─────────────────────────────→ LOW_BAT
ARMED    ──(client sends enableMotors=false OR PS5 "Options")──────→ DISARMED
FALLEN/LOW_BAT ──(client sends reset)──────────────────────────────→ DISARMED
```

In any disarmed/fallen/low-bat state: drive EN high (motors off), zero step
rates. Status flags reflected in telemetry frame.

Battery cutoff has hysteresis: 12.0 V to disable, 12.5 V to re-enable.

---

## 9. File layout

```
firmware/
  PLAN.md
  platformio.ini
  README.md
  src/
    main.cpp              // setup(): init NVS, WiFi, BT, IMU, motors; create tasks; loop() empty
    config.h              // pin map, build-time constants, version string
    shared_state.{h,cpp}  // SharedState struct + atomics + lock helpers
    params.{h,cpp}        // ControlParams + defaults + NVS load/save + JSON (de)serialize
    imu.{h,cpp}           // MPU6050 init, sampling, complementary filter
    motors.{h,cpp}        // FastAccelStepper init, TMC2209 UART config, setWheelVelocity()
    controller.{h,cpp}    // cascaded controller (outer PID, inner PD-on-tilt + FF)
    joystick.{h,cpp}      // Bluepad32 PS5 callbacks → SharedState targets
    net.{h,cpp}           // WiFi STA connect, mDNS, HTTP root, AsyncWebSocket, JSON dispatch
    telemetry.{h,cpp}     // binary frame packer + 100 Hz broadcast task
    safety.{h,cpp}        // state machine, fall/lowBat detection, motor enable gate
    battery.{h,cpp}       // ADC read, divider scaling, exponential smoothing
    util/log.h            // tiny logging macro wrapper
  test/                   // future: native unit tests for controller math
```

---

## 10. `platformio.ini` sketch

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_speed = 921600
build_flags =
  -DCORE_DEBUG_LEVEL=3
  -DBB_FW_VERSION=\"0.1.0\"
lib_deps =
  electroniccats/MPU6050
  gin66/FastAccelStepper
  teemuatlut/TMCStepper
  ricaun/Bluepad32          ; superseded — using rodneybakiskan/ps5-esp32 instead (F12)
  esphome/AsyncTCP-esphome  ; or ESP32Async/AsyncTCP
  esphome/ESPAsyncWebServer-esphome
  bblanchon/ArduinoJson
```

---

## 11. Simulator-side integration plan (separate, AFTER firmware skeleton works)

Captured here so it's not forgotten; not part of the firmware scope.

- Add `simulator/src/datasource.ts` with `SimulatorSource` (current local sim)
  and `DeviceSource` (WS client to bot).
- Add connection panel in toolbar: host input, Connect button, status LED
  (disconnected/connecting/connected/error), and a `Source: Sim | Device`
  toggle.
- In `Device` mode:
  - Charts fed by `DeviceSource` parsing 56-byte frames.
  - Param panel `onChange` sends `{type:"setParam", path, value}` instead of
    mutating local params.
  - Arrow keys send `{type:"setTarget", v}`.
  - Reset/Kick: Reset sends `{type:"reset"}`; Kick is sim-only — hide or
    grey out.
  - On connect: send `getParams`, wait for `params` reply, populate UI.
- Optionally overlay the simulator's own model output on the same charts
  (dashed) for comparison.

---

## 12. Implementation TODO (in order)

Each item is a commit-sized chunk.

### Firmware

- [ ] **F1.** Bootstrap PlatformIO project: `platformio.ini`, `main.cpp` with
  serial banner + LED blink task, `config.h` with pin map. Compile + flash +
  watch banner. *(milestone: hello world on hardware.)*
- [ ] **F2.** `params.{h,cpp}`: `ControlParams` struct, defaults, NVS save/load,
  JSON (de)serialize via ArduinoJson. Unit-callable from `main` for sanity.
- [ ] **F3.** `shared_state.{h,cpp}`: atomics + spinlock skeleton. Hook into
  `main`.
- [ ] **F4.** `battery.{h,cpp}`: ADC read on GPIO34, divider scaling, smoothing.
  Print on serial every 1 s.
- [ ] **F5.** `imu.{h,cpp}`: MPU6050 init via I²C, raw sampling at 1 kHz on a
  timer or in a task, complementary filter, expose `theta` / `thetaDot` to
  `SharedState`. Log values; verify by tilting board.
- [ ] **F6.** `motors.{h,cpp}`: TMC2209 UART config (current, microsteps,
  stealth), FastAccelStepper init for both wheels, `setWheelVelocity(left,
  right)` API. Bench test with small commanded velocity (chassis on blocks).
- [ ] **F7.** `controller.{h,cpp}`: cascaded controller as described in §2.
  Wired in `controlTask` at 200 Hz. **Test on bench with motors EN-disabled
  first**, log `v_wheel_cmd` and verify sign convention by hand-tilting.
- [ ] **F8.** `safety.{h,cpp}`: state machine, fall/lowBat detection, motor
  enable gate. Forces motors off until ARMED.
- [ ] **F9.** **First on-bot balance attempt.** Iterate Kθ, Kθ̇, then outer
  PID. Expect to bang on this for several sessions.
- [ ] **F10.** `net.{h,cpp}`: WiFi STA, mDNS, AsyncWebServer + AsyncWebSocket.
  Implement JSON control-message dispatch (setParam, setTarget, enableMotors,
  reset, getParams, saveParams, loadDefaults).
- [ ] **F11.** `telemetry.{h,cpp}`: pack `TelemetryFrame`, broadcast at 100 Hz
  to all WS clients.
- [x] **F12.** `joystick.{h,cpp}`: rodneybakiskan/ps5-esp32 init, PS5 pairing
  via BB_PS5_MAC, map left-stick Y → `target_v`, PS → arm, Share → disarm,
  Options → reset. Switched partition table to `no_ota.csv` (2 MB app) to
  fit Bluedroid alongside WiFi/AsyncWebServer. Bluepad32 ruled out: its
  canonical PIO route requires migrating to ESP-IDF + arduino-as-component.
- [x] **F13.** Polish: status LED encodes FALLEN / LOW_BAT / normal as
  100 ms-tick patterns. Top-level README added (project overview);
  firmware/README updated with pairing instructions + LED legend.

### Simulator

- [ ] **S1.** `datasource.ts` abstraction; refactor existing code to use
  `SimulatorSource`. No behavior change.
- [ ] **S2.** `DeviceSource` + connection panel + binary-frame parser.
- [ ] **S3.** Switch param panel + arrow keys to send WS messages in `Device`
  mode.
- [ ] **S4.** (optional) Overlay sim model on live charts.

---

## 13. Risks / things to revisit

- **Stepper torque ceiling.** 17HS19-2004S1 is heavy and has lots of torque,
  but if commanded acceleration exceeds pull-out torque the wheels stall and
  the bot falls instantly. Conservative `aMaxWheel` to start (e.g. 5 m/s²) and
  ramp up.
- **Microstepping vs. step rate.** At v_wheel = 1.5 m/s, 16× microstep →
  ~15 kHz step rate (fine). 32× → 30 kHz (still fine for FastAccelStepper).
  Don't go higher without checking.
- **Complementary filter tuning.** Wrong `compAlpha` → either drift (too gyro-
  trusting, alpha too low) or sluggish (too accel-trusting, alpha too high).
  Exposed as a tunable param so we can iterate from the simulator UI.
- **WiFi blocking.** AsyncWebServer is non-blocking but mDNS init can take
  seconds. Make sure WiFi connect is in setup() *after* control task is
  already running; bot should self-balance even if WiFi never connects.
- **Bluepad32 + WiFi coexistence.** ESP32 shares the radio. Both work
  simultaneously but there's a known bandwidth tradeoff. Should be fine at
  our data rates; flag if telemetry stutters.
- **Battery sag under stepper load.** Voltage divider should be RC-filtered
  (1 k + 1 µF) to reject spikes; cutoff hysteresis (12.0 V to disable, 12.5 V
  to re-enable) to avoid chattering.

---

## 14. Context for a fresh session resuming this work

If you're picking this up cold:

- The simulator under `simulator/` is a separate, working Vite + TypeScript
  app. It is NOT the firmware. It models the bot with a torque-source motor
  and a cascaded PID. It will eventually grow a "Device" data-source mode to
  consume telemetry from this firmware (see §11).
- The firmware does **not** port the simulator's controller verbatim. Real
  steppers are velocity sources, so the inner loop here outputs a wheel-
  velocity setpoint, not a torque (see §2). The simulator's tuned gains will
  not transfer.
- Hardware: ESP32-WROOM-32, 2× TMC2209 + 17HS19-2004S1 NEMA17 steppers, MPU6050
  IMU, 4S LiPo, PS5 DualSense over Bluetooth.
- Start at TODO **F1** unless instructed otherwise. Each F* is a commit-sized
  chunk; follow them in order, keep the bot disarmed until F8 is in place.
- Param tuning happens live from the simulator UI in `Device` mode (after
  S1–S3). Until then, expose a serial-console fallback or hardcode for
  bringup.
