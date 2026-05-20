# balancing-bot firmware

ESP32 firmware for the physical 2-wheel self-balancing robot. See
[`PLAN.md`](./PLAN.md) for the full design, decisions, and TODO list.
For the project overview and how this fits with the coproc and web UI,
see the [top-level README](../README.md).

## Status

Bootstrap, params + NVS, shared state, battery monitor, IMU +
complementary filter, TMC2209 + FastAccelStepper, cascaded PID
controller, safety FSM, binary telemetry, PS5 DualSense joystick
(classic BT), INA226 power monitor, and the UART link to the coproc
are all in. WiFi and the WS/HTTP surface live on the coproc now (see
`../coproc/`); main has no wireless other than Bluetooth. Hardware
bringup + tuning ongoing.

## Build

```sh
pio run                                 # compile
pio run -t upload                       # flash
pio device monitor                      # serial @ 115200
```

Only the PS5 controller MAC is a build flag on this side; WiFi
credentials moved to the coproc:

```ini
build_flags =
  -DBB_PS5_MAC=\"a0:fa:9c:14:de:f6\"   ; ESP32 advertises this MAC
```

Empty default falls back to scan-and-pair on first boot.

## Hardware

- ESP32-WROOM-32
- 2× TMC2209 stepper drivers (STEP/DIR + UART2)
- 2× 17HS19-2004S1 NEMA17 steppers, 103 mm direct-drive wheels
- MPU6050 IMU on I²C (400 kHz)
- 4S LiPo + buck to 5 V for logic, INA226 high-side monitor on the I²C bus
- PS5 DualSense over classic Bluetooth (HamzaYslmn/esp-ps5)
- INA226 high-side bus voltage + current monitor on the same I²C bus
- UART1 to the XIAO ESP32-S3 coproc (WiFi/HTTP/WS relay) @ 460800 baud

## Connections

GPIO numbers refer to the ESP32-WROOM-32. Authoritative source is
[`src/config.h`](./src/config.h); this table is for reading at a
glance. **L** = left wheel, **R** = right wheel, **both** = wired to
both drivers.

### MPU6050 (I²C)

| ESP32        | MPU6050 |
|--------------|---------|
| GPIO 21 SDA  | SDA     |
| GPIO 22 SCL  | SCL     |
| GPIO 19      | INT (optional, unused for now) |
| 3V3          | VCC     |
| GND          | GND     |

### TMC2209 stepper drivers

Two drivers on a **shared half-duplex UART bus**, addressed via their
MS1/MS2 strap pins. Per-wheel STEP/DIR are unique; everything else is
shared between L and R.

| ESP32              | Wired to                      | Notes |
|--------------------|-------------------------------|-------|
| GPIO 25            | TMC2209 **L** STEP            |       |
| GPIO 26            | TMC2209 **L** DIR             |       |
| GPIO 27            | TMC2209 **R** STEP            |       |
| GPIO 14            | TMC2209 **R** DIR             |       |
| GPIO 33            | TMC2209 EN of **both** L + R  | Active-low. Held HIGH (disabled) at boot; safety FSM (F8) drops LOW only on ARM. |
| GPIO 17 (UART2 TX) | TMC2209 PDN_UART of **both** L + R, **through a 1 kΩ series resistor** | Half-duplex bus shared by both drivers. |
| GPIO 16 (UART2 RX) | Same PDN_UART node as above   | Drivers tri-state when not addressed; the ESP32 listens on the same wire. Some TMC2209 carriers expose separate TX/RX pads internally bridged — either is fine. |
| 5 V (from buck)    | TMC2209 VIO + VM logic        | VM motor supply comes from pack via the driver carrier's VMOT input — see datasheet. |
| GND                | GND of both drivers           | Common ground with ESP32 + battery. |

Driver address straps (set on the carrier with MS1/MS2 jumpers):

| Driver | MS1 | MS2 | UART address (`DRV_ADDR_*`) |
|--------|-----|-----|------------------------------|
| L      | 0   | 0   | 0                            |
| R      | 1   | 0   | 1                            |

### INA226 power monitor

Shares the I²C bus with the MPU6050 (GPIO 21 SDA / GPIO 22 SCL).
Provides both pack voltage (used as `vBat`) and current draw.

### UART link to coproc

| ESP32              | Wired to                          | Notes |
|--------------------|-----------------------------------|-------|
| GPIO 4  (UART1 TX) | XIAO ESP32-S3 GPIO 44 (silk "RX") | main → coproc telemetry / status / params @ 460800 baud |
| GPIO 13 (UART1 RX) | XIAO ESP32-S3 GPIO 43 (silk "TX") | coproc → main forwarded WS commands |

### Status LED

| ESP32   | Wired to                                                   |
|---------|------------------------------------------------------------|
| GPIO 2  | Onboard blue LED on most WROOM-32 dev boards. No external wiring needed. |

### Power

- 4S LiPo (≈ 14.8 V nominal, 16.8 V full) → TMC2209 VMOT directly.
- Same pack → 5 V buck → ESP32 5V/VIN + TMC2209 VIO.
- All grounds common.

### Reserved / unused

GPIO 32 and GPIO 35 are reserved for future encoder support and not
wired in this build.

Pin map: [`src/config.h`](./src/config.h).

## Pairing the PS5 controller

The library makes the ESP32 pretend to be a PS5 console with a chosen
MAC. The controller has to be paired against that MAC once.

1. Pick any unicast MAC (e.g. `a0:fa:9c:14:de:f6`), set `BB_PS5_MAC`.
2. Pair the controller to that MAC by either of:
   - Plug the controller into a real PS5 / a phone, read its MAC, then
     swap the build flag to that value (no controller-side change).
   - Or write the chosen MAC into the controller via the
     `ps5SetBluetoothMacAddress` example sketch, while it is plugged in.
3. Flash this firmware. Hold PS button — controller's player LED goes
   solid white once connected. `joystick: PS5 connected` on serial.

## Status LED patterns

100 ms ticks, 1 s repeat. Priority: FALLEN > LOW_BAT > normal.

| State    | Pattern                              |
|----------|--------------------------------------|
| normal   | brief pulse (100 ms on, 900 ms off)  |
| LOW_BAT  | 50 % fast blink                      |
| FALLEN   | 3 quick pulses, long gap             |
