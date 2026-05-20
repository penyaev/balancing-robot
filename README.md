# balancing-robot

A 2-wheel self-balancing robot — physical hardware plus a browser-based
web UI that live-tunes parameters, charts telemetry, and streams the
on-board camera/mic over WebSocket. The bot is split across two ESP32s
with focused, non-overlapping wireless responsibilities:

- **main board (ESP32-WROOM-32)** — control loop, classic-BT DualSense
  joystick. No WiFi.
- **coproc (XIAO ESP32-S3 Sense)** — WiFi STA + AsyncWebServer + WS
  relay. No BT. Camera + microphone hardware on the same chip is
  dormant for now (Phases B/C).

```
                                  classic BT
   DualSense ──────────────────────────────┐
                                           ▼
                       ┌────────────────────────────────┐
                       │  main  (ESP32-WROOM-32)         │
                       │  esp-ps5 joystick callback      │
                       │  IMU, steppers, cascaded PID    │
                       │  safety FSM, telemetry producer │
                       └────────────┬───────────────────┘
                                    │ UART1 @ 460800 baud, full-duplex
                                    │   main → coproc: telemetry,
                                    │                  status, params
                                    │   coproc → main: WS commands
                                    ▼
                       ┌────────────────────────────────┐
                       │  coproc  (XIAO ESP32-S3 Sense)  │
                       │  WiFi STA + mDNS                │
                       │  AsyncWebServer + AsyncWebSocket│
                       │  GET / status page,             │
                       │  WS /ws telemetry broadcast     │
                       │  (camera/mic dormant for now)   │
                       └────────────┬───────────────────┘
                                    │ WiFi STA
                                    ▼
                       ┌────────────────────────────────┐
                       │  webui/ (browser)               │
                       │  Vite + TypeScript live UI      │
                       │  charts, tuning, camera/audio   │
                       │  ws://balancebot.local/ws       │
                       └────────────────────────────────┘
```

Onshape model available [here](https://cad.onshape.com/documents/6b9fd9304d8a6d27f15f411d/w/72b36573f85ef1e41fca52e4/e/ba4c9141accb33bb5640b345?renderMode=0&uiState=6a0df08f280133a7f9adecf1)

## Repo layout

- **`webui/`** — TypeScript / Vite browser app. Connects to the bot
  over WebSocket: live telemetry charts, firmware parameter tuning,
  camera/audio panes, and dot-matrix display controls. `npm run dev`.
- **`firmware/`** — PlatformIO project for the main ESP32-WROOM-32.
  Owns the whole control loop on-device so the bot stays balanced even
  if WiFi drops on the coproc. `pio run`. Full design + TODO list in
  [`firmware/PLAN.md`](./firmware/PLAN.md).
- **`coproc/`** — PlatformIO project for the XIAO ESP32-S3 Sense
  coprocessor. WiFi/HTTP/WS relay between the web UI and main board.
  Plain Arduino framework — no IDF gymnastics. See
  [`coproc/README.md`](./coproc/README.md).

Shared wire-format contracts live in `firmware/src/` and are included
by `coproc/` via a `-I../firmware/src` build flag:

- [`wire_proto.h`](./firmware/src/wire_proto.h) — generic typed-packet
  framing (magic + type + length + payload + CRC8) used for every UART
  exchange.
- [`telemetry.h`](./firmware/src/telemetry.h) — packed 116-byte
  telemetry frame struct.
- [`diag.h`](./firmware/src/diag.h) — 1 Hz status snapshot struct.

## Architecture in one paragraph

The bot's outer loop maps the desired cart velocity into a tilt
setpoint (PID with anti-windup); the inner loop maps tilt error into a
wheel-velocity command (PD + feed-forward) which the TMC2209 stepper
drivers track via FastAccelStepper. The MPU6050 feeds a complementary
filter at 200 Hz. A separate safety FSM owns the motor enable line and
arms the bot only when the IMU, battery, and tilt all look sane.
Telemetry is emitted as a 116-byte little-endian frame at 60 Hz over
the UART link to the coproc, which relays it to all WebSocket clients;
control commands come back over the same UART as forwarded JSON. A PS5
DualSense pairs to the main board directly over classic Bluetooth and
sets the velocity setpoint + arms / disarms / triggers reset.

## Hardware

ESP32-WROOM-32 · 2× TMC2209 + 17HS19-2004S1 NEMA17 (103 mm direct-drive
wheels) · MPU6050 · 4S LiPo. Pin map: [`firmware/src/config.h`](./firmware/src/config.h).

## Status

Main firmware: control loop, safety FSM, joystick, INA226 power
monitor, and the UART link to the coproc are in. Coproc: WiFi/HTTP/WS
relay in. The web UI talks to the real bot end-to-end (telemetry,
status page, params + commands round-trip, camera + audio streams,
dot-matrix display control). Hardware bringup + tuning ongoing.
Detailed firmware history in PLAN.md.
