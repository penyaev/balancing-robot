# coproc — Seeed XIAO ESP32-S3 Sense WiFi/HTTP/WS relay

A small companion ESP32-S3 that owns the wireless I/O the main control
board doesn't deal with. WiFi STA + AsyncWebServer + AsyncWebSocket on
this side; main owns classic Bluetooth (DualSense joystick) and the
control loop. Each board has one radio role and no coex risk between
them — which was the symptom that motivated the split: Bluedroid on the
WROOM-32 truncated WiFi response bodies once a controller was paired.

The XIAO's onboard OV2640 camera + PDM mic + 8 MB PSRAM are dormant in
the current scope but the hardware is in the right place for the video
+ audio streaming phases later.

## Scope

- **Now.** Receive 60 Hz telemetry + 1 Hz status + on-change params
  packets from the main ESP32 over Serial1 (UART, 460800 baud); cache
  the latest of each; relay telemetry to web UI clients over the
  binary WebSocket at `/ws`; render the cached status snapshot as the
  HTML page at `GET /`. WS commands (setParam, setTarget, reset, …)
  flow the other way: web UI → coproc WS → UART → main's command
  dispatcher.

- **Later.** OV2640 → JPEG MJPEG to a video pane in the web UI.
  PDM mic → I²S → PCM over WebSocket to an audio pane.

## Wire protocol

See `../firmware/src/wire_proto.h` for the canonical definition; this
project builds with `-I../firmware/src` so both sides share one header.
Telemetry frame layout is in `telemetry.h`, status snapshot layout in
`diag.h`.

## Build

Plain `framework = arduino` on the stock `espressif32` platform — no
IDF components, no bootstrap script.

```sh
cd coproc
pio run                                    # build
pio run -t upload                          # flash over USB-C
pio device monitor                          # serial console (USB-CDC)
```

WiFi credentials and the mDNS name are build flags in `platformio.ini`:

```ini
build_flags =
  -DBB_WIFI_SSID=\"YourSSID\"
  -DBB_WIFI_PASS=\"YourPass\"
  -DBB_MDNS_NAME=\"balancebot\"   ; → ws://balancebot.local/ws
```

## Wiring to the main ESP32

| coproc XIAO pin               | main ESP32 pin     | direction        |
|-------------------------------|--------------------|------------------|
| GPIO 43 (D6, silkscreen "TX") | GPIO 13 (UART1 RX) | coproc → main    |
| GPIO 44 (D7, silkscreen "RX") | GPIO 4  (UART1 TX) | main   → coproc  |
| 5V                            | 5V buck rail       | shared power     |
| GND                           | GND                | common ground    |

Authoritative pin assignments live in `../firmware/src/config.h`
(`PIN_COPROC_UART_TX/RX`, `COPROC_UART_BAUD`).

## Coproc-local peripherals

| coproc XIAO pin       | peripheral                     |
|-----------------------|--------------------------------|
| GPIO 7  (D8, SCK)     | MAX7219 8x32 display — CLK     |
| GPIO 8  (D9)          | MAX7219 8x32 display — CS/LOAD |
| GPIO 9  (D10, MOSI)   | MAX7219 8x32 display — DIN     |
| GPIO 41               | onboard PDM mic — DATA         |
| GPIO 42               | onboard PDM mic — CLK          |
| GPIO 10–18, 38–40, 47–48 | onboard OV2640 camera bus   |
