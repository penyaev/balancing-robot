// uartbus.h — full-duplex UART codec for the main↔coproc link.
//
// Replaces the old joystick-only one-way receiver. Main now talks to the
// XIAO ESP32-S3 Sense coproc over Serial1, in both directions:
//
//   main → coproc:
//     - 60 Hz telemetry frames (PKT_TELEMETRY, identical body to
//       telemetry::Frame; coproc re-broadcasts to WS clients)
//     - 1 Hz status snapshots (PKT_STATUS; coproc renders status page
//       from the cached body)
//     - on-change params dumps (PKT_PARAMS, JSON; coproc caches and
//       returns on WS "getParams")
//
//   coproc → main:
//     - JSON WS commands (PKT_WS_CMD; cmdrx.cpp dispatches them via the
//       same ArduinoJson-based handlers net.cpp used to use)
//
// Framing is in wire_proto.h. This module owns Serial1, the TX mutex,
// and the RX task. Producers (telemetry/diag/joystick callbacks) call
// the send* functions from any task — they serialize through the mutex
// onto Serial1.write(). Receiver runs its own task at CORE_COMMS, drains
// Serial1, runs a state-machine parser, and on a valid PKT_WS_CMD calls
// the callback registered by cmdrx::start().

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace uartbus {

// Callback type for incoming PKT_WS_CMD packets. Payload is the raw
// JSON byte sequence (NOT null-terminated). The callback runs on the
// rx task; it must not block for long. Typical implementation: copy
// the bytes into a local buffer and queue them for a deferred handler.
using CommandHandler = void (*)(const uint8_t* json, size_t len);

// Idempotent. Brings up Serial1 at COPROC_UART_BAUD on the
// PIN_COPROC_UART_TX/RX pins from config.h, allocates rx/tx buffers,
// spawns the rx task on core 0 at priority 1.
void start();

// Register the handler called for every well-formed PKT_WS_CMD packet.
// Pass nullptr to clear. Last writer wins; we only support one handler.
// Safe to call before or after start().
void setCommandHandler(CommandHandler cb);

// Pack and write a telemetry frame. Body is opaque (treated as N bytes
// to forward verbatim into the wire format). Typical caller: the
// telemetry task at 60 Hz with the packed Frame struct.
//
// Returns true on success, false if the link isn't up or the body is
// too large for MAX_PAYLOAD.
bool sendTelemetry(const uint8_t* body, size_t len);

// Same shape, for the 1 Hz status snapshot. Body layout is the
// agreed-upon binary struct defined in diag.h.
bool sendStatus(const uint8_t* body, size_t len);

// Same shape, for the params JSON dump. `json` is the byte sequence
// produced by serializeJson(); we do NOT require null termination but
// will pass it through to the coproc verbatim (coproc caches it and
// re-emits to WS clients on getParams).
bool sendParams(const uint8_t* json, size_t len);

// One-byte input-event frame (PKT_INPUT_EVENT). `code` is one of the
// wire::INPUT_* constants. Called from the PS5 input callback on a
// physical button edge — coproc uses these to advance the dot-matrix
// page without us having to round-trip through the WS layer.
bool sendInputEvent(uint8_t code);

} // namespace uartbus
