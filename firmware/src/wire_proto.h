// wire_proto.h — UART protocol between the main ESP32 and the XIAO S3 coproc.
//
// SCOPE: this header is *the* contract between two physically separate
// firmware projects:
//   - firmware/  (this directory) — main ESP32. Sends telemetry/status/params
//     packets (PKT_TELEMETRY / PKT_STATUS / PKT_PARAMS) over Serial1; receives
//     PKT_WS_CMD packets carrying JSON commands forwarded from the simulator
//     via the coproc's WebSocket. Decoder lives in firmware/src/uartbus.cpp.
//   - coproc/    (sibling directory, XIAO ESP32-S3 Sense) — receives those
//     packets, caches the latest of each, and relays telemetry over WS +
//     renders the status page. Includes this file via
//     build_flags=-I../firmware/src to share the wire format verbatim.
//
// One source of truth, two readers. Don't fork it.
//
// PACKET FRAMING (both directions, identical):
//
//   offset  size  field
//   0       1     magic1 = 0xA5
//   1       1     magic2 = 0x5A
//   2       1     type   — see PKT_* constants below
//   3       2     length — uint16 LE, number of payload bytes (excludes
//                          the header and the trailing CRC)
//   5..L+4  L     payload — opaque to the framer; meaning is per-type
//   L+5     1     crc8 over bytes 0..L+4 inclusive (Dallas/Maxim CRC-8,
//                 poly 0x07, init 0x00, no reflection, no xor-out)
//
// So minimum frame size = 6 bytes (zero-payload), maximum = 6 + 65535 bytes
// (in practice we cap at a couple of KB — see MAX_PAYLOAD below).
//
// Why these choices:
//
// * Single packet format for all four types keeps the rx state machine
//   trivial: scan for magic → read 3 header bytes → read payload → check
//   CRC → dispatch by type. No per-type framing logic.
//
// * Magic+CRC+length-prefix is enough to recover from a noisy boot
//   transient or a partial-frame mid-stream: we drop until magic
//   re-appears and validate via CRC8 before acting.
//
// * 460800 baud is plenty for our worst-case mix (~60 Hz × 116 B telemetry
//   + 1 Hz × few hundred B status/params + ad-hoc commands ≈ 7.9 KB/s vs
//   46 KB/s capacity). Don't drop below ~230400 if changing — the
//   telemetry stream alone is ~7 KB/s.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace wire {

constexpr uint8_t MAGIC1 = 0xA5;
constexpr uint8_t MAGIC2 = 0x5A;

// Packet types. Allocated in ranges so the direction is obvious from the
// byte: 0x10-0x1F = main→coproc, 0x20-0x2F = coproc→main. Pick the next
// available type in the appropriate range when adding new ones.
constexpr uint8_t PKT_TELEMETRY = 0x10;  // main→coproc, ~60 Hz, ~116 B
                                         //   body. Body is the exact
                                         //   telemetry::Frame bytes;
                                         //   coproc relays verbatim via
                                         //   ws.binaryAll.
constexpr uint8_t PKT_STATUS    = 0x11;  // main→coproc, ~1 Hz, small
                                         //   binary. Subsystem readiness
                                         //   flags + reset reason +
                                         //   anything the status page
                                         //   needs but isn't in
                                         //   telemetry. See diag.cpp.
constexpr uint8_t PKT_PARAMS    = 0x12;  // main→coproc, on-change + boot,
                                         //   JSON body. Output of
                                         //   params::toJson(); cached on
                                         //   coproc and served on WS
                                         //   "getParams" + the status
                                         //   page params section.
constexpr uint8_t PKT_INPUT_EVENT = 0x13; // main→coproc, ad-hoc. 1-byte
                                          //   payload: one of the INPUT_*
                                          //   codes below. Emitted from
                                          //   joystick.cpp on physical
                                          //   button-edge events that the
                                          //   coproc cares about (today:
                                          //   advancing the dot-matrix
                                          //   page on Square).
constexpr uint8_t PKT_WS_CMD    = 0x20;  // coproc→main, ad-hoc. Body is
                                         //   the raw {"type":...} JSON
                                         //   forwarded from a simulator
                                         //   WS frame. cmdrx.cpp parses
                                         //   it with the same ArduinoJson
                                         //   dispatcher that net.cpp
                                         //   used to use.

// Input event codes carried in PKT_INPUT_EVENT (1-byte payload).
constexpr uint8_t INPUT_BTN_SQUARE = 0x01; // PS5 Square button — edge.

// Sanity ceiling on payload length. The params JSON envelope is the
// biggest payload we emit: ~1.4 KB today (~46 ControlParams float
// fields × ~30 chars per "field":value pair, plus the type envelope),
// growing as we add more fields. 2 KB gives ~50% headroom.
//
// Safe to bump further because the buffers that hold this much (in
// uartbus.cpp's s_txBuf / s_rxPayload and the coproc's matching pair)
// are file-scope static — *not* on any FreeRTOS task stack. Bumping
// past several KB would just bloat .bss; no overflow risk.
constexpr size_t MAX_PAYLOAD = 2048;

// Header bytes before the payload + trailing CRC byte after.
constexpr size_t HEADER_SIZE  = 5;  // magic1 + magic2 + type + length (2)
constexpr size_t TRAILER_SIZE = 1;  // crc8
constexpr size_t OVERHEAD     = HEADER_SIZE + TRAILER_SIZE;

// CRC-8 / Dallas-Maxim — same flavour the OneWire family uses, well-
// understood, single-byte output, catches all single-bit errors and
// most burst errors of length <8 in payloads of our size class.
//
// The `seed` parameter allows chaining across non-contiguous spans:
// compute over the header, then continue over the payload buffer,
// without having to coalesce both into a scratch buffer first. The rx
// state machine relies on this so it doesn't need MAX_PAYLOAD bytes of
// stack scratch.
inline uint8_t crc8(const uint8_t* data, size_t len, uint8_t seed = 0) {
  uint8_t crc = seed;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

// Pack one packet into `out`. Caller owns the buffer; we don't allocate.
// out must be at least OVERHEAD + payloadLen bytes. payload may be null
// iff payloadLen == 0. Returns the total frame size on success, or 0
// if payloadLen > MAX_PAYLOAD.
inline size_t packFrame(uint8_t* out,
                        uint8_t type,
                        const uint8_t* payload,
                        size_t payloadLen) {
  if (payloadLen > MAX_PAYLOAD) return 0;
  out[0] = MAGIC1;
  out[1] = MAGIC2;
  out[2] = type;
  out[3] = static_cast<uint8_t>(payloadLen & 0xFF);
  out[4] = static_cast<uint8_t>((payloadLen >> 8) & 0xFF);
  for (size_t i = 0; i < payloadLen; ++i) {
    out[HEADER_SIZE + i] = payload[i];
  }
  out[HEADER_SIZE + payloadLen] = crc8(out, HEADER_SIZE + payloadLen);
  return OVERHEAD + payloadLen;
}

// Validate a complete frame buffer and read out the header fields. The
// caller is responsible for delivering exactly OVERHEAD + (decoded length)
// bytes — this function assumes the rx state machine has already counted
// off the right amount. Returns true iff magic + CRC check out.
//
// Outputs (set only on success):
//   *typeOut       = packet type
//   *payloadOut    = pointer into the input buffer where the payload
//                    starts. NOT a copy; valid only as long as the caller
//                    keeps the buffer alive.
//   *payloadLenOut = payload length (0..MAX_PAYLOAD)
inline bool unpackFrame(const uint8_t* in,
                        size_t inLen,
                        uint8_t* typeOut,
                        const uint8_t** payloadOut,
                        size_t* payloadLenOut) {
  if (inLen < OVERHEAD) return false;
  if (in[0] != MAGIC1 || in[1] != MAGIC2) return false;
  const size_t plen = static_cast<size_t>(in[3]) |
                      (static_cast<size_t>(in[4]) << 8);
  if (plen > MAX_PAYLOAD) return false;
  if (inLen != OVERHEAD + plen) return false;
  if (crc8(in, HEADER_SIZE + plen) != in[HEADER_SIZE + plen]) return false;
  *typeOut       = in[2];
  *payloadOut    = in + HEADER_SIZE;
  *payloadLenOut = plen;
  return true;
}

} // namespace wire
