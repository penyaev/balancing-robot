// sketch.cpp — WiFi/HTTP/WS relay coproc.
//
// Receives PKT_TELEMETRY / PKT_STATUS / PKT_PARAMS packets from the main
// ESP32 over Serial1 (see wire_proto.h), caches the latest of each, and:
//
//   * Forwards telemetry frames to WebSocket clients via ws.binaryAll
//     (60 Hz, ~116 B/frame — the exact bytes main sends).
//   * Forwards fresh params JSON to WS clients on receipt (broadcast +
//     also served as the reply to a "getParams" WS command).
//   * Renders HTTP `GET /` from the cached telemetry + status + params.
//
// Outbound WS commands (simulator → coproc) get parsed enough to detect
// `getParams` (which we serve from cache) and otherwise forwarded raw
// over UART as PKT_WS_CMD packets. Coproc acks the WS client immediately;
// main applies the command asynchronously.
//
// This file is intentionally monolithic — it's the only meaningful piece
// of code on the coproc. ~600 lines is fine to keep all the relay logic
// in one place for now; if camera/audio land (Phase B/C) we can split.

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_system.h>

#include "telemetry.h"   // brings in telemetry::Frame + FRAME_SIZE
#include "wire_proto.h"
#include "diag.h"        // diag::StatusSnapshot layout (shared with main)
#include "camera.h"      // OV2640 init + /stream MJPEG handler
#include "audio.h"       // PDM mic init + /audio WS broadcaster
#include "display.h"     // MAX7219 8x32 dot-matrix banner

// ----------------------------------------------------------------------
// Build-time configuration
// ----------------------------------------------------------------------

#ifndef BB_WIFI_SSID
#define BB_WIFI_SSID ""
#endif
#ifndef BB_WIFI_PASS
#define BB_WIFI_PASS ""
#endif
#ifndef BB_MDNS_NAME
#define BB_MDNS_NAME "balancebot"
#endif



namespace {

// Serial1 pins on XIAO ESP32-S3 (silkscreen TX/RX pads).
constexpr int8_t UART_TX_PIN = 43;
constexpr int8_t UART_RX_PIN = 44;
constexpr uint32_t UART_BAUD = 460800;

constexpr uint16_t HTTP_PORT = 80;

// ----------------------------------------------------------------------
// Globals (single-thread access via Arduino loopTask)
// ----------------------------------------------------------------------

AsyncWebServer    g_server(HTTP_PORT);
AsyncWebSocket    g_ws("/ws");
// Dedicated audio WebSocket. Lives at /audio so the PCM stream's
// backpressure and disconnect lifecycle are independent of the
// telemetry+commands WS at /ws — a paused tab on the audio side won't
// block telemetry frames, and vice versa.
AsyncWebSocket    g_audioWs("/audio");

// Latest telemetry frame as opaque bytes — exactly what main sent us.
// We hand this verbatim to ws.binaryAll on every receipt, AND decode it
// for the status-page renderer (which needs typed access to fields).
uint8_t  g_telemBuf[wire::MAX_PAYLOAD];
size_t   g_telemLen = 0;
bool     g_telemValid = false;
uint32_t g_telemRxCount = 0;     // packets RX'd from UART (CRC-OK)
uint32_t g_telemLastMs  = 0;

// Loss diagnostics. seq is a u32 monotonic counter inside the telemetry
// frame at offset 4 (see firmware/src/telemetry.cpp). Comparing it to
// the previous one tells us how many packets vanished BETWEEN main's
// transmit and our successful decode — UART rx-buffer overflow if
// loop() stalls, a CRC fault on noise, a one-byte drop. We can't
// distinguish those from each other, but anything non-zero here means
// the UART side is bleeding. ws_sent / ws_drops are the broadcast side:
// per-client increments where canSend() returned true / false. The
// difference between rx and the per-client send rate tells us
// independently whether the WiFi pipe is healthy or the per-client
// queue is backed up.
uint32_t g_telemSeqGaps     = 0;
uint32_t g_telemLastSeq     = 0;
bool     g_telemHasLastSeq  = false;
uint32_t g_telemWsSent      = 0;   // sum across all clients
uint32_t g_telemWsDrops     = 0;
// Raw UART byte counter, incremented for every byte consumed by
// drainUart (regardless of whether it parses into a valid frame).
// Comparing per-second rate to the expected 60 × 121 = 7260 B/s
// distinguishes "bytes aren't arriving on the wire" from "bytes
// arriving but corrupted into CRC-fail frames".
uint32_t g_uartRxBytes      = 0;

// Latest status snapshot — fixed binary layout from diag.h.
diag::StatusSnapshot g_status{};
bool                 g_statusValid = false;

// Latest params dump — the JSON envelope main sent. We forward this byte
// stream verbatim on `getParams` replies and on every fresh receipt
// (broadcast to all connected clients).
char     g_paramsJson[2048];
size_t   g_paramsLen = 0;

// Connection statistics for the status page.
uint32_t g_wsClientCount = 0;

// Coproc's own reset reason (cached at boot; esp_reset_reason() returns
// the same value across calls so we capture it once and use it for the
// status page).
esp_reset_reason_t g_coprocResetReason = ESP_RST_UNKNOWN;

// Map an integer reset-reason code to a short human-readable name. Used
// for both coproc's own (esp_reset_reason_t) and main's (int16_t from
// boot_diag::numericReason, which is a cast of the same enum) — the
// underlying values are esp_reset_reason_t on both chips.
const char* resetReasonStr(int code) {
  switch (code) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    default:                return "OTHER";
  }
}

// Format an uptime in ms as a compact "1h 23m 45s" / "45s" string into
// the caller's buffer. Returns the populated buffer.
const char* formatUptime(uint32_t ms, char* buf, size_t cap) {
  uint32_t s = ms / 1000;
  uint32_t h = s / 3600; s %= 3600;
  uint32_t m = s / 60;   s %= 60;
  if (h > 0) snprintf(buf, cap, "%uh %um %us", (unsigned)h, (unsigned)m, (unsigned)s);
  else if (m > 0) snprintf(buf, cap, "%um %us", (unsigned)m, (unsigned)s);
  else snprintf(buf, cap, "%us", (unsigned)s);
  return buf;
}

// ----------------------------------------------------------------------
// UART RX state machine. Same logic as firmware/src/uartbus.cpp's rxTask
// but driven from loop() — state persists across loop() invocations.
// ----------------------------------------------------------------------

enum RxState {
  RX_WAIT_MAGIC1 = 0,
  RX_WAIT_MAGIC2,
  RX_TYPE,
  RX_LEN_LO,
  RX_LEN_HI,
  RX_PAYLOAD,
  RX_CRC,
};

RxState  rx_state = RX_WAIT_MAGIC1;
uint8_t  rx_type = 0;
uint16_t rx_payloadLen = 0;
uint16_t rx_payloadIdx = 0;
uint8_t  rx_payload[wire::MAX_PAYLOAD];
// CRC computed incrementally over header + payload as bytes arrive,
// so we don't need a contiguous scratch buffer at CRC-check time.
uint8_t  rx_runningCrc = 0;

void onTelemetry(const uint8_t* body, size_t len) {
  if (len > wire::MAX_PAYLOAD) return;
  memcpy(g_telemBuf, body, len);
  g_telemLen   = len;
  g_telemValid = true;
  g_telemRxCount++;
  g_telemLastMs = millis();

  // Track seq jumps on the UART side. Frame layout (firmware/src/
  // telemetry.cpp): u32 magic at off 0, u32 seq at off 4. If seq
  // jumps by more than 1, count the missed packets. Useful for
  // distinguishing main→coproc loss from coproc→browser loss in the
  // heartbeat log.
  if (len >= 8) {
    uint32_t seq;
    memcpy(&seq, body + 4, sizeof(seq));
    if (g_telemHasLastSeq) {
      const uint32_t expected = g_telemLastSeq + 1;
      if (seq != expected) {
        g_telemSeqGaps += (seq - expected);    // u32 wraparound OK
      }
    }
    g_telemLastSeq    = seq;
    g_telemHasLastSeq = true;
  }

  // Per-client send with backpressure check. Telemetry runs at 60 Hz; a
  // slow client (browser hiccup, WiFi retransmit window) can saturate
  // AsyncWebSocket's per-client queue in a few frames. binaryAll() would
  // log "Too many messages queued" warnings repeatedly when that
  // happens; iterating and gating on canSend() drops new frames for
  // saturated clients silently while healthy clients keep streaming.
  // Count both outcomes so the heartbeat can report rates independent
  // of client count.
  for (auto& c : g_ws.getClients()) {
    if (c.status() != WS_CONNECTED) continue;
    if (c.canSend()) {
      c.binary(g_telemBuf, g_telemLen);
      g_telemWsSent++;
    } else {
      g_telemWsDrops++;
    }
  }
}

void onStatus(const uint8_t* body, size_t len) {
  if (len != sizeof(diag::StatusSnapshot)) {
    Serial.printf("coproc: status packet bad size (%u != %u)\n",
                  (unsigned)len, (unsigned)sizeof(diag::StatusSnapshot));
    return;
  }
  diag::StatusSnapshot s;
  memcpy(&s, body, sizeof(s));
  if (s.layoutVersion != diag::STATUS_LAYOUT_VERSION) {
    Serial.printf("coproc: status layoutVersion=%u, expected %u\n",
                  (unsigned)s.layoutVersion,
                  (unsigned)diag::STATUS_LAYOUT_VERSION);
    return;
  }
  g_status = s;
  g_statusValid = true;
  // Feed the dot-matrix battery page from the status snapshot. Status
  // is independent of the 60 Hz telemetry stream — even if the
  // operator has muted telemetry, vBat refreshes here at 1 Hz, which
  // is faster than the battery-page redraw interval anyway.
  display::onStatus(g_status);

  // Heartbeat to WS clients. Status arrives at 1 Hz independent of the
  // telemetry stream; broadcasting a tiny JSON ping on the same path
  // gives the simulator's stale-feed watchdog a signal that doesn't
  // depend on telemetry being enabled. ~30 B/s/client — negligible.
  if (g_ws.count() > 0) {
    char buf[48];
    const int n = snprintf(buf, sizeof(buf),
                           "{\"type\":\"ping\",\"upMs\":%lu}",
                           (unsigned long)s.mainUptimeMs);
    if (n > 0 && static_cast<size_t>(n) < sizeof(buf)) {
      g_ws.textAll(buf, static_cast<size_t>(n));
    }
  }
}

void onParams(const uint8_t* body, size_t len) {
  if (len >= sizeof(g_paramsJson)) {
    Serial.printf("coproc: params JSON too big (%u >= %u), dropping\n",
                  (unsigned)len, (unsigned)sizeof(g_paramsJson));
    return;
  }
  memcpy(g_paramsJson, body, len);
  g_paramsJson[len] = '\0';
  g_paramsLen = len;
  // Broadcast to all clients so any open UI gets the fresh values
  // without waiting for an explicit getParams.
  if (g_ws.count() > 0) {
    g_ws.textAll(g_paramsJson, g_paramsLen);
  }
}

void rxByte(uint8_t b) {
  switch (rx_state) {
    case RX_WAIT_MAGIC1:
      if (b == wire::MAGIC1) {
        rx_runningCrc = wire::crc8(&b, 1);
        rx_state = RX_WAIT_MAGIC2;
      }
      break;
    case RX_WAIT_MAGIC2:
      if (b == wire::MAGIC2) {
        rx_runningCrc = wire::crc8(&b, 1, rx_runningCrc);
        rx_state = RX_TYPE;
      } else if (b == wire::MAGIC1) {
        rx_runningCrc = wire::crc8(&b, 1);
      } else {
        rx_state = RX_WAIT_MAGIC1;
      }
      break;
    case RX_TYPE:
      rx_type = b;
      rx_runningCrc = wire::crc8(&b, 1, rx_runningCrc);
      rx_state = RX_LEN_LO;
      break;
    case RX_LEN_LO:
      rx_payloadLen = b;
      rx_runningCrc = wire::crc8(&b, 1, rx_runningCrc);
      rx_state = RX_LEN_HI;
      break;
    case RX_LEN_HI:
      rx_payloadLen |= (static_cast<uint16_t>(b) << 8);
      rx_runningCrc = wire::crc8(&b, 1, rx_runningCrc);
      if (rx_payloadLen > wire::MAX_PAYLOAD) {
        rx_state = RX_WAIT_MAGIC1;
        break;
      }
      rx_payloadIdx = 0;
      rx_state = (rx_payloadLen == 0) ? RX_CRC : RX_PAYLOAD;
      break;
    case RX_PAYLOAD:
      rx_payload[rx_payloadIdx++] = b;
      rx_runningCrc = wire::crc8(&b, 1, rx_runningCrc);
      if (rx_payloadIdx >= rx_payloadLen) rx_state = RX_CRC;
      break;
    case RX_CRC:
      if (rx_runningCrc == b) {
        switch (rx_type) {
          case wire::PKT_TELEMETRY: onTelemetry(rx_payload, rx_payloadLen); break;
          case wire::PKT_STATUS:    onStatus(rx_payload, rx_payloadLen);    break;
          case wire::PKT_PARAMS:    onParams(rx_payload, rx_payloadLen);    break;
          case wire::PKT_INPUT_EVENT:
            if (rx_payloadLen == 1 && rx_payload[0] == wire::INPUT_BTN_SQUARE) {
              display::nextPage();
            }
            break;
          default: /* unknown / non-inbound type — drop silently */         break;
        }
      }
      rx_state = RX_WAIT_MAGIC1;
      break;
  }
}

void drainUart() {
  while (Serial1.available()) {
    const int b = Serial1.read();
    if (b < 0) break;
    g_uartRxBytes++;
    rxByte(static_cast<uint8_t>(b));
  }
}

// ----------------------------------------------------------------------
// UART TX (commands forwarded from WS to main)
// ----------------------------------------------------------------------

// Static TX scratch — file-scope so we don't put 1 KB on the loopTask
// stack. Coproc has only one TX caller (the WS event task), so no mutex
// needed; AsyncTCP callbacks are serialized on its own task.
uint8_t tx_buf[wire::OVERHEAD + wire::MAX_PAYLOAD];

bool sendWsCmd(const uint8_t* json, size_t len) {
  if (len > wire::MAX_PAYLOAD) return false;
  const size_t framed = wire::packFrame(tx_buf, wire::PKT_WS_CMD, json, len);
  if (framed == 0) return false;
  Serial1.write(tx_buf, framed);
  return true;
}

// ----------------------------------------------------------------------
// WebSocket handlers
// ----------------------------------------------------------------------

void wsSendAck(AsyncWebSocketClient* client, const char* of) {
  if (!client) return;
  JsonDocument doc;
  doc["type"] = "ack";
  doc["of"]   = of;
  String out;
  serializeJson(doc, out);
  client->text(out);
}

void wsSendError(AsyncWebSocketClient* client, const char* msg) {
  if (!client) return;
  JsonDocument doc;
  doc["type"] = "error";
  doc["msg"]  = msg;
  String out;
  serializeJson(doc, out);
  client->text(out);
}

// Reply to a getParams request from the cached JSON. The body in
// g_paramsJson is already the {"type":"params","params":{…}} envelope
// (main built it that way in diag::serializeParamsJson). Send as-is.
void wsReplyParams(AsyncWebSocketClient* client) {
  if (!client) return;
  if (g_paramsLen == 0) {
    wsSendError(client, "params not yet received from main");
    return;
  }
  client->text(g_paramsJson, g_paramsLen);
}

void wsOnMessage(AsyncWebSocketClient* client, const char* data, size_t len) {
  // Peek at the type to decide between "serve locally" (getParams) and
  // "forward to main" (everything else). We parse twice if forwarding,
  // which is cheap at the rate WS commands arrive.
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    wsSendError(client, err.c_str());
    return;
  }
  JsonObjectConst msg = doc.as<JsonObjectConst>();
  const char* type = msg["type"] | static_cast<const char*>(nullptr);
  if (type == nullptr) {
    wsSendError(client, "missing type");
    return;
  }

  if (strcmp(type, "getParams") == 0) {
    wsReplyParams(client);
    return;
  }

  // Coproc-local toggles. Used by the webui to A/B which subsystem (if
  // any) is causing loop() stalls. We handle these here rather than
  // forwarding to main — main doesn't know about the camera/mic.
  if (strcmp(type, "setCameraEnabled") == 0) {
    JsonVariantConst v = msg["enabled"];
    if (!v.is<bool>()) { wsSendError(client, "setCameraEnabled: missing bool enabled"); return; }
    camera::setEnabled(v.as<bool>());
    wsSendAck(client, type);
    return;
  }
  if (strcmp(type, "setMicEnabled") == 0) {
    JsonVariantConst v = msg["enabled"];
    if (!v.is<bool>()) { wsSendError(client, "setMicEnabled: missing bool enabled"); return; }
    audio::setEnabled(v.as<bool>());
    wsSendAck(client, type);
    return;
  }
  if (strcmp(type, "setDisplayText") == 0) {
    JsonVariantConst v = msg["text"];
    if (!v.is<const char*>()) { wsSendError(client, "setDisplayText: missing string text"); return; }
    display::setText(v.as<const char*>());
    wsSendAck(client, type);
    return;
  }
  if (strcmp(type, "setDisplayEnabled") == 0) {
    JsonVariantConst v = msg["enabled"];
    if (!v.is<bool>()) { wsSendError(client, "setDisplayEnabled: missing bool enabled"); return; }
    display::setEnabled(v.as<bool>());
    wsSendAck(client, type);
    return;
  }
  if (strcmp(type, "setDisplayPage") == 0) {
    JsonVariantConst v = msg["page"];
    if (v.is<const char*>()) {
      const char* p = v.as<const char*>();
      if      (strcmp(p, "text")    == 0) display::setPage(display::PageId::Text);
      else if (strcmp(p, "battery") == 0) display::setPage(display::PageId::Battery);
      else if (strcmp(p, "eyes")    == 0) display::setPage(display::PageId::Eyes);
      else if (strcmp(p, "next")    == 0) display::nextPage();
      else { wsSendError(client, "setDisplayPage: unknown page"); return; }
    } else if (v.is<int>()) {
      display::setPage(static_cast<display::PageId>(v.as<int>()));
    }
    JsonVariantConst cyc = msg["autoCycleMs"];
    if (cyc.is<int>()) display::setAutoCycleMs(cyc.as<uint32_t>());
    wsSendAck(client, type);
    return;
  }

  // Forward to main. Coproc acks immediately — the user-visible effect
  // (param change, motor enable, …) will reflect in the next periodic
  // telemetry/params snapshot.
  if (!sendWsCmd(reinterpret_cast<const uint8_t*>(data), len)) {
    wsSendError(client, "uart forward failed (payload too large?)");
    return;
  }
  wsSendAck(client, type);
}

void wsOnEvent(AsyncWebSocket* /*server*/, AsyncWebSocketClient* client,
               AwsEventType evt, void* arg, uint8_t* data, size_t len) {
  switch (evt) {
    case WS_EVT_CONNECT:
      Serial.printf("coproc: ws client #%u connected from %s\n",
                    (unsigned)client->id(),
                    client->remoteIP().toString().c_str());
      // Send the cached params immediately so the freshly-loaded UI
      // doesn't have to round-trip a getParams.
      if (g_paramsLen > 0) {
        client->text(g_paramsJson, g_paramsLen);
      }
      g_wsClientCount = g_ws.count();
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("coproc: ws client #%u disconnected\n",
                    (unsigned)client->id());
      g_wsClientCount = g_ws.count();
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
      if (info != nullptr && info->final && info->index == 0 &&
          info->len == len && info->opcode == WS_TEXT) {
        // Single-frame text message — the only kind the simulator sends.
        wsOnMessage(client, reinterpret_cast<const char*>(data), len);
      }
      break;
    }
    default:
      break;
  }
}

// ----------------------------------------------------------------------
// HTTP status page renderer
//
// Sources:
//   * subsystem readiness: g_status (PKT_STATUS, 1 Hz) + telemetry flags
//   * live state: g_telemBuf decoded as telemetry::Frame
//   * params: cached JSON
//   * TMC2209 driver detail: g_status.drvL/drvR (DriverSnapshot, also
//     1 Hz). Main reads the registers each tick under the existing
//     UartGuard so the bus doesn't fight the driver-watchdog task.
//
// Bit-decode helpers (IOIN / GCONF / DRV_STATUS) are ported from
// motors_drv.cpp. Kept as small free functions here rather than as
// shared headers because the HTML output style is renderer-specific.
// ----------------------------------------------------------------------

void renderIoinBits(AsyncResponseStream* out, uint32_t ioin) {
  out->print((ioin & (1u << 0)) ? F("ENN=1 ")        : F("ENN=0 "));
  out->print((ioin & (1u << 2)) ? F("MS1=1 ")        : F("MS1=0 "));
  out->print((ioin & (1u << 3)) ? F("MS2=1 ")        : F("MS2=0 "));
  out->print((ioin & (1u << 4)) ? F("DIAG=1 ")       : F("DIAG=0 "));
  out->print((ioin & (1u << 6)) ? F("PDN_UART=1 ")   : F("PDN_UART=0 "));
  out->print((ioin & (1u << 7)) ? F("STEP=1 ")       : F("STEP=0 "));
  out->print((ioin & (1u << 8)) ? F("spread_en=1 ") : F("spread_en=0 "));
  out->print((ioin & (1u << 9)) ? F("DIR=1")         : F("DIR=0"));
}

void renderDrvStatusFlags(AsyncResponseStream* out, uint32_t st) {
  bool any = false;
  auto emit = [&](bool cond, const __FlashStringHelper* name,
                  const __FlashStringHelper* cls) {
    if (!cond) return;
    out->print(F("<span class=")); out->print(cls);
    out->print('>'); out->print(name); out->print(F("</span> "));
    any = true;
  };
  emit(st & (1u << 1),  F("ot"),    F("fail"));
  emit(st & (1u << 0),  F("otpw"),  F("warn"));
  emit(st & (1u << 2),  F("s2ga"),  F("fail"));
  emit(st & (1u << 3),  F("s2gb"),  F("fail"));
  emit(st & (1u << 4),  F("s2vsa"), F("fail"));
  emit(st & (1u << 5),  F("s2vsb"), F("fail"));
  emit(st & (1u << 6),  F("ola"),   F("warn"));
  emit(st & (1u << 7),  F("olb"),   F("warn"));
  emit(st & (1u << 11), F("t157"),  F("fail"));
  emit(st & (1u << 10), F("t150"),  F("warn"));
  emit(st & (1u << 9),  F("t143"),  F("warn"));
  emit(st & (1u << 8),  F("t120"),  F("warn"));
  if (!any) out->print(F("<span class=ok>ok</span>"));
}

void renderDriverDetail(AsyncResponseStream* out, char side,
                        const motors::DriverSnapshot& s) {
  out->print(F("<tr><td class=k>driver "));
  out->print(side);
  out->print(F("</td><td>"));
  if (!s.valid) {
    out->print(F("<span class=fail>not detected</span> &mdash; UART ack missing at boot"));
    out->print(F("</td></tr>"));
    return;
  }
  out->print(F("<span class=ok>OK</span>"));
  if (s.reset_recoveries > 0) {
    out->print(F(" <span class=warn>"));
    out->print((unsigned long)s.reset_recoveries);
    out->print(F(" reset"));
    if (s.reset_recoveries != 1) out->print('s');
    out->print(F(" recovered</span>"));
  }
  out->print(F("</td></tr>"));

  // IOIN
  out->printf("<tr><td class=k>&nbsp;&nbsp;IOIN</td><td>0x%08lX (ver=0x%02lX)<br>"
              "<span style='color:#888;font-size:11px'>",
              (unsigned long)s.ioin,
              (unsigned long)((s.ioin >> 24) & 0xFF));
  renderIoinBits(out, s.ioin);
  out->print(F("</span></td></tr>"));

  // IFCNT
  out->printf("<tr><td class=k>&nbsp;&nbsp;IFCNT</td><td>%u "
              "<span style='color:#888;font-size:11px'>(register-write counter)</span>"
              "</td></tr>",
              (unsigned)s.ifcnt);

  // GSTAT
  out->printf("<tr><td class=k>&nbsp;&nbsp;GSTAT</td><td>0x%X ", (unsigned)s.gstat);
  if (s.gstat == 0) {
    out->print(F("<span class=ok>clean</span>"));
  } else {
    if (s.gstat & 0x1) out->print(F("<span class=warn>reset</span> "));
    if (s.gstat & 0x2) out->print(F("<span class=fail>drv_err</span> "));
    if (s.gstat & 0x4) out->print(F("<span class=fail>uv_cp</span>"));
  }
  out->print(F(" <span style='color:#888;font-size:11px'>(latches, cleared on read)</span>"));
  out->print(F("</td></tr>"));

  // GCONF
  out->printf("<tr><td class=k>&nbsp;&nbsp;GCONF</td><td>0x%lX<br>"
              "<span style='color:#888;font-size:11px'>",
              (unsigned long)s.gconf);
  const uint32_t g = s.gconf;
  out->print((g & (1u << 0)) ? F("I_scale_analog=1 ")  : F("I_scale_analog=0 "));
  out->print((g & (1u << 1)) ? F("internal_Rsense=1 ") : F("internal_Rsense=0 "));
  out->print((g & (1u << 2)) ? F("en_spreadCycle=1 ")  : F("en_spreadCycle=0 "));
  out->print((g & (1u << 3)) ? F("shaft=1 ")           : F("shaft=0 "));
  out->print((g & (1u << 4)) ? F("index_otpw=1 ")      : F("index_otpw=0 "));
  out->print((g & (1u << 5)) ? F("index_step=1 ")      : F("index_step=0 "));
  out->print((g & (1u << 6)) ? F("pdn_disable=1 ")     : F("pdn_disable=0 "));
  if (g & (1u << 7)) out->print(F("<span class=ok>mstep_reg_select=1</span> "));
  else               out->print(F("<span class=fail>mstep_reg_select=0 (MRES from MS1/MS2 pins!)</span> "));
  out->print((g & (1u << 8)) ? F("multistep_filt=1 ") : F("multistep_filt=0 "));
  out->print((g & (1u << 9)) ? F("test_mode=1")       : F("test_mode=0"));
  out->print(F("</span></td></tr>"));

  // microsteps
  out->printf("<tr><td class=k>&nbsp;&nbsp;microsteps</td><td>%u "
              "<span style='color:#888;font-size:11px'>(CHOPCONF.MRES)</span></td></tr>",
              (unsigned)s.microsteps);
  // rms_current
  out->printf("<tr><td class=k>&nbsp;&nbsp;rms_current</td><td>%u mA "
              "<span style='color:#888;font-size:11px'>(configured)</span></td></tr>",
              (unsigned)s.rms_current_ma);
  // CS_ACTUAL
  out->printf("<tr><td class=k>&nbsp;&nbsp;CS_ACTUAL</td><td>%u/31 "
              "<span style='color:#888;font-size:11px'>(live 5-bit scaler)</span></td></tr>",
              (unsigned)s.cs_actual);
  // TSTEP
  out->print(F("<tr><td class=k>&nbsp;&nbsp;TSTEP</td><td>"));
  if ((s.tstep & 0xFFFFF) == 0xFFFFF) {
    out->print(F("&infin; <span style='color:#888;font-size:11px'>(stopped)</span>"));
  } else {
    out->printf("%lu <span style='color:#888;font-size:11px'>(int-clock ticks between steps)</span>",
                (unsigned long)(s.tstep & 0xFFFFF));
  }
  out->print(F("</td></tr>"));

  // DRV_STATUS
  out->printf("<tr><td class=k>&nbsp;&nbsp;DRV_STATUS</td><td>0x%08lX ",
              (unsigned long)s.drv_status);
  renderDrvStatusFlags(out, s.drv_status);
  out->print(F("<br><span style='color:#888;font-size:11px'>"));
  out->print((s.drv_status & (1u << 31)) ? F("stst=1 (standstill) ") : F("stst=0 (moving) "));
  out->print((s.drv_status & (1u << 30)) ? F("stealth=1") : F("stealth=0"));
  out->print(F("</span></td></tr>"));
}

// Mirror of firmware/src/safety.h's enum class State. Kept in sync by
// the wire-layout version (STATUS_LAYOUT_VERSION) — if main's enum
// values shift we bump that and update both sides together.
const char* safetyStateName(uint8_t s) {
  switch (s) {
    case 0: return "DISARMED";
    case 1: return "READY";
    case 2: return "ARMED";
    case 3: return "FALLEN";
    case 4: return "LOW_BAT";
    default: return "?";
  }
}

void writeStatusHtml(AsyncResponseStream* out) {
  // Decode telemetry frame if we have one.
  telemetry::Frame f{};
  const bool haveFrame = g_telemValid && g_telemLen == sizeof(f);
  if (haveFrame) memcpy(&f, g_telemBuf, sizeof(f));

  out->print(F("<!doctype html><html><head><meta charset=utf-8>"));
  out->print(F("<title>balancebot status</title>"));
  // CSS: word-break:break-word on td keeps long register-decode bit
  // strings from forcing horizontal scroll. max-width on body caps the
  // overall page width so labels stay readable on wide monitors too.
  out->print(F("<style>"
               "body{font-family:ui-monospace,Menlo,monospace;background:#0b0e14;color:#cbd5e1;margin:1em;max-width:1100px;}"
               "h1,h2,h3{color:#e2e8f0;}"
               "h2{margin-top:1.5em;border-bottom:1px solid #2a2f38;padding-bottom:0.2em;}"
               "h3{margin-top:1em;color:#a3b1c6;}"
               "table{border-collapse:collapse;margin:0.5em 0;width:100%;}"
               "td{padding:2px 12px 2px 0;vertical-align:top;overflow-wrap:anywhere;word-break:break-word;}"
               "td.k{color:#8a93a3;white-space:nowrap;width:1%;}"
               ".ok{color:#7be07b;}"
               ".warn{color:#ffb454;}"
               ".fail{color:#ff6b6b;}"
               "pre{background:#11151c;padding:0.5em;border-radius:4px;overflow:auto;white-space:pre-wrap;word-break:break-word;}"
               "</style></head><body>"));
  out->print(F("<h1>balancebot</h1>"));

  auto badge = [&](bool ok) {
    out->print(ok ? F("<span class=ok>OK</span>") : F("<span class=fail>FAIL</span>"));
  };
  auto unknownIfNoStatus = [&]() {
    out->print(F("<span class=warn>?</span>"));
  };
  char tbuf[32];

  // ========================== MAIN BOARD ============================
  // Data sources: the periodic PKT_STATUS snapshot (subsystems +
  // reset reason + uptime + driver registers) and the 60 Hz telemetry
  // frame (live numerics).
  out->print(F("<h2>main board (ESP32-WROOM-32)</h2>"));
  out->print(F("<h3>subsystems</h3><table>"));

  out->print(F("<tr><td class=k>last reset reason</td><td>"));
  if (g_statusValid) {
    out->printf("%s (%d)",
                resetReasonStr(g_status.resetReason),
                (int)g_status.resetReason);
  } else { unknownIfNoStatus(); }
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>uptime</td><td>"));
  if (g_statusValid) {
    out->print(formatUptime(g_status.mainUptimeMs, tbuf, sizeof(tbuf)));
  } else { unknownIfNoStatus(); }
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>IMU</td><td>"));
  if (g_statusValid) badge(g_status.imuReady); else unknownIfNoStatus();
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>stepper L</td><td>"));
  if (g_statusValid) badge(g_status.driverReadyL); else unknownIfNoStatus();
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>stepper R</td><td>"));
  if (g_statusValid) badge(g_status.driverReadyR); else unknownIfNoStatus();
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>INA226 (I&sup2;C power)</td><td>"));
  if (g_statusValid && g_status.ina226Ready) {
    badge(true);
    if (haveFrame) {
      out->printf(" &mdash; %.2f V, %+.3f A", (double)f.vBus, (double)f.iBus);
    }
  } else if (g_statusValid) {
    badge(false);
    out->print(F(" &mdash; not detected"));
  } else { unknownIfNoStatus(); }
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>safety FSM</td><td>"));
  if (g_statusValid) out->print(safetyStateName(g_status.safetyState));
  else               unknownIfNoStatus();
  out->print(F("</td></tr>"));

  // PS5 connection bit. Sourced from status flags (always available)
  // rather than the telemetry frame, which the operator can mute.
  // Bit ordering matches shared::FLAG_PS_CONNECTED in
  // firmware/src/shared_state.h.
  out->print(F("<tr><td class=k>PS5 controller</td><td>"));
  if (g_statusValid) badge(g_status.flags & (1u << 3));
  else               unknownIfNoStatus();
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>telemetry seq</td><td>"));
  if (haveFrame) out->print((unsigned)f.seq);
  else           unknownIfNoStatus();
  out->print(F("</td></tr>"));

  out->print(F("</table>"));

  // Drivers (TMC2209 register dump from the 1 Hz status snapshot)
  out->print(F("<h3>drivers (TMC2209)</h3><table>"));
  if (g_statusValid) {
    renderDriverDetail(out, 'L', g_status.drvL);
    renderDriverDetail(out, 'R', g_status.drvR);
  } else {
    out->print(F("<tr><td colspan=2><span class=warn>no status snapshot received yet</span></td></tr>"));
  }
  out->print(F("</table>"));

  // Live state
  out->print(F("<h3>live state</h3><table>"));
  if (haveFrame) {
    out->printf("<tr><td class=k>theta</td><td>%+.3f rad</td></tr>",      (double)f.theta);
    out->printf("<tr><td class=k>thetaDot</td><td>%+.3f rad/s</td></tr>", (double)f.thetaDot);
    out->printf("<tr><td class=k>xDot (est)</td><td>%+.3f m/s</td></tr>", (double)f.xDot);
    out->printf("<tr><td class=k>targetV</td><td>%+.3f m/s</td></tr>",    (double)f.targetV);
    out->printf("<tr><td class=k>targetTurn</td><td>%+.3f m/s</td></tr>", (double)f.targetTurn);
    out->printf("<tr><td class=k>vWheelCmd</td><td>%+.3f m/s</td></tr>",  (double)f.vWheelCmd);
  } else {
    out->print(F("<tr><td colspan=2><span class=warn>no telemetry frame yet</span></td></tr>"));
  }
  out->print(F("</table>"));

  // Params dump
  out->print(F("<h3>params (live, from main)</h3>"));
  if (g_paramsLen > 0) {
    out->print(F("<pre>"));
    out->write(reinterpret_cast<const uint8_t*>(g_paramsJson), g_paramsLen);
    out->print(F("</pre>"));
  } else {
    out->print(F("<p><span class=warn>params not yet received from main</span></p>"));
  }

  // ========================== COPROC ================================
  // Data is local to this chip — no UART round-trip needed.
  out->print(F("<h2>coproc (XIAO ESP32-S3 Sense)</h2>"));
  out->print(F("<h3>subsystems</h3><table>"));

  out->print(F("<tr><td class=k>last reset reason</td><td>"));
  out->printf("%s (%d)",
              resetReasonStr(static_cast<int>(g_coprocResetReason)),
              (int)g_coprocResetReason);
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>uptime</td><td>"));
  out->print(formatUptime(millis(), tbuf, sizeof(tbuf)));
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>WiFi</td><td>"));
  if (WiFi.status() == WL_CONNECTED) {
    out->print(F("<span class=ok>connected</span> ip="));
    out->print(WiFi.localIP());
    out->print(F(" rssi="));
    out->print(WiFi.RSSI());
    out->print(F(" dBm"));
  } else {
    out->print(F("<span class=fail>not connected</span>"));
  }
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>WS clients</td><td>"));
  out->print((unsigned long)g_wsClientCount);
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>UART link</td><td>"));
  if (haveFrame) {
    const uint32_t age = millis() - g_telemLastMs;
    out->printf("<span class=ok>alive</span> &mdash; last telemetry %lu ms ago",
                (unsigned long)age);
  } else {
    out->print(F("<span class=fail>no frames yet</span>"));
  }
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>telemetry recv'd</td><td>"));
  out->printf("%lu frames", (unsigned long)g_telemRxCount);
  if (g_telemSeqGaps > 0) {
    out->printf(", <span class=warn>%lu missed</span>",
                (unsigned long)g_telemSeqGaps);
  }
  out->print(F(" <span style='color:#888;font-size:11px'>"
              "(missed = seq jumps in UART rx — most often coproc loop() "
              "stalled and the UART rx-buffer overflowed)</span>"));
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>telemetry broadcast</td><td>"));
  out->printf("sent=%lu, dropped=%lu",
              (unsigned long)g_telemWsSent,
              (unsigned long)g_telemWsDrops);
  out->print(F(" <span style='color:#888;font-size:11px'>"
              "(dropped = canSend() said no — per-client AsyncTCP queue "
              "saturated, usually means WiFi is the bottleneck)</span>"));
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>camera</td><td>"));
  if (camera::isReady()) {
    badge(true);
    out->print(F(" &mdash; <a href=\"/stream\">/stream</a> "));
    if (camera::isEnabled()) {
      out->print(F("<span style='color:#888'>(enabled)</span>"));
    } else {
      out->print(F("<span class=warn>(disabled by webui)</span>"));
    }
  } else {
    badge(false);
    out->print(F(" &mdash; init failed or no sensor"));
  }
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>display (MAX7219)</td><td>"));
  if (display::isReady()) {
    badge(true);
    if (display::isEnabled()) {
      out->print(F(" <span style='color:#888'>(enabled)</span>"));
    } else {
      out->print(F(" <span class=warn>(disabled by webui)</span>"));
    }
  } else {
    badge(false);
    out->print(F(" &mdash; init failed"));
  }
  out->print(F("</td></tr>"));

  out->print(F("<tr><td class=k>microphone</td><td>"));
  if (audio::isReady()) {
    badge(true);
    out->printf(" &mdash; PDM 16 kHz, ws=/audio, clients=%u ",
                (unsigned)g_audioWs.count());
    if (audio::isEnabled()) {
      out->print(F("<span style='color:#888'>(enabled)</span>"));
    } else {
      out->print(F("<span class=warn>(disabled by webui)</span>"));
    }
  } else {
    badge(false);
    out->print(F(" &mdash; I2S init failed"));
  }
  out->print(F("</td></tr>"));

  out->print(F("</table>"));

  out->print(F("</body></html>"));
}

void httpHandleRoot(AsyncWebServerRequest* req) {
  AsyncResponseStream* out = req->beginResponseStream("text/html");
  writeStatusHtml(out);
  req->send(out);
}

} // namespace

// ----------------------------------------------------------------------
// Arduino entry points
// ----------------------------------------------------------------------

void setup() {
  // Capture reset reason as the very first thing — esp_reset_reason()
  // returns the same value across the whole boot but reading early
  // guarantees we don't lose it to a subsequent SW reset / abort.
  g_coprocResetReason = esp_reset_reason();

  // Dot matrix first. The MAX7219 powers up with an indeterminate
  // framebuffer + intensity, so without an immediate init the panel
  // shows random pixels (sometimes fully lit, sometimes blank, etc.)
  // for the whole boot window — up to ~15 s if WiFi has to time out
  // below. Bringing the driver up here clears it within a few ms of
  // power-on, and the text page is then armed on the empty
  // framebuffer so the first visible content is our "balancebot"
  // banner, not garbage.
  display::start();

  Serial.begin(115200);
  // Wait for USB-CDC enumeration so the boot banner isn't lost.
  delay(2000);

  Serial.println();
  Serial.println("===========================================");
  Serial.println("coproc: BOOT — WiFi/HTTP/WS relay (XIAO S3)");
  Serial.printf( "coproc: reset reason = %s (%d)\n",
                 resetReasonStr(static_cast<int>(g_coprocResetReason)),
                 (int)g_coprocResetReason);

  Serial1.setRxBufferSize(2048);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.printf("coproc: Serial1 @ %lu baud (TX=GPIO%d RX=GPIO%d)\n",
                (unsigned long)UART_BAUD,
                (int)UART_TX_PIN, (int)UART_RX_PIN);

  // WiFi. Skip cleanly if credentials are blank (build with empty
  // BB_WIFI_SSID for offline bench bring-up).
  if (BB_WIFI_SSID[0] != '\0') {
    Serial.printf("coproc: WiFi connecting to '%s'\n", BB_WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // best latency for WS traffic
    WiFi.begin(BB_WIFI_SSID, BB_WIFI_PASS);
    // Async-ish wait — we want to advertise mDNS once we have an IP.
    const uint32_t deadline = millis() + 15000;
    while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("coproc: WiFi ip=%s rssi=%d\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
      if (MDNS.begin(BB_MDNS_NAME)) {
        MDNS.addService("http", "tcp", HTTP_PORT);
        Serial.printf("coproc: mDNS http://%s.local/\n", BB_MDNS_NAME);
      } else {
        Serial.println("coproc: mDNS.begin failed");
      }
    } else {
      Serial.println("coproc: WiFi timeout (will keep retrying in background)");
    }
  } else {
    Serial.println("coproc: WiFi disabled (BB_WIFI_SSID empty)");
  }

  // Camera. Brought up AFTER WiFi so any init failure (sensor missing,
  // PSRAM not available, etc.) is visible on the status page rather than
  // bricking the whole boot. start() returns false on failure; we keep
  // going without a camera in that case — the /stream route will then
  // serve a 503 and the status page will report "FAIL".
  camera::start();

  // Audio. Independent of camera — start() spawns a pinned reader task
  // and from then on the I2S DMA is harvested continuously. attachWs
  // wires the reader task to g_audioWs so it knows where to broadcast
  // PCM chunks.
  audio::attachWs(&g_audioWs);
  audio::start();

  // HTTP + WebSocket
  g_ws.onEvent(wsOnEvent);
  g_server.addHandler(&g_ws);
  g_server.addHandler(&g_audioWs);
  g_server.on("/", HTTP_GET, httpHandleRoot);
  g_server.on("/stream", HTTP_GET, camera::handleStream);
  g_server.begin();
  Serial.printf("coproc: HTTP/WS up on port %u (ws=/ws, audio=/audio, mjpeg=/stream)\n",
                (unsigned)HTTP_PORT);

  Serial.println("===========================================");
}

void loop() {
  drainUart();
  display::loop();

  // 1 Hz heartbeat — but rates are computed over the ACTUAL elapsed
  // interval, not assumed 1 s. When loop() stalls (e.g., AsyncTCP
  // contention from a saturated WS broadcast, USB-CDC tx blocked by
  // host backpressure), this interval grows and the rates drop visibly
  // below 60/s. Combined with the seq-gap counter, that pins the loss
  // to the UART rx side vs the broadcast side.
  static uint32_t lastHbMs    = 0;
  static uint32_t lastRxCount = 0;
  static uint32_t lastSeqGaps = 0;
  static uint32_t lastWsSent  = 0;
  static uint32_t lastWsDrops = 0;
  static uint32_t lastUartBytes = 0;
  const uint32_t now = millis();
  if (now - lastHbMs > 1000) {
    const uint32_t dtMs = (lastHbMs == 0) ? 1000 : (now - lastHbMs);
    const uint32_t rxDelta   = g_telemRxCount  - lastRxCount;
    const uint32_t gapDelta  = g_telemSeqGaps  - lastSeqGaps;
    const uint32_t sentDelta = g_telemWsSent   - lastWsSent;
    const uint32_t dropDelta = g_telemWsDrops  - lastWsDrops;
    const uint32_t byteDelta = g_uartRxBytes   - lastUartBytes;
    // Round-half-up rates. dtMs can be tens of seconds after a stall,
    // so this also reveals an "interval was 5000 ms" anomaly.
    auto perSec = [&](uint32_t n) -> unsigned {
      return (unsigned)((static_cast<uint64_t>(n) * 1000 + dtMs / 2) / dtMs);
    };

    // Time each potentially-blocking call inside the heartbeat. If
    // either cleanupClients walks into a lock held by AsyncTCP (which
    // is contended by the camera HTTP stream), this will show ms→s
    // numbers and pinpoint the cause of the loop() stall.
    const uint32_t tBeforeCleanup = millis();
    g_ws.cleanupClients();
    const uint32_t tAfterCleanupWs = millis();
    g_audioWs.cleanupClients();
    const uint32_t tAfterCleanupAudio = millis();

    Serial.printf(
        "coproc: hb wifi=%d ws_clients=%u dt=%lu ms | "
        "uart_rx=%u B/s (total=%lu) tel_rx=%u/s (total=%lu, seq_gaps=%lu) | "
        "ws_sent=%u/s ws_drop=%u/s (total_sent=%lu drop=%lu) | "
        "cleanup_ws=%lu ms cleanup_audio=%lu ms | "
        "params_b=%u status_v=%d | telemetry_dbg=%lu ms, core=%d\n",
        (int)(WiFi.status() == WL_CONNECTED),
        (unsigned)g_ws.count(),
        (unsigned long)dtMs,
        perSec(byteDelta), (unsigned long)g_uartRxBytes,
        perSec(rxDelta),   (unsigned long)g_telemRxCount, (unsigned long)g_telemSeqGaps,
        perSec(sentDelta), perSec(dropDelta),
        (unsigned long)g_telemWsSent, (unsigned long)g_telemWsDrops,
        (unsigned long)(tAfterCleanupWs   - tBeforeCleanup),
        (unsigned long)(tAfterCleanupAudio - tAfterCleanupWs),
        (unsigned)g_paramsLen, (int)g_statusValid);

    (void)gapDelta;   // already shown as the running total

    //Serial.printf("free=%u min=%u\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());

    lastHbMs      = now;
    lastRxCount   = g_telemRxCount;
    lastSeqGaps   = g_telemSeqGaps;
    lastWsSent    = g_telemWsSent;
    lastWsDrops   = g_telemWsDrops;
    lastUartBytes = g_uartRxBytes;
  }

  // Yield. Without this the IDF task watchdog complains. The AsyncTCP
  // task drives its own work; we don't have anything time-critical here.
  vTaskDelay(pdMS_TO_TICKS(2));
}
