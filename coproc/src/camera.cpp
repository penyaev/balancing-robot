// camera.cpp — see camera.h.
//
// Implementation notes:
//
// * Pin map below is the Seeed-published XIAO ESP32-S3 Sense layout for
//   the on-board OV2640 (parallel DVP). Wired through the bottom-board
//   ribbon; no jumpers, no choices. PCLK on GPIO 13 doesn't conflict
//   with the coproc's UART link (43/44) or with the Phase C PDM mic
//   pins (41/42).
//
// * Pixel format is JPEG: the OV2640 has on-sensor JPEG hardware, so the
//   ESP32 doesn't burn cycles encoding. fb->buf points directly at a
//   ready-to-send JPEG stream.
//
// * Frame buffers live in PSRAM (CAMERA_FB_IN_PSRAM). The XIAO Sense has
//   8 MB; a VGA JPEG at quality 12 is ~20–50 KB per frame, so two buffers
//   (~100 KB) doesn't even register against the budget.
//
// * grab_mode = CAMERA_GRAB_LATEST: when esp_camera_fb_get() is called,
//   it returns the most recent completed frame and drops anything older.
//   The opposite mode (CAMERA_GRAB_WHEN_EMPTY) would queue, which on a
//   slow consumer would back-pressure all the way up to the DMA layer
//   and either stall capture or run us out of buffers. Latest-drop is
//   the right policy for a live preview where freshness > completeness.
//
// * MJPEG framing: each frame is preceded by a small boundary header
//   ("\r\n--bbframe\r\nContent-Type: ...\r\nContent-Length: N\r\n\r\n")
//   and followed by the JPEG bytes. ESPAsyncWebServer's chunked-response
//   filler is called repeatedly with whatever buffer space is currently
//   available in the AsyncTCP send window; we ride that natural cadence
//   and don't enforce a separate framerate cap (camera produces ~15 fps
//   at VGA, TCP/WiFi throttles us if we'd exceed bandwidth).
//
// * Per-request state lives in a heap-allocated StreamState that's
//   captured by std::shared_ptr inside the filler lambda. When the
//   response object is destroyed (client disconnects or stream errors
//   out), the lambda is destroyed too, the shared_ptr ref count drops,
//   and the StreamState destructor returns any held camera_fb_t to the
//   pool — no leaks.

#include "camera.h"

#include <Arduino.h>
#include <esp_camera.h>
#include <memory>
#include <stdio.h>
#include <string.h>

namespace camera {

namespace {

// XIAO ESP32-S3 Sense camera pin map (Seeed-published).
constexpr int PIN_PWDN  = -1;
constexpr int PIN_RESET = -1;
constexpr int PIN_XCLK  = 10;
constexpr int PIN_SIOD  = 40;  // I2C SDA to the OV2640 SCCB
constexpr int PIN_SIOC  = 39;  // I2C SCL
constexpr int PIN_VSYNC = 38;
constexpr int PIN_HREF  = 47;
constexpr int PIN_PCLK  = 13;
constexpr int PIN_Y9    = 48;
constexpr int PIN_Y8    = 11;
constexpr int PIN_Y7    = 12;
constexpr int PIN_Y6    = 14;
constexpr int PIN_Y5    = 16;
constexpr int PIN_Y4    = 18;
constexpr int PIN_Y3    = 17;
constexpr int PIN_Y2    = 15;

// MJPEG boundary string. Has to be globally unique inside a frame so we
// pick something the JPEG payload would never contain. Mirrored in the
// Content-Type header in handleStream().
constexpr const char* BOUNDARY = "bbframe";

// No software FPS cap. AsyncWebServer's chunked-response filler can't
// cleanly say "no data right now": returning 0 ends the stream and
// returning RESPONSE_TRY_AGAIN breaks the library's inflight-credit
// accounting (drains credits without sending data that would get ACK'd
// to replenish them; after a couple of calls the response stalls and
// completes prematurely). vTaskDelay inside the filler would block
// AsyncTCP — the very thing we wanted to avoid. Sensor-level
// throttling via lower xclk would slow the sensor but with
// GRAB_LATEST the filler still re-sends the same frame, so it doesn't
// reduce bandwidth either.
//
// The practical knob for the on-air bandwidth is per-frame size:
// adjust `jpeg_quality` (10 = high quality / large; 63 = low / small)
// and `frame_size` (FRAMESIZE_VGA vs QVGA, etc.) in esp_camera_init
// below. Halving the frame size or doubling the quality number
// roughly halves bandwidth and gives WiFi/AsyncTCP the headroom that
// keeps telemetry healthy.

bool s_ready = false;
// Runtime on/off. Defaults to enabled on boot; webui toggles via the
// setCameraEnabled WS command. Just a flag — sensor + DMA stay alive
// (cheap, no CPU work when nothing reads frames) but the HTTP filler
// short-circuits, which kills the multipart encoding + WiFi tx that is
// the actual cost of the camera stream. Volatile because the flag is
// read from AsyncTCP's task and written from the same task via the
// wsOnMessage path — single-task access in practice, but mark it
// volatile so the compiler doesn't hoist the load across calls.
volatile bool s_enabled = true;

// Per-stream state. One instance per HTTP client.
struct StreamState {
  camera_fb_t* fb = nullptr;
  size_t       bodyPos = 0;       // bytes of fb->buf already sent
  char         header[160];       // boundary + headers for the current frame
  size_t       headerLen = 0;
  size_t       headerPos = 0;
  enum Phase { GRAB, HEAD, BODY };
  Phase        phase = GRAB;

  ~StreamState() {
    // Crucial cleanup: if the client disconnects mid-frame, we still
    // hold a buffer the camera driver needs back in its pool. Without
    // this the FB pool would leak one slot per dropped client and
    // future fb_get() would eventually return null.
    if (fb) esp_camera_fb_return(fb);
  }
};

}  // namespace

bool start() {
  camera_config_t cfg = {};
  cfg.ledc_channel  = LEDC_CHANNEL_0;   // unused by anything else on coproc
  cfg.ledc_timer    = LEDC_TIMER_0;
  cfg.pin_d0        = PIN_Y2;
  cfg.pin_d1        = PIN_Y3;
  cfg.pin_d2        = PIN_Y4;
  cfg.pin_d3        = PIN_Y5;
  cfg.pin_d4        = PIN_Y6;
  cfg.pin_d5        = PIN_Y7;
  cfg.pin_d6        = PIN_Y8;
  cfg.pin_d7        = PIN_Y9;
  cfg.pin_xclk      = PIN_XCLK;
  cfg.pin_pclk      = PIN_PCLK;
  cfg.pin_vsync     = PIN_VSYNC;
  cfg.pin_href      = PIN_HREF;
  cfg.pin_sccb_sda  = PIN_SIOD;
  cfg.pin_sccb_scl  = PIN_SIOC;
  cfg.pin_pwdn      = PIN_PWDN;
  cfg.pin_reset     = PIN_RESET;
  cfg.xclk_freq_hz  = 20000000;  // 20 MHz
  cfg.pixel_format  = PIXFORMAT_JPEG;
  cfg.frame_size    = FRAMESIZE_VGA;        // 640x480
  cfg.jpeg_quality  = 12;                    // 10 = high, 63 = low
  cfg.fb_count      = 2;                     // double buffer (PSRAM)
  cfg.fb_location   = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode     = CAMERA_GRAB_LATEST;

  const esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("camera: init FAILED, err=0x%x\n", err);
    s_ready = false;
    return false;
  }

  // Sanity log: confirm the sensor is the OV2640 we expect.
  sensor_t* s = esp_camera_sensor_get();
  if (s != nullptr) {
    Serial.printf("camera: init OK — sensor PID=0x%02x VER=0x%02x, fb=VGA JPEG q=%d PSRAM=2x\n",
                  (unsigned)s->id.PID, (unsigned)s->id.VER, cfg.jpeg_quality);
  } else {
    Serial.println("camera: init OK but esp_camera_sensor_get() returned null");
  }

  s_ready = true;
  return true;
}

bool isReady() {
  return s_ready;
}

void setEnabled(bool enabled) {
  if (s_enabled == enabled) return;
  s_enabled = enabled;
  Serial.printf("camera: %s by request\n", enabled ? "enabled" : "disabled");
}

bool isEnabled() {
  return s_enabled;
}

void handleStream(AsyncWebServerRequest* req) {
  if (!s_ready) {
    req->send(503, "text/plain", "camera not ready");
    return;
  }
  if (!s_enabled) {
    req->send(503, "text/plain", "camera disabled");
    return;
  }

  // Heap-allocated state shared with the filler lambda. shared_ptr lets
  // the state's destructor run when the lambda is destroyed (client
  // disconnect, error) — returning any held fb to the camera pool.
  auto state = std::make_shared<StreamState>();

  AsyncWebServerResponse* res = req->beginChunkedResponse(
      "multipart/x-mixed-replace; boundary=bbframe",
      [state](uint8_t* buf, size_t maxLen, size_t /*idx*/) mutable -> size_t {
        size_t out = 0;

        // Runtime kill switch. Returning 0 ends the chunked response;
        // browser's <img> sees the connection close and can choose to
        // reconnect when the user flips the toggle back on.
        if (!s_enabled) return 0;

        // Phase 1: grab a new frame if we don't have one. esp_camera_fb_get
        // blocks until the next captured frame is available — typically a
        // few ms. With CAMERA_GRAB_LATEST we always get the freshest one.
        if (state->phase == StreamState::GRAB) {
          state->fb = esp_camera_fb_get();
          if (!state->fb) {
            // Sensor failure mid-stream. Returning 0 ends the chunked
            // response; the browser will see the connection close and
            // can reconnect on its own.
            return 0;
          }
          state->headerLen = snprintf(
              state->header, sizeof(state->header),
              "\r\n--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
              BOUNDARY, (unsigned)state->fb->len);
          state->headerPos = 0;
          state->bodyPos = 0;
          state->phase = StreamState::HEAD;
        }

        // Phase 2: send the boundary + per-frame headers. Small (~70 B)
        // so we usually drain it in a single call.
        if (state->phase == StreamState::HEAD) {
          const size_t rem = state->headerLen - state->headerPos;
          const size_t n = (rem < maxLen) ? rem : maxLen;
          memcpy(buf + out, state->header + state->headerPos, n);
          state->headerPos += n;
          out += n;
          maxLen -= n;
          if (state->headerPos == state->headerLen) {
            state->phase = StreamState::BODY;
          }
        }

        // Phase 3: send the JPEG bytes. A VGA JPEG at q=12 is ~20–50 KB,
        // so we'll need ~15–40 calls of ~1.4 KB each to drain it.
        if (state->phase == StreamState::BODY && maxLen > 0) {
          const size_t rem = state->fb->len - state->bodyPos;
          const size_t n = (rem < maxLen) ? rem : maxLen;
          memcpy(buf + out, state->fb->buf + state->bodyPos, n);
          state->bodyPos += n;
          out += n;
          if (state->bodyPos == state->fb->len) {
            esp_camera_fb_return(state->fb);
            state->fb = nullptr;
            state->phase = StreamState::GRAB;   // next call grabs a fresh frame
          }
        }

        return out;
      });

  // Discourage caches and let the browser keep the connection open
  // indefinitely.
  res->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  res->addHeader("Pragma", "no-cache");
  res->addHeader("Connection", "close");
  req->send(res);
}

}  // namespace camera
