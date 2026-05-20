// camera.h — OV2640 init + multipart MJPEG HTTP handler.
//
// Phase B of the coproc roadmap. Brings up the camera on the XIAO ESP32-S3
// Sense expansion board and exposes its native JPEG output via a
// multipart/x-mixed-replace endpoint that any browser renders directly in
// an <img> tag.
//
// Wiring is on-PCB (the expansion-board ribbon connector); pin map is
// the standard Seeed values, locked inside the .cpp. No GPIO conflicts
// with the existing UART link (43/44) or with the Phase C PDM mic
// pins (41/42).

#pragma once

#include <ESPAsyncWebServer.h>

namespace camera {

// Initialise the OV2640 sensor. Safe to call once from setup() AFTER WiFi
// is up (camera init can take ~200 ms and we'd rather get an IP first so
// any failure is visible on the status page). Returns true on success;
// returns false and logs the esp_camera_init error code if anything
// fails — caller can continue without a camera, just no video pane.
bool start();

// True iff start() succeeded. Used by the status page row and by the
// /stream handler to short-circuit with a clean 503 instead of pretending
// to stream nothing.
bool isReady();

// Runtime on/off. Sensor stays initialised either way (cheap; the heavy
// work is the HTTP multipart encoding + WiFi tx, both gated below). When
// disabled, /stream serves 503 and any in-flight chunked response ends
// itself on the next filler call. Toggled from the webui via the
// setCameraEnabled WS command so we can A/B whether the camera stream
// is what's stalling the coproc's loop().
void setEnabled(bool enabled);
bool isEnabled();

// Async HTTP handler for GET /stream. Builds a per-request streamer that
// pumps multipart/x-mixed-replace frames until the client disconnects.
// CAMERA_GRAB_LATEST mode means a slow consumer drops frames silently
// rather than queueing them in PSRAM, so we never run out of buffers.
void handleStream(AsyncWebServerRequest* req);

}  // namespace camera
