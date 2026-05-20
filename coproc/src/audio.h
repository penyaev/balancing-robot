// audio.h — PDM microphone capture + /audio WebSocket broadcast.
//
// Phase C. Brings up the on-board PDM mic on the XIAO ESP32-S3 Sense
// (PDM CLK on GPIO 42, DATA on GPIO 41) via I2S in PDM-RX mode, and
// pumps fixed-size PCM chunks to clients connected on a dedicated
// /audio WebSocket. We isolate audio onto its own WS endpoint so it
// doesn't share backpressure with the 60 Hz telemetry stream on /ws.
//
// Wire format: each binary WebSocket message carries N consecutive
// 16-bit little-endian PCM samples, mono, 16 kHz. Default chunk is
// 512 samples = 1024 bytes = ~32 ms — small enough that browser-side
// jitter buffering can keep up with normal WiFi latency.

#pragma once

#include <ESPAsyncWebServer.h>

namespace audio {

// Configure I2S0 in PDM-RX mode and spawn the reader task. Returns true
// on success; logs + returns false if I2S init fails (no mic plugged
// in, pin conflict, etc.). Safe to call without /audio being set up —
// the task just discards samples until at least one client connects.
bool start();

// True iff start() succeeded. Status-page row.
bool isReady();

// Runtime on/off. Defaults to enabled on boot; webui toggles via the
// setMicEnabled WS command. When disabled the reader task calls
// i2s_stop, which halts the I2S DMA peripheral and removes the bulk
// of the per-second CPU + memory work. When re-enabled, the task
// calls i2s_start and resumes its capture+broadcast loop.
void setEnabled(bool enabled);
bool isEnabled();

// Pointer to the audio WebSocket that the reader task broadcasts on.
// Owned by main.cpp (registered with the AsyncWebServer like /ws), but
// passed in here so the reader task can iterate clients + per-client
// canSend() check. Call once at startup, before start().
void attachWs(AsyncWebSocket* ws);

}  // namespace audio
