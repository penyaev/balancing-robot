// audio.cpp — see audio.h.
//
// Implementation notes:
//
// * The XIAO ESP32-S3 Sense's PDM mic is wired to GPIO 42 (CLK) and
//   GPIO 41 (DATA). PDM is a 1-bit oversampled bitstream; the ESP32's
//   I2S hardware does the decimation + low-pass filtering for us and
//   delivers 16-bit PCM at the configured sample rate. We pick 16 kHz
//   mono — voice-band, 32 KB/s wire rate, comfortably below the camera
//   stream and trivial against any WiFi link.
//
// * The arduino-esp32 2.0.17 SDK exposes only the LEGACY I2S API
//   (driver/i2s.h). The new I2S API (driver/i2s_pdm.h, ESP-IDF 5.x)
//   isn't available until arduino-esp32 3.x, which we can't move to
//   without ledcWriteTone refactor on main. The legacy API works fine
//   for our purposes and ChangeLog entries for the new API list PDM-RX
//   feature parity with the legacy — moving over later is a search /
//   replace.
//
// * Reader runs as a pinned FreeRTOS task. We MUST NOT do this work in
//   loop() — that task also drains the 460800-baud UART link to main
//   and a 32 ms blocking i2s_read() would skip ~15 telemetry frames
//   per call. Pinning to core 0 keeps it off the Arduino-loop core
//   (core 1) and shares core 0 with AsyncTCP (the WS send path), which
//   keeps the broadcast latency tight.
//
// * Per-client backpressure: same canSend() pattern as the telemetry
//   broadcast in main.cpp. A browser that pauses (tab backgrounded,
//   AudioContext suspended) gets its frames silently dropped rather
//   than backing up the L2CAP queue.

#include "audio.h"

#include <Arduino.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace audio {

namespace {

constexpr i2s_port_t   I2S_PORT      = I2S_NUM_0;
constexpr int          PIN_PDM_CLK   = 42;
constexpr int          PIN_PDM_DATA  = 41;
constexpr uint32_t     SAMPLE_RATE   = 16000;
constexpr size_t       SAMPLES_PER_CHUNK = 512;        // ~32 ms @ 16 kHz
constexpr size_t       CHUNK_BYTES   = SAMPLES_PER_CHUNK * sizeof(int16_t);
constexpr BaseType_t   READER_CORE   = 0;
constexpr UBaseType_t  READER_PRIO   = 5;
constexpr int          READER_STACK  = 4096;

bool           s_ready = false;
AsyncWebSocket* s_ws   = nullptr;
TaskHandle_t   s_task  = nullptr;
// Runtime on/off. The reader task watches this flag and calls
// i2s_start / i2s_stop on transitions so the DMA peripheral itself is
// idle when disabled — not just the broadcast loop. Volatile because
// it's written from AsyncTCP (wsOnMessage) and read from the audio
// reader task on core 0.
volatile bool s_enabled = true;
// What the task believes it has currently configured. Lets the task
// detect transitions and call i2s_start / i2s_stop exactly once per
// transition rather than every tick.
bool          s_running = false;

// File-scope so it doesn't sit on the reader task's stack.
int16_t s_pcmBuf[SAMPLES_PER_CHUNK];

void readerTask(void* /*arg*/) {
  for (;;) {
    // Apply enable/disable transitions on the I2S peripheral. We do
    // this from inside the task (rather than from setEnabled in some
    // other context) so the i2s_start/stop calls don't race with
    // i2s_read. The task is the only caller of the I2S driver besides
    // start()'s one-time install.
    if (s_enabled && !s_running) {
      i2s_start(I2S_PORT);
      s_running = true;
    } else if (!s_enabled && s_running) {
      i2s_stop(I2S_PORT);
      s_running = false;
    }

    if (!s_running) {
      // Disabled: idle the task. 50 ms is responsive enough for the
      // user toggling without spinning the CPU.
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Use a finite timeout (not portMAX_DELAY) so we still poll the
    // enable flag periodically — otherwise a disable command while the
    // task is blocked in i2s_read would have to wait for the next
    // 32 ms chunk to arrive. 50 ms is well past the worst-case
    // chunk-arrival interval and is the same cadence we idle at.
    size_t got = 0;
    const esp_err_t err = i2s_read(I2S_PORT, s_pcmBuf, CHUNK_BYTES,
                                   &got, pdMS_TO_TICKS(50));
    if (err != ESP_OK || got == 0) continue;

    // No clients → nothing to send. We still consume the bytes so the
    // I2S DMA ring doesn't overflow during idle periods.
    if (s_ws == nullptr || s_ws->count() == 0) continue;

    // Per-client send with canSend() backpressure — mirrors the
    // telemetry broadcast pattern. Drops on saturated clients keep
    // the AsyncTCP queue from filling and the warning log from
    // spamming. binary() takes ownership of an internal copy.
    for (auto& c : s_ws->getClients()) {
      if (c.status() == WS_CONNECTED && c.canSend()) {
        c.binary(reinterpret_cast<uint8_t*>(s_pcmBuf), got);
      }
    }
  }
}

}  // namespace

void attachWs(AsyncWebSocket* ws) {
  s_ws = ws;
}

bool start() {
  i2s_config_t cfg = {};
  cfg.mode             = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  cfg.sample_rate      = SAMPLE_RATE;
  cfg.bits_per_sample  = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format   = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count    = 4;
  cfg.dma_buf_len      = SAMPLES_PER_CHUNK;   // in samples per buffer
  cfg.use_apll         = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk       = 0;

  esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("audio: i2s_driver_install failed, err=0x%x\n", err);
    return false;
  }

  i2s_pin_config_t pins = {};
  pins.mck_io_num   = I2S_PIN_NO_CHANGE;
  pins.bck_io_num   = I2S_PIN_NO_CHANGE;
  pins.ws_io_num    = PIN_PDM_CLK;     // PDM clock on the legacy API maps to ws_io_num
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = PIN_PDM_DATA;
  err = i2s_set_pin(I2S_PORT, &pins);
  if (err != ESP_OK) {
    Serial.printf("audio: i2s_set_pin failed, err=0x%x\n", err);
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }

  // Spawn the reader. Pinned to core 0 to avoid contending with the
  // Arduino loopTask (UART drain + HTTP renderer) on core 1.
  const BaseType_t ok = xTaskCreatePinnedToCore(
      readerTask, "audio-pdm", READER_STACK, nullptr,
      READER_PRIO, &s_task, READER_CORE);
  if (ok != pdPASS) {
    Serial.println("audio: xTaskCreatePinnedToCore failed");
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }

  Serial.printf("audio: I2S PDM RX init OK @ %u Hz mono, chunk=%u samples (%u ms)\n",
                (unsigned)SAMPLE_RATE,
                (unsigned)SAMPLES_PER_CHUNK,
                (unsigned)(1000UL * SAMPLES_PER_CHUNK / SAMPLE_RATE));
  s_ready = true;
  return true;
}

bool isReady() {
  return s_ready;
}

void setEnabled(bool enabled) {
  if (s_enabled == enabled) return;
  s_enabled = enabled;
  Serial.printf("audio: %s by request\n", enabled ? "enabled" : "disabled");
  // The actual i2s_start / i2s_stop transition happens inside the
  // reader task on its next iteration (see readerTask), so this call
  // is non-blocking and safe from any context.
}

bool isEnabled() {
  return s_enabled;
}

}  // namespace audio
