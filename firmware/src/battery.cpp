// battery.cpp — see battery.h.

#include "battery.h"

#include <Arduino.h>

#include "config.h"
#include "ina226.h"
#include "shared_state.h"

namespace battery {

namespace {

constexpr int   POLL_HZ      = 50;
constexpr int   POLL_MS      = 1000 / POLL_HZ;
constexpr float SMOOTH_TAU_S = 1.0f;          // IIR time constant. Long
                                              // enough that brief sags
                                              // from WiFi / camera bursts
                                              // on the shared 5V buck
                                              // rail don't spuriously
                                              // trip FLAG_LOW_BAT.
constexpr int   LOG_PERIOD_MS = 10000;        // periodic heartbeat every 10 s
                                              // (state transitions still log
                                              //  immediately — see edge-
                                              //  trigger below)
constexpr BaseType_t CORE_BATTERY = 1;

// One-pole IIR: y += alpha * (x - y). alpha = dt / (tau + dt).
inline float smooth(float prev, float x, float dt, float tau) {
  const float alpha = (tau > 0.0f) ? (dt / (tau + dt)) : 1.0f;
  return prev + alpha * (x - prev);
}

void batteryTask(void* /*arg*/) {
  // Wait for INA226 to latch its first sample before priming the IIR,
  // so we don't ramp up from 0 V on boot.
  while (!ina226::isReady()) {
    vTaskDelay(pdMS_TO_TICKS(POLL_MS));
  }
  float v = shared::g.vBus.load(std::memory_order_relaxed);
  shared::g.vBat.store(v, std::memory_order_relaxed);

  TickType_t last = xTaskGetTickCount();
  uint32_t lastLogMs = 0;
  uint32_t lowSinceMs = 0; // 0 means "not currently below threshold"
  bool prevLowFlag = false; // for edge-triggered logging on LOW transitions
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(POLL_MS));

    const float vRaw = shared::g.vBus.load(std::memory_order_relaxed);
    const float dt   = POLL_MS * 0.001f;
    v = smooth(v, vRaw, dt, SMOOTH_TAU_S);
    shared::g.vBat.store(v, std::memory_order_relaxed);

    // Low-battery flag with hysteresis + debounce (PLAN.md §8/§13).
    const uint32_t nowMs = millis();
    const bool curFlag = shared::getFlag(shared::FLAG_LOW_BAT);
    if (!curFlag) {
      if (v < VBAT_CUTOFF_V) {
        if (lowSinceMs == 0) lowSinceMs = nowMs;
        if ((nowMs - lowSinceMs) >= (uint32_t)LOWBAT_DEBOUNCE_MS) {
          shared::setFlag(shared::FLAG_LOW_BAT, true);
        }
      } else {
        lowSinceMs = 0;
      }
    } else {
      // Already in low-bat: clear only when above the recover threshold.
      if (v > VBAT_RECOVER_V) {
        shared::setFlag(shared::FLAG_LOW_BAT, false);
        lowSinceMs = 0;
      }
    }

    if (nowMs - lastLogMs >= (uint32_t)LOG_PERIOD_MS) {
      lastLogMs = nowMs;
      Serial.print(F("vBat: "));
      Serial.print(v, 2);
      Serial.print(F(" V (raw "));
      Serial.print(vRaw, 2);
      Serial.print(F(" V)"));
      if (shared::getFlag(shared::FLAG_LOW_BAT)) {
        Serial.print(F(" [LOW]"));
      }
      Serial.println();
    }

    // Edge-triggered: log immediately when LOW asserts or clears so the
    // transition is visible even between heartbeats.
    const bool nowLow = shared::getFlag(shared::FLAG_LOW_BAT);
    if (nowLow != prevLowFlag) {
      Serial.print(F("vBat: "));
      Serial.print(v, 2);
      Serial.println(nowLow ? F(" V — LOW asserted")
                            : F(" V — LOW cleared"));
      prevLowFlag = nowLow;
    }
  }
}

} // namespace

void start() {
  static bool started = false;
  if (started) return;
  started = true;
  xTaskCreatePinnedToCore(
    batteryTask, "battery", 2048, nullptr, 2, nullptr, CORE_BATTERY);
}

} // namespace battery
