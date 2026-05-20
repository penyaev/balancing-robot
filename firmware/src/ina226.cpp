// ina226.cpp — see ina226.h.

#include "ina226.h"

#include <Arduino.h>
#include <Wire.h>
#include <atomic>

#include "config.h"
#include "shared_state.h"

namespace ina226 {

namespace {

constexpr BaseType_t CORE_INA226   = 1;
constexpr int        SAMPLE_PERIOD_MS = 20;   // 50 Hz

// Register map (datasheet Table 6-2).
constexpr uint8_t REG_CONFIG  = 0x00;
constexpr uint8_t REG_BUS     = 0x02;
constexpr uint8_t REG_CURRENT = 0x04;
constexpr uint8_t REG_CAL     = 0x05;

// Configuration register value: 4-sample averaging, 1.1 ms conversion time
// for both bus and shunt, continuous shunt+bus mode. Field layout
// (datasheet 8.6.3.1):
//   bits 14:12 = reserved (read as 100)
//   bits 11:9  = AVG     = 001 → 4 samples averaged
//   bits 8:6   = VBUSCT  = 100 → 1.1 ms bus conversion
//   bits 5:3   = VSHCT   = 100 → 1.1 ms shunt conversion
//   bits 2:0   = MODE    = 111 → shunt+bus, continuous
// Total conversion: 4 × (1.1 + 1.1) ms = 8.8 ms → fresh sample ~113 Hz,
// faster than our 50 Hz polling rate so we never re-read the same value.
constexpr uint16_t CONFIG_VAL = 0x4327;

// Current_LSB = 0.5 mA per LSB. With int16 raw, ±16.384 A full-scale,
// comfortably covering the bot's 1–2 A typical draw plus stepper peaks
// and any future high-current accessories. Calibration register value
// per datasheet 8.6.3.4:
//   CAL = 0.00512 / (Current_LSB × R_shunt)
//       = 0.00512 / (0.0005 × 0.002)
//       = 5120
constexpr float    CURRENT_LSB_A = 0.0005f;
constexpr uint16_t CAL_VAL       = 5120;

// Bus voltage register LSB per datasheet 8.6.3.3.
constexpr float    BUS_LSB_V = 0.00125f;       // 1.25 mV per LSB

std::atomic<bool> g_ready{false};
bool s_started = false;

// I²C primitives. Both return true on success. We re-issue Wire.begin()
// in start() since it's documented as idempotent — protects against the
// (theoretical) case of being called before imu::start().
bool writeReg(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>((val >> 8) & 0xFF));
  Wire.write(static_cast<uint8_t>(val & 0xFF));
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint16_t& out) {
  // I²C bus is shared with the MPU6050, which the IMU task hits at
  // 200 Hz. The arduino-esp32 Wire library serializes individual calls
  // but the no-stop + repeated-start sequence (endTransmission(false)
  // → requestFrom) is two calls; an IMU transaction can squeeze in
  // between and corrupt our read. Retry a few times on failure to
  // absorb that race; only really-gone chips show all attempts failing.
  for (int attempt = 0; attempt < 3; ++attempt) {
    Wire.beginTransmission(INA226_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) == 0) {
      const uint8_t got = Wire.requestFrom(static_cast<uint8_t>(INA226_ADDR),
                                           static_cast<uint8_t>(2));
      if (got == 2) {
        const uint8_t hi = Wire.read();
        const uint8_t lo = Wire.read();
        out = (static_cast<uint16_t>(hi) << 8) | lo;
        return true;
      }
    }
    if (attempt < 2) vTaskDelay(pdMS_TO_TICKS(1));  // brief settle before retry
  }
  return false;
}

void ina226Task(void* /*arg*/) {
  // Wire is brought up by imu::start() at boot; we share the bus. Calling
  // begin() here is idempotent and protects against init-order changes.
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  bool ok = writeReg(REG_CONFIG, CONFIG_VAL);
  ok = writeReg(REG_CAL, CAL_VAL) && ok;
  if (!ok) {
    Serial.println(F("ina226: init write failed (chip absent?)"));
  } else {
    Serial.print(F("ina226: configured ("));
    Serial.print(INA226_SHUNT_OHMS * 1000.0f, 3);
    Serial.print(F(" m\xCE\xA9 shunt, "));
    Serial.print(CURRENT_LSB_A * 1e6f, 0);
    Serial.println(F(" \xC2\xB5""A/LSB)"));
  }

  // Logging policy: with the retries above, a healthy chip should never
  // fail. Some genuine flapping (loose wire, power dip) can still happen
  // and we want visibility — but we don't want a per-sample log line.
  //
  // We log:
  //   * once when a "real" failure streak begins (>= STREAK_FAIL_LOG
  //     consecutive failures, ~1 s at SAMPLE_PERIOD_MS = 20 ms)
  //   * once when reads recover from such a streak
  //   * a periodic "still failing, N failures so far" every LOG_PERIOD_MS
  //     until recovery
  // Single isolated failures absorbed by readReg's retries leave no log
  // entry at all.
  constexpr int      STREAK_FAIL_LOG = 50;     // ~1 s of consecutive fails
  constexpr uint32_t LOG_PERIOD_MS   = 10000;  // re-log "still failing" every 10 s

  TickType_t nextWake     = xTaskGetTickCount();
  bool       firstSample  = true;
  bool       streakLogged = false;
  uint32_t   failStreak   = 0;
  uint32_t   totalFails   = 0;
  uint32_t   lastLogMs    = 0;

  for (;;) {
    vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

    uint16_t busRaw  = 0;
    uint16_t curRaw  = 0;
    const bool gotBus = readReg(REG_BUS,     busRaw);
    const bool gotCur = readReg(REG_CURRENT, curRaw);

    if (gotBus && gotCur) {
      // Bus voltage is unsigned 16-bit; current is signed 16-bit (regen
      // current shows as negative).
      const float vBus = static_cast<float>(busRaw) * BUS_LSB_V;
      const float iBus =
          static_cast<float>(static_cast<int16_t>(curRaw)) * CURRENT_LSB_A;
      shared::g.vBus.store(vBus, std::memory_order_relaxed);
      shared::g.iBus.store(iBus, std::memory_order_relaxed);
      if (firstSample) {
        firstSample = false;
        g_ready.store(true);
      }
      if (streakLogged) {
        Serial.print(F("ina226: I²C recovered after "));
        Serial.print((unsigned long)failStreak);
        Serial.println(F(" failed samples"));
        streakLogged = false;
      }
      failStreak = 0;
    } else {
      failStreak++;
      totalFails++;
      const uint32_t nowMs = millis();
      if (!streakLogged && failStreak >= STREAK_FAIL_LOG) {
        Serial.print(F("ina226: I²C failing persistently ("));
        Serial.print((unsigned long)failStreak);
        Serial.println(F(" consecutive samples)"));
        streakLogged = true;
        lastLogMs    = nowMs;
      } else if (streakLogged && (nowMs - lastLogMs) >= LOG_PERIOD_MS) {
        Serial.print(F("ina226: still failing, "));
        Serial.print((unsigned long)totalFails);
        Serial.println(F(" failures so far"));
        lastLogMs = nowMs;
      }
    }
  }
}

} // namespace

void start() {
  if (s_started) return;
  s_started = true;
  // CORE_BATTERY = 1; share that priority class with battery.cpp since
  // both are slow-poll sensor tasks with no real-time deadline.
  xTaskCreatePinnedToCore(
      ina226Task, "ina226", 3072, nullptr,
      /*priority=*/2, nullptr, CORE_INA226);
}

bool isReady() { return g_ready.load(); }

} // namespace ina226
