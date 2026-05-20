// boot_diag.cpp — see boot_diag.h.
//
// The whole module is a one-shot capture of esp_reset_reason() plus a
// label table. Kept deliberately tiny: nothing here may fail (we want
// to be able to log "PANIC" *because* the previous run died), and
// nothing here may pull in heavyweight subsystems (called before
// IMU/motors/net are even up).

#include "boot_diag.h"

#include <Arduino.h>
#include <esp_system.h>

namespace boot_diag {
namespace {

bool g_inited = false;
esp_reset_reason_t g_reason = ESP_RST_UNKNOWN;
uint32_t g_bootMs = 0;

}  // namespace

void init() {
  if (g_inited) return;
  g_inited = true;
  // esp_reset_reason() reads the cause stashed in PMU/RTC by the
  // previous reset event; the value is preserved across all reset
  // types we care about (including BROWNOUT — the RTC domain stays
  // up while the CPU domain browns out, by design).
  g_reason = esp_reset_reason();
  g_bootMs = millis();

  Serial.print(F("boot: reset reason = "));
  Serial.print(static_cast<int>(g_reason));
  Serial.print(F(" ("));
  Serial.print(reasonStr());
  Serial.println(F(")"));
  if (isAlarming()) {
    // Loud, on its own line, so an operator scrolling the serial
    // log after a fall sees it without squinting. The status page
    // will also flag this in red on the next refresh.
    Serial.println(F("boot: WARNING — previous run ended with an "
                     "abnormal reset (brownout / panic / watchdog). "
                     "Investigate before re-arming."));
  }
}

esp_reset_reason_t reason()    { return g_reason; }
int                numericReason() { return static_cast<int>(g_reason); }
uint32_t           bootMs()    { return g_bootMs; }

const char* reasonStr() {
  // Subset of esp_reset_reason_t we explicitly label. The IDF defines
  // a few more (SDIO, USB, JTAG, EFUSE, PWR_GLITCH, CPU_LOCKUP) that
  // either don't apply to this board or aren't present in older IDF
  // versions; the default branch returns "OTHER" and callers should
  // pair the label with numericReason() if they want disambiguation.
  switch (g_reason) {
    case ESP_RST_POWERON:   return "POWERON";    // cold boot
    case ESP_RST_EXT:       return "EXT";        // external reset pin
    case ESP_RST_SW:        return "SW";         // esp_restart()
    case ESP_RST_PANIC:     return "PANIC";      // exception / abort()
    case ESP_RST_INT_WDT:   return "INT_WDT";    // interrupt watchdog
    case ESP_RST_TASK_WDT:  return "TASK_WDT";   // task watchdog
    case ESP_RST_WDT:       return "WDT";        // other watchdog
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";  // wake from deep sleep
    case ESP_RST_BROWNOUT:  return "BROWNOUT";   // VDD dipped under threshold
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    default:                return "OTHER";
  }
}

bool isAlarming() {
  switch (g_reason) {
    case ESP_RST_BROWNOUT:
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return true;
    default:
      return false;
  }
}

}  // namespace boot_diag
