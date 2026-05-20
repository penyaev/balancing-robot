// boot_diag.h — capture and surface the cause of the most recent ESP32
// reset. Filled in once at the very top of setup() and never modified
// afterwards, so any subsystem (banner, status page, telemetry…) can
// query it without coordination.
//
// Why this matters: the bot occasionally exhibits a "sudden buzz +
// fall + restart" failure mode with no preceding telemetry signature.
// Without knowing whether the chip rebooted because of a brownout
// (electrical / power-supply issue) or a panic / task watchdog
// (firmware crash / stuck task), we can't begin to bisect the cause.
// esp_reset_reason() is the single most decisive piece of evidence
// and it costs ~0 to capture.
//
// Usage: call boot_diag::init() once, as early in setup() as possible
// (after Serial.begin so the log line is visible). After that, reason()
// / reasonStr() / isAlarming() / numericReason() return the cached
// value forever.

#pragma once

#include <esp_system.h>
#include <stdint.h>

namespace boot_diag {

// Capture esp_reset_reason() and Serial-log it. Idempotent — second
// and subsequent calls are no-ops. The captured value reflects WHY
// the chip is currently coming up; on the very first boot after a
// power cycle that's ESP_RST_POWERON, on the boot that follows a
// brownout it's ESP_RST_BROWNOUT, etc. There is no separate
// "previous boot's reason" — esp_reset_reason() at boot N already
// answers "why did boot N start", which is what we want to show.
void init();

// Cached reset-reason enum from the last init() call. Defaults to
// ESP_RST_UNKNOWN before init() runs (shouldn't happen in practice
// since init() is called at the top of setup()).
esp_reset_reason_t reason();

// Numeric reset-reason value (== reason() cast to int). Useful when
// the IDF version on this build defines a reason code that our
// switch in reasonStr() doesn't have a label for — callers can
// log the number alongside reasonStr()'s "OTHER" fallback.
int numericReason();

// Human label: "POWERON", "BROWNOUT", "PANIC", "TASK_WDT", etc.
// Returns "OTHER" for IDF-defined reasons we don't explicitly
// label (e.g. SDIO/USB/JTAG resets that don't apply to this board).
const char* reasonStr();

// True if the reason indicates an unexpected / unhealthy reset that
// the operator should investigate. Specifically: BROWNOUT, PANIC,
// INT_WDT, TASK_WDT, generic WDT. POWERON / SW (esp_restart) /
// EXT (chip reset pin) / DEEPSLEEP are considered normal.
bool isAlarming();

// millis() value captured at init() time. Mostly 0–few ms; exposed
// so the status page can show "first observed N ms after boot" if
// we ever want it. Cheap and harmless to keep around.
uint32_t bootMs();

}  // namespace boot_diag
