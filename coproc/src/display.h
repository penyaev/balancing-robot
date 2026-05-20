// display.h — MAX7219 8x32 dot-matrix, multi-page.
//
// Four cascaded 8x8 MAX7219 modules on the XIAO ESP32-S3's hardware
// SPI pins. Cycles between three pages:
//
//   Text    — scrolling banner (default "balancebot"); contents
//             swappable at runtime via display::setText().
//   Battery — vBat from the most recent telemetry frame, rendered as
//             a centered "12.4V"; blinks at 2 Hz when the LOW_BAT flag
//             is set on that frame.
//   Eyes    — two animated eyes, blink + random gaze, pure pixel work.
//
// Pages advance on three independent triggers:
//
//   - Auto-cycle timer (default 8 s, settable via setAutoCycleMs).
//   - PS5 Square button — main forwards a PKT_INPUT_EVENT over UART,
//     coproc dispatch calls display::nextPage().
//   - WS command `setDisplayPage` from the simulator UI.

#pragma once

#include <stdint.h>

namespace diag { struct StatusSnapshot; }

namespace display {

enum class PageId : uint8_t {
  Text    = 0,
  Battery = 1,
  Eyes    = 2,
  Count,
};

bool start();
bool isReady();

void setEnabled(bool enabled);
bool isEnabled();

// Replace the scrolling banner content. Copied into an internal
// buffer, so the caller's storage doesn't have to outlive the call.
// Implicitly switches the active page to PageId::Text so the new text
// is visible immediately.
void setText(const char* s);

// Page control. setPage clamps to a valid PageId and resets the
// auto-cycle countdown. nextPage walks through PageId in order and
// wraps.
void setPage(PageId p);
void nextPage();
PageId currentPage();

// Auto-cycle interval in ms; 0 disables auto-cycle (button / WS still
// work). Resets the countdown.
void setAutoCycleMs(uint32_t ms);

// Feed the latest decoded status snapshot (PKT_STATUS @ 1 Hz). We
// cache vBat + lowBat for the battery page. Safe to call from the
// UART rx context — we copy the fields we need by value.
//
// Why status and not telemetry: status is independent of the high-rate
// telemetry stream and is always emitted by main, so the battery page
// keeps refreshing even when the operator has muted telemetry from
// the UI.
void onStatus(const diag::StatusSnapshot& s);

// Drive the active page. Cheap when nothing needs redrawing — safe to
// call every loop() tick.
void loop();

}  // namespace display
