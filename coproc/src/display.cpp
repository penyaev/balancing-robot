// display.cpp — see display.h.
//
// MD_Parola owns the SPI link + the underlying MD_MAX72XX framebuffer.
// The text and battery pages use Parola's text rendering (PA_SCROLL_LEFT
// and PA_PRINT respectively); the eyes page draws raw pixels through
// MD_Parola::getGraphicObject(). When switching to the eyes page we
// call displaySuspend(true) so the animator stops touching the buffer
// behind us; switching back arms a Parola effect and clears the
// suspend.
//
// Page advance is driven from loop() against millis(); button presses
// from main (PKT_INPUT_EVENT → nextPage()) and the WS setDisplayPage
// command both bypass the timer by calling setPage()/nextPage(), which
// resets the countdown.

#include "display.h"

#include <Arduino.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>

#include "config.h"   // VBAT_CUTOFF_V — single source of truth on main
#include "diag.h"

namespace display {

namespace {

constexpr MD_MAX72XX::moduleType_t MODULE_TYPE = MD_MAX72XX::FC16_HW;
constexpr uint8_t MAX_DEVICES = 4;       // 4 × 8x8 = 32 cols
constexpr uint8_t CS_PIN      = 8;

constexpr uint8_t  INTENSITY        = 2;     // 0..15
constexpr uint16_t SCROLL_SPEED     = 50;    // ms/col (text page)
constexpr uint16_t SCROLL_PAUSE     = 0;     // ms at end
constexpr uint32_t DEFAULT_CYCLE_MS = 0;     // 0 = no auto-advance.
                                             // Operator advances via the
                                             // Square button or the
                                             // setDisplayPage WS cmd.
                                             // Set via setAutoCycleMs to
                                             // re-enable the timer.
constexpr uint32_t LOWBAT_BLINK_MS  = 250;   // 2 Hz blink

constexpr char   kDefaultText[] = "hi there";
constexpr size_t TEXT_CAP       = 128;

// Maps the smoothed pack voltage to a linear 0–100 % readout. Fine
// for a glance gauge, not fuel-gauge-grade (LiPo discharge isn't
// linear).
//
//   0 %  = VBAT_CUTOFF_V — the firmware's hard low-battery threshold
//          (config.h). Below this main flips FLAG_LOW_BAT and the
//          safety FSM stops the bot, so anything below is unusable
//          charge from the operator's POV — calling it 0 % matches
//          how the bot actually behaves.
//   100 %= V_FULL — 4S LiPo nominal-full.
constexpr float V_EMPTY = VBAT_CUTOFF_V;
constexpr float V_FULL  = 16.8f;

// Below this percentage the battery page shows an empty icon and
// blinks the outline at LOWBAT_BLINK_MS cadence.
constexpr int LOW_PCT = 10;

MD_Parola s_parola(MODULE_TYPE, CS_PIN, MAX_DEVICES);

bool s_ready    = false;
bool s_enabled  = true;
char s_text[TEXT_CAP] = {0};

PageId   s_page          = PageId::Text;
uint32_t s_pageEnteredMs = 0;
uint32_t s_autoCycleMs   = DEFAULT_CYCLE_MS;

// Latest decoded status snapshot (fed from main.cpp::onStatus via
// display::onStatus). Only the bits the pages need; the rest of the
// snapshot is the coproc-side HTML's job.
bool  s_telemValid = false;       // historical name; means "we've seen
                                  //   at least one PKT_STATUS"
float s_vBat       = 0.0f;
bool  s_lowBat     = false;
bool  s_fallen     = false;
// Safety FSM bits the eyes page needs. We mirror just the two we
// care about (ARMED predicate + ready-to-arm predicate) instead of
// retaining the full state — anything more would duplicate logic the
// firmware already does.
bool  s_armed      = false;
bool  s_readyToArm = false; // safety state == READY && autoArmEnabled

// Cast of safety::State on the main side (firmware/src/safety.h).
constexpr uint8_t SAFETY_ARMED = 2;
constexpr uint8_t SAFETY_READY = 1;

// Mirror of the bit positions in shared::FLAG_* (firmware/src/
// shared_state.h). We could include the header — coproc has the
// firmware-src include path — but it pulls in std::atomic globals and
// the rest of shared::g, which we don't need. Bit positions are part
// of the wire contract (they're already serialised verbatim into the
// telemetry frame's flags word), so duplicating them here is safe.
constexpr uint16_t FLAG_FALLEN  = 1u << 0;
constexpr uint16_t FLAG_LOW_BAT = 1u << 1;

// ---------------------------------------------------------------- text

void armScroll() {
  s_parola.displayText(s_text, PA_LEFT, SCROLL_SPEED, SCROLL_PAUSE,
                       PA_SCROLL_LEFT, PA_SCROLL_LEFT);
  s_parola.displayReset();
}

void enterText() {
  s_parola.displaySuspend(false);
  s_parola.displayClear();
  armScroll();
}

void tickText(uint32_t /*now*/) {
  if (s_parola.displayAnimate()) s_parola.displayReset();
}

// ------------------------------------------------------------- battery

// Battery page redraw state. Last-rendered fields gate the work in
// tickBattery so we only repaint on actual change.
enum class IconMode : uint8_t { Fill, Outline, None };

int      s_batLastPct       = -1;
IconMode s_batLastIcon      = IconMode::Fill;
bool     s_batBlinkOn       = true;
uint32_t s_batLastBlinkMs   = 0;
char     s_batLastTextBuf[8]= {0};
// Persistent text buffer for displayText() — the lib only stores the
// pointer, so the storage must outlive the call. We rotate values
// into here.
char     s_batTextBuf[8]    = {0};

// Battery icon geometry. 10 cols wide, 8 rows tall, anchored to the
// visual LEFT of the 32-col panel.
//
// On this FC16_HW chain raw column 0 lands at the visual right edge
// of the panel and raw column 31 at the visual left, so we place the
// icon at the HIGH raw-col indices (22..31). Visually that's cols
// 0..9 from the left.
//
//   visual col   raw col    role
//   --------------------------------------------
//   0            31         left edge      (0xFF)
//   1            30         inner gap      (0x81)
//   2..7         29..24     tick slots     (0x81 base, +0x3C if filled)
//   8            23         right edge     (0xFF)
//   9            22         cap, "stick"   (0x3C)
//
// Bit 0 = top row; ICON_TICK lights rows 2..5 (vertical middle).
constexpr uint16_t ICON_W       = 10;
constexpr uint16_t ICON_RAW_LEFT  = 31;          // visual col 0
constexpr uint16_t ICON_RAW_CAP   = ICON_RAW_LEFT - ICON_W + 1; // 22
constexpr uint16_t ICON_TICK_C0   = ICON_RAW_LEFT - 2;          // raw 29 = visual col 2
constexpr uint16_t ICON_TICK_CN   = 6;           // tick slot count
constexpr uint8_t  ICON_FULL      = 0xFF;
constexpr uint8_t  ICON_FRAME     = 0x81;        // top + bottom edges
constexpr uint8_t  ICON_TICK      = 0x3C;        // rows 2..5

int batteryPercent() {
  if (!s_telemValid) return -1;
  const float frac = (s_vBat - V_EMPTY) / (V_FULL - V_EMPTY);
  if (!isfinite(frac)) return -1;
  if (frac <= 0.0f) return 0;
  if (frac >= 1.0f) return 100;
  return static_cast<int>(frac * 100.0f + 0.5f);
}

void drawBatteryIcon(int pct) {
  // Paints the icon onto raw cols 22..31 (visual cols 0..9). The
  // text-rendering path already zoneClear()'d the whole zone, so we
  // can write outline + fill without worrying about residue.
  MD_MAX72XX* mx = s_parola.getGraphicObject();
  if (mx == nullptr) return;
  // Left edge (visual col 0).
  mx->setColumn(ICON_RAW_LEFT, ICON_FULL);
  // Inner cols (visual cols 1..7) — frame by default.
  for (int v = 1; v <= 7; ++v) {
    mx->setColumn(ICON_RAW_LEFT - v, ICON_FRAME);
  }
  // Right edge of body (visual col 8).
  mx->setColumn(ICON_RAW_LEFT - 8, ICON_FULL);
  // Cap / "stick" (visual col 9), pointing right.
  mx->setColumn(ICON_RAW_CAP, ICON_TICK);
  // Fill ticks proportional to pct, left-to-right visually (cols 2..7).
  if (pct < 0) return;
  int filled = (pct * ICON_TICK_CN + 50) / 100;
  if (filled < 0) filled = 0;
  if (filled > ICON_TICK_CN) filled = ICON_TICK_CN;
  for (int i = 0; i < filled; ++i) {
    // First tick is visual col 2 = raw ICON_TICK_C0 (29), then 28, 27...
    mx->setColumn(ICON_TICK_C0 - static_cast<uint16_t>(i),
                  ICON_FRAME | ICON_TICK);
  }
}

void clearBatteryIcon() {
  MD_MAX72XX* mx = s_parola.getGraphicObject();
  if (mx == nullptr) return;
  for (uint16_t i = 0; i < ICON_W; ++i) {
    mx->setColumn(ICON_RAW_LEFT - i, 0);
  }
}

void formatBatteryPercent(char* out, size_t cap, int pct) {
  if (pct < 0) snprintf(out, cap, "--%%");
  else         snprintf(out, cap, "%d%%", pct);
}

void paintIcon(int pct, IconMode mode) {
  switch (mode) {
    case IconMode::Fill:    drawBatteryIcon(pct); break;
    case IconMode::Outline: drawBatteryIcon(0);   break;  // contour, 0 ticks
    case IconMode::None:    clearBatteryIcon();   break;
  }
}

void drawBatteryText(int pct) {
  // PA_PRINT's in-effect zoneClear()s the entire zone, so we paint
  // the icon AFTER this call to avoid wiping it. PA_NO_EFFECT exit
  // (vs PA_PRINT) is critical — PA_PRINT's exit phase clears the
  // zone again, which used to drop the buffer one frame after the
  // initial paint.
  formatBatteryPercent(s_batTextBuf, sizeof(s_batTextBuf), pct);
  s_parola.displayText(s_batTextBuf, PA_RIGHT, 0, 0, PA_PRINT, PA_NO_EFFECT);
  s_parola.displayReset();
  while (s_parola.displayAnimate()) { /* settle FSM to END */ }
}

void enterBattery() {
  s_parola.displaySuspend(false);
  s_parola.displayClear();
  s_batLastPct        = -2;            // sentinel — force redraw on first tick
  s_batLastIcon       = IconMode::Fill;
  s_batBlinkOn        = true;
  s_batLastBlinkMs    = millis();
  s_batLastTextBuf[0] = '\0';
}

void tickBattery(uint32_t now) {
  // No displayAnimate() — drawBatteryText drives the FSM to END.

  const int pct = batteryPercent();
  const bool lowPct = (pct >= 0) && (pct < LOW_PCT);
  // Low-bat alarm fires on EITHER the firmware-side flag (telemetry's
  // hysteresis-aware reading) or our own percentage threshold —
  // either condition starts the blink.
  const bool lowAlarm = s_lowBat || lowPct;

  if (lowAlarm) {
    if (now - s_batLastBlinkMs >= LOWBAT_BLINK_MS) {
      s_batBlinkOn     = !s_batBlinkOn;
      s_batLastBlinkMs = now;
    }
  } else {
    s_batBlinkOn = true;
  }

  // Healthy:  filled icon at all times, text shows pct.
  // Low/bat:  text stays solid; icon alternates between an outline
  //           (no ticks — pack is empty) and fully blank at 2 Hz.
  const IconMode desiredIcon = lowAlarm
      ? (s_batBlinkOn ? IconMode::Outline : IconMode::None)
      : IconMode::Fill;

  const bool textChanged = (pct != s_batLastPct);
  const bool iconChanged = (desiredIcon != s_batLastIcon);
  if (!textChanged && !iconChanged) return;

  if (textChanged) {
    // Full redraw: text first (zoneClears the buffer, so the icon
    // needs to follow it), then icon.
    drawBatteryText(pct);
    paintIcon(pct, desiredIcon);
  } else {
    // Icon-only update — text stays. The icon's 10 raw cols are the
    // only thing rewritten, so the operator sees a steady percentage
    // while the icon blinks at low battery.
    paintIcon(pct, desiredIcon);
  }

  s_batLastPct  = pct;
  s_batLastIcon = desiredIcon;
}

// ----------------------------------------------------------------- eyes

// Each eye is 8 cols × 8 rows, encoded as 8 column bytes (bit 0 = top
// row). Pupil is the 2x2 dark patch inside the sclera. Three gaze
// variants (left, center, right) shift the pupil columns. A closed
// eye is just a single horizontal bar drawn at row 4.
//
// Layout on the 32-col panel: left eye at cols 4–11, right eye at
// cols 20–27 — 8 cols of margin outside, 8 cols of nose gap. Reads
// as a face from a meter away.
constexpr uint8_t kEyeCenter[8] = {
  0x3E, 0x41, 0x41, 0x4D, 0x4D, 0x41, 0x41, 0x3E,
};
constexpr uint8_t kEyeLeft[8]   = {
  0x3E, 0x41, 0x4D, 0x4D, 0x41, 0x41, 0x41, 0x3E,
};
constexpr uint8_t kEyeRight[8]  = {
  0x3E, 0x41, 0x41, 0x41, 0x4D, 0x4D, 0x41, 0x3E,
};
constexpr uint8_t kEyeClosed[8] = {
  0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00,
};
// Sleeping eye — a shallow smile-like arc, shown when the bot is in
// a state the operator can't directly act on: DISARMED (preconditions
// not met), READY-without-autoArm (won't engage on its own with the
// new joystick mapping), or LOW_BAT (latched until reset). The shape
// reads as a closed eyelid with the eye facing down.
//
//   . . . . . . . .
//   . . . . . . . .
//   . . . . . . . .
//   X . . . . . . X     row 3 — eyelid edges
//   . X X X X X X .     row 4 — bottom of the arc
//   . . . . . . . .
//   . . . . . . . .
//   . . . . . . . .
constexpr uint8_t kEyeSleep[8] = {
  0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x08,
};

// Dead eye — X-shaped, shown on both eyes when the safety FSM has
// latched FALLEN. The pattern is two crossing diagonals fitting
// inside the same 8×8 bounding box as the live eye, so transitions
// in/out don't shift position.
//
//   . . . . . . . .
//   . X . . . . X .
//   . . X . . X . .
//   . . . X X . . .
//   . . . X X . . .
//   . . X . . X . .
//   . X . . . . X .
//   . . . . . . . .
constexpr uint8_t kEyeDead[8] = {
  0x00, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x00,
};

constexpr uint16_t kLeftEyeCol  = 4;
constexpr uint16_t kRightEyeCol = 20;

enum class Gaze : uint8_t { Left, Center, Right };

Gaze     s_eyesGaze        = Gaze::Center;
uint32_t s_eyesNextGazeMs  = 0;
uint32_t s_eyesNextBlinkMs = 0;
uint32_t s_eyesBlinkEndMs  = 0;
bool     s_eyesBlinking    = false;
// What kind of eye we last painted. Used to detect edges between the
// three modes (live / sleep / dead) and force a redraw on a mode
// change even when the underlying gaze / blink state hasn't moved.
enum class EyeMode : uint8_t { Live, Sleep, Dead };
EyeMode  s_eyesLastMode    = EyeMode::Live;

const uint8_t* eyePixels(Gaze g) {
  switch (g) {
    case Gaze::Left:   return kEyeLeft;
    case Gaze::Right:  return kEyeRight;
    case Gaze::Center: default: return kEyeCenter;
  }
}

void drawEyesFrame(const uint8_t* eye) {
  MD_MAX72XX* mx = s_parola.getGraphicObject();
  if (mx == nullptr) return;
  mx->control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  mx->clear();
  for (uint8_t i = 0; i < 8; ++i) {
    mx->setColumn(kLeftEyeCol  + i, eye[i]);
    mx->setColumn(kRightEyeCol + i, eye[i]);
  }
  mx->control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

// Pick the eye sprite mode from the cached safety bits. Priority:
//   FALLEN  → Dead   (latched fault, dead-cross)
//   ARMED   → Live   (active balancing)
//   READY+autoArm → Live  (about to engage; eyes already open)
//   else    → Sleep  (DISARMED, LOW_BAT, READY-without-autoArm — the
//                     bot can't arm without operator input)
EyeMode currentEyeMode() {
  if (s_fallen)                 return EyeMode::Dead;
  if (s_armed || s_readyToArm)  return EyeMode::Live;
  return EyeMode::Sleep;
}

void enterEyes() {
  // Stop Parola's animator so it doesn't overwrite our pixels with the
  // tail end of a previous text/battery effect on the very next tick.
  s_parola.displaySuspend(true);
  const uint32_t now = millis();
  s_eyesGaze        = Gaze::Center;
  s_eyesBlinking    = false;
  s_eyesNextGazeMs  = now + 1500 + (esp_random() % 1500);
  s_eyesNextBlinkMs = now + 4000 + (esp_random() % 3000);
  s_eyesBlinkEndMs  = 0;
  // First draw reflects the current safety mode so switching to the
  // eyes page never flashes the wrong sprite for one frame.
  const EyeMode mode = currentEyeMode();
  switch (mode) {
    case EyeMode::Dead:  drawEyesFrame(kEyeDead);  break;
    case EyeMode::Sleep: drawEyesFrame(kEyeSleep); break;
    case EyeMode::Live:  drawEyesFrame(eyePixels(s_eyesGaze)); break;
  }
  s_eyesLastMode = mode;
}

void tickEyes(uint32_t now) {
  const EyeMode mode = currentEyeMode();

  // Non-live modes override everything: static sprite, no blink, no
  // gaze updates. We still advance the live-eye RNG schedule below so
  // a recovery edge doesn't immediately fire a queued blink.
  if (mode != EyeMode::Live) {
    if (mode != s_eyesLastMode) {
      drawEyesFrame(mode == EyeMode::Dead ? kEyeDead : kEyeSleep);
      s_eyesLastMode = mode;
    }
    s_eyesBlinking    = false;
    s_eyesNextGazeMs  = now + 1500 + (esp_random() % 1500);
    s_eyesNextBlinkMs = now + 4000 + (esp_random() % 3000);
    return;
  }

  // Force a redraw if we just came back from a non-live mode (the
  // panel currently shows the sleep/dead sprite, not whatever live
  // gaze our s_eyes* timers think we're on).
  bool redraw = (s_eyesLastMode != EyeMode::Live);
  s_eyesLastMode = EyeMode::Live;

  if (s_eyesBlinking) {
    if (now >= s_eyesBlinkEndMs) {
      s_eyesBlinking    = false;
      s_eyesNextBlinkMs = now + 4000 + (esp_random() % 3000);
      redraw = true;
    }
  } else if (now >= s_eyesNextBlinkMs) {
    s_eyesBlinking   = true;
    s_eyesBlinkEndMs = now + 120;
    redraw = true;
  }

  if (!s_eyesBlinking && now >= s_eyesNextGazeMs) {
    const uint32_t r = esp_random() % 3;
    Gaze g = (r == 0) ? Gaze::Left : (r == 1) ? Gaze::Center : Gaze::Right;
    if (g != s_eyesGaze) {
      s_eyesGaze = g;
      redraw = true;
    }
    s_eyesNextGazeMs = now + 1500 + (esp_random() % 1500);
  }

  if (redraw) {
    drawEyesFrame(s_eyesBlinking ? kEyeClosed : eyePixels(s_eyesGaze));
  }
}

// ----------------------------------------------------- page dispatch

struct PageVT {
  void (*enter)();
  void (*tick)(uint32_t);
};

constexpr PageVT kPages[] = {
  { enterText,    tickText    },  // PageId::Text
  { enterBattery, tickBattery },  // PageId::Battery
  { enterEyes,    tickEyes    },  // PageId::Eyes
};

void enterPage(PageId p) {
  s_page          = p;
  s_pageEnteredMs = millis();
  if (s_ready && s_enabled) {
    kPages[static_cast<size_t>(p)].enter();
  }
}

}  // namespace

bool start() {
  strncpy(s_text, kDefaultText, TEXT_CAP - 1);
  s_text[TEXT_CAP - 1] = '\0';

  if (!s_parola.begin()) {
    Serial.println("display: MD_Parola begin() failed");
    s_ready = false;
    return false;
  }
  s_parola.setIntensity(INTENSITY);
  s_parola.displayClear();

  s_ready   = true;
  s_enabled = true;
  // Boot to the eyes page. The default text banner ("balancebot") is
  // visible only when the operator deliberately switches to it (via
  // Square / WS / autocycle), and a stale-data battery readout is
  // never the first thing on the panel at power-on.
  enterPage(PageId::Eyes);
  Serial.printf("display: ready (MAX7219 x%u, CS=GPIO%u, text=\"%s\", "
                "cycle=%ums)\n",
                (unsigned)MAX_DEVICES, (unsigned)CS_PIN, s_text,
                (unsigned)s_autoCycleMs);
  return true;
}

bool isReady()   { return s_ready; }
bool isEnabled() { return s_enabled; }

void setEnabled(bool enabled) {
  if (!s_ready || s_enabled == enabled) {
    s_enabled = enabled;
    return;
  }
  s_enabled = enabled;
  if (enabled) {
    s_parola.displayShutdown(false);
    enterPage(s_page);
  } else {
    s_parola.displaySuspend(false);
    s_parola.displayClear();
    s_parola.displayShutdown(true);
  }
}

void setText(const char* s) {
  if (s == nullptr) return;
  strncpy(s_text, s, TEXT_CAP - 1);
  s_text[TEXT_CAP - 1] = '\0';
  // Surface the new text immediately. If we're already on the text
  // page, this re-arms the scroll with the new buffer; if not, it
  // switches pages so the operator sees what they just typed.
  if (s_page == PageId::Text) {
    if (s_ready && s_enabled) armScroll();
  } else {
    setPage(PageId::Text);
  }
}

void setPage(PageId p) {
  if (static_cast<uint8_t>(p) >= static_cast<uint8_t>(PageId::Count)) {
    return;
  }
  enterPage(p);
}

void nextPage() {
  const auto idx = static_cast<uint8_t>(s_page);
  const auto n   = static_cast<uint8_t>(PageId::Count);
  enterPage(static_cast<PageId>((idx + 1u) % n));
}

PageId currentPage() { return s_page; }

void setAutoCycleMs(uint32_t ms) {
  s_autoCycleMs   = ms;
  s_pageEnteredMs = millis();
}

void onStatus(const diag::StatusSnapshot& s) {
  s_vBat       = s.vBat;
  s_lowBat     = (s.flags & FLAG_LOW_BAT) != 0;
  s_fallen     = (s.flags & FLAG_FALLEN)  != 0;
  s_armed      = (s.safetyState == SAFETY_ARMED);
  s_readyToArm = (s.safetyState == SAFETY_READY) && (s.autoArmEnabled != 0);
  s_telemValid = true;
}

void loop() {
  if (!s_ready || !s_enabled) return;
  const uint32_t now = millis();
  if (s_autoCycleMs && (now - s_pageEnteredMs >= s_autoCycleMs)) {
    nextPage();
    return;
  }
  kPages[static_cast<size_t>(s_page)].tick(now);
}

}  // namespace display
