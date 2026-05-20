// joystick.cpp — see joystick.h. PS5 DualSense via HamzaYslmn/esp-ps5.
//
// Reads the controller's stick + button state from the library's global
// `ps5` instance, applies the deadband + expo response curve we tuned on
// the UART-receiver path, and writes shared::g.targetV / targetTurn.
// Buttons drive safety::request*().
//
// Mapping (same as the UART-receiver era):
//   left stick Y  → targetV   (forward = stick up)
//   right stick X → targetTurn (right turn = stick right)
//   PS button     → safety::requestArm
//   Share button  → safety::requestDisarm
//   Options       → safety::requestReset
//
// Sign note: esp-ps5 documents ly/ry as "push UP = negative" (DualSense's
// raw convention). targetV is positive-forward, so we negate ly.
//
// Threading: the library fires its onPacket callback from the Bluedroid
// task on core 0. Our writes are atomic stores (shared::g) + atomic
// one-shot flag flips (safety::request*) — both designed to be safe
// from any task without locking.

#include "joystick.h"

#include <Arduino.h>
#include <math.h>
#include <ps5Controller.h>

#include "config.h"
#include "params.h"
#include "safety.h"
#include "shared_state.h"
#include "uartbus.h"
#include "wire_proto.h"

namespace joystick {

namespace {

bool s_started = false;

// Last lightbar colour we sent. Compared per tick of the poll task so
// we only push an HID OUTPUT report on actual change (or once on
// reconnect, when we reset s_lightbarHaveLast = false). The sentinel
// is needed because the colour after a reconnect isn't ours — the
// controller resets to its default blue and we want our first state
// reading after pairing to push, even if it matches the previous
// post-disconnect cache.
uint8_t s_lightbarR = 0, s_lightbarG = 0, s_lightbarB = 0;
bool    s_lightbarHaveLast = false;

// Apply the translated-deadband + power-law expo curve to a normalized
// stick value in [-1, +1]. Returns a curved value in [-1, +1]. Same
// function we had in the UART receiver — moved here. db/expo clamped
// defensively against bad setParam writes (negative db → suppress
// nothing; expo <= 0 → invert / explode).
float applyStickCurve(float x, float db, float expo) {
  if (db < 0.0f) db = 0.0f;
  if (db > 0.95f) db = 0.95f;
  if (expo < 0.1f) expo = 0.1f;
  float absx = (x < 0.0f) ? -x : x;
  if (absx <= db) return 0.0f;
  float remapped = (absx - db) / (1.0f - db);
  if (remapped > 1.0f) remapped = 1.0f;
  const float curved = powf(remapped, expo);
  return (x < 0.0f) ? -curved : curved;
}

void onConnect() {
  shared::setFlag(shared::FLAG_PS_CONNECTED, true);
  // The controller resets its lightbar to default blue on every
  // pair-up; flagging "no last colour known" forces the poll task to
  // push our state-driven colour on the next tick.
  s_lightbarHaveLast = false;
  Serial.println(F("joystick: PS5 connected"));
}

void onDisconnect() {
  // Critical safety: zero the targets immediately. If the user wanders
  // off while armed and the controller goes out of range, we don't want
  // the last commanded velocity to persist. Safety FSM stays in its
  // current state — disarm is the operator's call — but the cart stops
  // accelerating in any direction.
  shared::g.targetV.store(0.0f, std::memory_order_relaxed);
  shared::g.targetTurn.store(0.0f, std::memory_order_relaxed);
  shared::setFlag(shared::FLAG_PS_CONNECTED, false);
  Serial.println(F("joystick: PS5 disconnected (targetV/Turn zeroed)"));
}

void onEvent() {
  // DualSense BT report-mode gate. The lib's parser uses fixed byte
  // offsets for the enhanced 0x31 report, but the controller ships in
  // short-report 0x01 mode after pairing and only flips to 0x31 after
  // it receives an output report on the interrupt PSM. The lib's
  // first-pair kick tries once during the channels-up-but-no-input
  // window; on our DS5 firmware the controller races and emits its
  // first 0x01 packet before the kick fires. That sets g_active=true
  // inside the lib, which gates all subsequent kicks off. Stuck in
  // 0x01 forever: lib reads ly from where rx lives, cross.pressed
  // tracks R2, etc.
  //
  // Re-kick from here until we observe an actual 0x31 packet. Inspect
  // ps5.latestPacket directly; HamzaYslmn/esp-ps5 doesn't expose the
  // received report ID otherwise. ps5.send() routes through the same
  // ps5BuildAndSend() the lib uses, but its only gate is g_active —
  // which is already true once we're in onEvent — so it actually fires
  // on the interrupt PSM, unlike isConnected()'s gated version.
  const uint8_t* pkt = ps5.latestPacket;
  const uint8_t  reportId = pkt ? ((pkt[0] == 0xA1) ? pkt[1] : pkt[0]) : 0;
  if (reportId != 0x31) {
    static uint32_t kickAt = 0;
    static uint32_t kickLogAt = 0;
    const uint32_t now = millis();
    if (now - kickAt > 300UL) {
      kickAt = now;
      ps5.send();  // forces ps5BuildAndSend() (output frame on interrupt PSM)
      if (now - kickLogAt > 2000UL) {
        kickLogAt = now;
        Serial.printf("joystick: report 0x%02X, kicking to 0x31...\n",
                      (unsigned)reportId);
      }
    }
    // Skip stick/button processing — bytes are at wrong offsets in 0x01.
    return;
  }

  // Stick values: esp-ps5 fills ps5.lx/ly/rx/ry as int8 in [-128, +127].
  constexpr float SCALE = 127.0f;
  const float nLeftY  = -static_cast<float>(ps5.ly) / SCALE;  // forward = +
  const float nRightX =  static_cast<float>(ps5.rx) / SCALE;  // right   = +

  const float db       = params::current.stickDeadband;
  const float expo     = params::current.stickExpo;
  const float vMaxCart = params::current.vMaxCart;
  const float vMaxTurn = params::current.vMaxTurn;

  float v = applyStickCurve(nLeftY,  db, expo) * vMaxCart;
  float t = applyStickCurve(nRightX, db, expo) * vMaxTurn;
  if (v >  vMaxCart) v =  vMaxCart; else if (v < -vMaxCart) v = -vMaxCart;
  if (t >  vMaxTurn) t =  vMaxTurn; else if (t < -vMaxTurn) t = -vMaxTurn;
  shared::g.targetV.store(v, std::memory_order_relaxed);
  shared::g.targetTurn.store(t, std::memory_order_relaxed);

  // Button edges. esp-ps5's .pressed flag is consume-on-read: returns
  // true once per rising edge, auto-clears. Holding doesn't spam at
  // 250 Hz; only fires on physical button presses.
  //
  // Mapping:
  //   PS    → requestReset + enable auto-arm. RESET clears FALLEN /
  //           LOW_BAT latches into DISARMED so the safety FSM can
  //           pick up. autoArmEnabled=1 makes the safety loop arm
  //           the bot once it sits inside the auto-arm angle window
  //           for the dwell period (params::autoArmHoldMs) — the
  //           operator never has to press an "arm" button.
  //   Share → requestDisarm + disable auto-arm. Without disabling
  //           auto-arm here, a disarm while the bot is balanced
  //           would silently re-arm after autoArmHoldMs and undo the
  //           operator's request.
  // There is no direct-arm button — arming is always automatic.
  if (ps5.ps_btn.pressed) {
    Serial.println(F("joystick: PS edge -> requestReset + autoArm=on"));
    safety::requestReset();
    params::current.autoArmEnabled = 1.0f;
  }
  if (ps5.share.pressed) {
    Serial.println(F("joystick: Share edge -> requestDisarm + autoArm=off"));
    safety::requestDisarm();
    params::current.autoArmEnabled = 0.0f;
  }

  // Square — cycle the coproc's dot-matrix page. Edge is one-shot
  // (consume-on-read), so this fires exactly once per physical press.
  // sendInputEvent emits a 7-byte PKT_INPUT_EVENT frame over the
  // shared UART; coproc's dispatcher calls display::nextPage(). Cost
  // is noise next to the 60 Hz telemetry stream.
  if (ps5.square.pressed) {
    Serial.println(F("joystick: Square edge -> coproc nextPage"));
    uartbus::sendInputEvent(wire::INPUT_BTN_SQUARE);
  }
  // Cross button — unmapped. (Was a random-lightbar sanity check; the
  // lightbar is now state-driven from lightbarPollTask.)
}

} // namespace

// Periodic ps5.isConnected() poll. Per the lib's own comments
// (ps5Controller.cpp:270-289), isConnected() does much more than return
// a flag: it drives auto-reconnect AND sends the output-frame "kick" on
// the interrupt PSM that flips a freshly-paired DualSense from short
// report 0x01 to enhanced 0x31. The lib expects loop() to call it
// every iteration; we don't have a loop(), so we spin a tiny task.
//
// 50 ms is fast enough to catch the brief window between "channels
// open" and "first input report" when the kick needs to fire. Past
// connection, the call costs a few atomic reads.
// Pick the lightbar colour from the current safety state + autoArm
// param. Returns the (r,g,b) tuple via output args so the caller can
// dedupe against the last-sent colour and only fire ps5.send() on
// change.
//
// Mapping:
//   ARMED                                → green         (0,  255,   0)
//   READY + autoArmEnabled               → grayish white (80, 80,   80)
//   FALLEN or LOW_BAT (attention needed) → red           (255, 0,    0)
//   DISARMED or READY-without-autoArm    → yellow        (255, 150,  0)
//   (i.e. bot can't arm yet but isn't faulted)
//
// Red is reserved for "operator needs to do something to recover".
// Yellow is "bot is just waiting" — preconditions not yet met, or
// auto-arm switched off. The dot-matrix face (sleep / dead) carries
// the same distinction in a different channel.
void pickLightbarColor(uint8_t& r, uint8_t& g, uint8_t& b) {
  const safety::State st = safety::state();
  if (st == safety::State::ARMED) {
    r = 0; g = 255; b = 0; return;
  }
  if (st == safety::State::FALLEN || st == safety::State::LOW_BAT) {
    r = 255; g = 0; b = 0; return;
  }
  if (st == safety::State::READY &&
      params::current.autoArmEnabled > 0.5f) {
    r = 80; g = 80; b = 80; return;
  }
  // DISARMED or READY-without-autoArm: yellow.
  r = 255; g = 150; b = 0;
}

void isConnectedPollTask(void*) {
  for (;;) {
    (void)ps5.isConnected();

    // Lightbar refresh. Cheap when nothing changed (just two atomic
    // reads + a few comparisons); pushes one ~12-byte HID OUTPUT
    // report when the state transitions. 50 ms cadence is well above
    // the library's per-send rate-limit advisory and gives sub-frame
    // latency on a state change.
    if (ps5.isConnected()) {
      uint8_t r, g, b;
      pickLightbarColor(r, g, b);
      if (!s_lightbarHaveLast ||
          r != s_lightbarR || g != s_lightbarG || b != s_lightbarB) {
        ps5.lightbar(r, g, b).send();
        s_lightbarR = r; s_lightbarG = g; s_lightbarB = b;
        s_lightbarHaveLast = true;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void start() {
  if (s_started) return;
  s_started = true;

  ps5.attachOnConnect(&onConnect);
  ps5.attachOnDisconnect(&onDisconnect);
  ps5.attach(&onEvent);

  // Spawn the isConnected() poller before begin() so the very first
  // post-channel-open window is covered. Pinned to core 0 with the
  // other comms tasks; non-real-time, 50 ms cadence.
  xTaskCreatePinnedToCore(
      isConnectedPollTask, "ps5-poll", 3072, nullptr,
      /*priority=*/1, nullptr, /*core=*/0);

  // BB_PS5_MAC is a string literal injected from platformio.ini. We use
  // the targeted-pair form so the library skips its scan window and
  // goes straight to L2CAP reconnect — the controller is pre-bonded to
  // this MAC (we did the dance at session start). Empty MAC reverts
  // to discovery mode.
  if (BB_PS5_MAC[0] != '\0') {
    const bool ok = ps5.begin(BB_PS5_MAC);
    Serial.print(F("joystick: ps5.begin(\""));
    Serial.print(BB_PS5_MAC);
    Serial.print(F("\") -> "));
    Serial.println(ok ? F("ok") : F("FAIL"));
  } else {
    const bool ok = ps5.begin();  // scan + first-pair workflow
    Serial.print(F("joystick: ps5.begin() (scan) -> "));
    Serial.println(ok ? F("ok") : F("FAIL"));
  }
}

} // namespace joystick
