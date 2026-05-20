// joystick.h — PS5 DualSense input via classic Bluetooth on the main ESP32.
//
// After the architecture inversion (BT moved off coproc), the original-
// session pattern returns: a classic-BT library (HamzaYslmn/esp-ps5)
// runs on the main board and fires a callback every HID packet (~250 Hz).
// The callback writes shared::g.targetV / targetTurn and toggles
// FLAG_PS_CONNECTED + dispatches arm/disarm/reset edges.
//
// Why this library (vs rodneybakiskan/ps5-esp32 we used earlier): esp-ps5
// exposes rumble + lightbar + adaptive-trigger output APIs that we want
// to wire up later as feedback channels (rumble on low battery, lightbar
// colour mirrors safety FSM, etc.). Same Bluedroid stack underneath; no
// build-system migration needed.
//
// Stick handling: the same deadband + power-law expo curve we developed
// for the UART-receiver path moves here intact — params::current.stick-
// Deadband and stickExpo apply identically. Mapping: left-stick Y →
// targetV (cart velocity); right-stick X → targetTurn (steering).

#pragma once

namespace joystick {

// Idempotent. Brings up Bluedroid + esp-ps5, registers callbacks, then
// returns. The library spawns its own BT-controller task that fires our
// callbacks at the controller's HID rate.
void start();

} // namespace joystick
