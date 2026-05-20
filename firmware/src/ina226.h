// ina226.h — INA226 high-side voltage + current monitor.
//
// A small standalone driver. Doesn't pull in an external library because
// the INA226 register surface is tiny (config + calibration write, two
// reads) and the existing firmware mixes bespoke drivers (battery.cpp)
// with vendor ones (electroniccats/MPU6050) — we lean bespoke here to
// keep the dependency tree thin.
//
// Init sequence:
//   1. Wait for Wire (started by imu::start() at boot) — we share the
//      MPU6050 I²C bus on GPIO 21 / GPIO 22.
//   2. Write the configuration register (averaging, conversion time,
//      continuous shunt+bus mode).
//   3. Write the calibration register from the shunt-resistance constant
//      in config.h and our chosen Current_LSB.
//   4. Spawn ina226Task on CORE_BATTERY at priority 2 (next to battery.cpp,
//      same loop rate class).
//
// Runtime:
//   - Sampler reads bus voltage and current registers every ~20 ms.
//   - Writes shared::g.vBus and shared::g.iBus (atomics; no lock).
//   - On I²C read error: leaves the previous shared values unchanged
//     and logs once. The telemetry frame just keeps the last-known
//     reading; if the chip disappears entirely you'll see a flat trace.
//
// Sign convention: positive iBus = current flowing through the shunt
// in the direction of the IN+/IN− wiring on your module. If you see
// negative current under nominal load, swap the shunt-input wires.

#pragma once

namespace ina226 {

// Idempotent. Spawns ina226Task on first call.
void start();

// True once the first config+cal write succeeded and the first sample
// has been latched into shared::g.
bool isReady();

} // namespace ina226
