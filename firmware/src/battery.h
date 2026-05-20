// battery.h — pack-voltage monitoring (PLAN.md §4 V_sense, §13 risks).
//
// Sources the pack voltage from the INA226's bus reading
// (shared::g.vBus), smooths it with a one-pole IIR, and publishes to
// shared::g.vBat. Also drives the FLAG_LOW_BAT bit with hysteresis
// (VBAT_CUTOFF_V to drop, VBAT_RECOVER_V to clear) so safetyTask can
// react.
//
// Owns its own FreeRTOS task pinned to core 1 (alongside controlTask /
// safetyTask), polling at 50 Hz. The control loop never reads the
// INA226 directly.

#pragma once

namespace battery {

// Spawns the polling task. Idempotent.
void start();

} // namespace battery
