// diag.h — periodic main → coproc status + params publisher.
//
// Two streams of data the coproc needs but that don't fit the high-rate
// telemetry frame:
//
//   1) Subsystem readiness flags (status snapshot). Pushed at 1 Hz.
//      Carries the bits the status page renders in its "subsystems"
//      section that aren't in shared::g (reset reason, imu::isReady,
//      motors::isDriverReadyL/R, ina226::isReady). Tiny binary struct;
//      coproc caches the latest and uses it for both `GET /` and any
//      future status WS broadcast.
//
//   2) ControlParams JSON. Pushed on boot (so coproc has a seed) plus
//      whenever params::current changes (cheap memcmp every 1 Hz tick
//      against a copy of the last sent). Coproc caches the JSON string
//      and serves it on WS "getParams" plus the status-page params dump.
//
// Both share the 1 Hz task that lives on CORE_COMMS at low priority,
// next to other comms-side housekeeping. No real-time deadline.

#pragma once

#include <stdint.h>

#include "motors.h"  // motors::DriverSnapshot

namespace diag {

// Binary status struct emitted at 1 Hz as PKT_STATUS payload. Shared
// verbatim between main and coproc: main packs, coproc unpacks. Layout
// must stay byte-identical on both sides — bump STATUS_LAYOUT_VERSION
// and revisit coproc's renderer if you change anything here.
//
// v2 added the two embedded DriverSnapshots.
// v3 added mainUptimeMs.
// v4 added vBat + lowBat + telemetryEnabled so coproc's battery page
//    and the UI sync don't depend on the 60 Hz telemetry stream.
// v5 replaced the one-off lowBat byte with the full shared::g.flags
//    word + an explicit safetyState byte, so the coproc has the same
//    state surface the telemetry frame carries (FALLEN, LOW_BAT,
//    MOTORS_ENABLED, PS_CONNECTED) AND can distinguish DISARMED from
//    READY (which a flags-only view can't).
// v6 surfaced autoArmEnabled so the coproc's eyes page can pick
//    sleeping eyes when READY+!autoArm without round-tripping the
//    params JSON.
struct __attribute__((packed)) StatusSnapshot {
  uint8_t  layoutVersion;   // currently 6
  uint8_t  imuReady;        // 0 / 1
  uint8_t  driverReadyL;    // 0 / 1
  uint8_t  driverReadyR;    // 0 / 1
  uint8_t  ina226Ready;     // 0 / 1
  uint8_t  telemetryEnabled;// 0 / 1, current state of telemetry::isEnabled().
                            //   Echoed back so the UI can resync if the user
                            //   toggled it before the WS came up.
  uint8_t  safetyState;     // safety::State cast — DISARMED / READY / ARMED /
                            //   FALLEN / LOW_BAT.
  uint8_t  autoArmEnabled;  // 0 / 1, mirror of params::current.autoArmEnabled
                            //   above its 0.5 threshold.
  uint16_t flags;           // shared::g.flags word (FLAG_FALLEN / LOW_BAT /
                            //   MOTORS_ENABLED / PS_CONNECTED bits — same
                            //   semantics as the telemetry frame's flags).
  int16_t  resetReason;     // boot_diag::numericReason() — signed; -1
                            //   if cache miss
  uint16_t reserved1;       // pad to 4-byte alignment for uptime + vBat
  uint32_t mainUptimeMs;    // millis() at status-packet send time on main.
                            //   Coproc subtracts its own millis() from the
                            //   value at receive time to track drift / age.
  float    vBat;            // [V] smoothed (shared::g.vBat). Same source the
                            //   telemetry frame carries — replicated here so
                            //   the coproc's dot-matrix battery page +
                            //   status HTML stay live when the telemetry
                            //   stream is muted.
  motors::DriverSnapshot drvL;  // 32 B, see motors.h
  motors::DriverSnapshot drvR;  // 32 B
};
static_assert(sizeof(StatusSnapshot) == 8 + 2 + 2 + 2 + 4 + 4 + 32 + 32,
              "diag::StatusSnapshot wire layout");

constexpr uint8_t STATUS_LAYOUT_VERSION = 6;

// Idempotent. Spawns diagTask on first call.
void start();

} // namespace diag
