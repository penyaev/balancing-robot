// motors_drv.h — TMC2209 control plane (UART, probe, GCONF watchdog,
// status HTML). Step-pulse generation lives elsewhere (motors.cpp).
//
// Why this is a separate module:
//   The driver chip (TMC2209) and the step-pulse source (LEDC/RMT/...) are
//   independent concerns sharing only a frequency. Keeping them split lets
//   us swap step generators (we replaced FastAccelStepper with LEDC
//   without touching a single TMC register write) and gives the per-driver
//   register dump on the / status page a single source of truth.
//
// Threading: every UART register read/write goes through an internal
// mutex. The driver-drift watchdog task spawned by start() also takes
// that mutex, so callers can interleave with it freely. Direct access
// to TMC2209Stepper is discouraged but available via driverL()/R() —
// hold uartLock() / use UartGuard if you do.

#pragma once

#include <stdint.h>

#include "motors.h"  // motors::DriverSnapshot

class Print;
class TMC2209Stepper;
struct ControlParams;

namespace motors {
namespace drv {

// Bring up Serial2 + mutex, scan the UART for any TMC2209s present
// (diagnostic), probe the two configured addresses, apply runtime
// config (currents/microsteps/chopper/GCONF), and spawn the
// GCONF-drift watchdog. Returns true iff BOTH drivers acked their
// IFCNT-increment probe.
bool start(const ControlParams& cp);

// Per-driver liveness: true iff the chip acked at start().
bool isReadyL();
bool isReadyR();

// Re-apply currents / microsteps / GCONF to both drivers. Idempotent;
// safe to call from any task (takes the UART mutex).
void applyParamsLive(const ControlParams& cp);

// Render a per-driver register dump as <tr> rows for the / status page.
// side='L' or 'R'. Every value is a live UART read (six register reads
// per call). Renders a single "not detected" row if the driver was
// missing at boot.
void printDriverDetailsHtml(::Print& out, char side);

// Live register-state snapshot. Same UART reads printDriverDetailsHtml
// does, but returned as raw values instead of rendered HTML. Intended
// for the diag task (1 Hz) so the coproc can render driver detail on
// its status page without round-tripping. side='L'/'R'. Fills out.valid
// = 0 if the driver wasn't detected at boot.
void readDriverSnapshot(char side, motors::DriverSnapshot& out);

// One-line status summary on a generic stream (boot log, debug cmd).
void printStatus(::Print& out);

// Escape hatches for the rare caller that needs to talk to the chip
// directly (currently none outside this module). Always pair with
// UartGuard or uartLock()/uartUnlock().
TMC2209Stepper& driverL();
TMC2209Stepper& driverR();

void uartLock();
void uartUnlock();

class UartGuard {
 public:
  UartGuard()  { uartLock(); }
  ~UartGuard() { uartUnlock(); }
};

}  // namespace drv
}  // namespace motors
