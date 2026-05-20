// motors_drv.cpp — TMC2209 control plane (UART, probe, watchdog,
// status HTML). See motors_drv.h for rationale.

#include "motors_drv.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "config.h"
#include "params.h"

namespace motors {
namespace drv {
namespace {

// One HardwareSerial for the shared driver UART bus. UART2 default pins on
// the ESP32 are GPIO 16/17 — exactly what the pin map uses.
HardwareSerial& kDrvUart = Serial2;

TMC2209Stepper drvL{&kDrvUart, DRV_RSENSE_OHM, DRV_ADDR_L};
TMC2209Stepper drvR{&kDrvUart, DRV_RSENSE_OHM, DRV_ADDR_R};

// Per-driver UART probe results, captured at start() and exposed via
// isReadyL/R().
bool readyL = false;
bool readyR = false;

// Mutex serialising access to the shared half-duplex driver UART bus.
// Three contexts may issue register reads/writes: bring-up code in
// start(), the HTTP status-page handler (printDriverDetailsHtml), and
// the watchdog task below. Without a lock they would interleave bytes
// on the wire and corrupt every read.
SemaphoreHandle_t kUartMutex = nullptr;

// Reset-recovery counters. Incremented by the watchdog every time it
// observes a driver whose GCONF no longer matches our configured value
// (i.e. the chip browned out / RESET-latched and lost its register
// state). Surfaced on the / status page so a power-supply problem shows
// up as a steadily climbing number rather than a silent slowness.
volatile uint32_t resetRecoveryL = 0;
volatile uint32_t resetRecoveryR = 0;

// Expected GCONF value after applyDriverConfig() runs successfully.
// Derivation:
//   bit 0 I_scale_analog   = 0 (we use the on-board sense resistor)
//   bit 1 internal_Rsense  = 0 (external Rsense)
//   bit 2 en_spreadCycle   = MOTOR_USE_SPREADCYCLE (config.h)
//   bit 3 shaft            = 0
//   bit 4 index_otpw       = 0
//   bit 5 index_step       = 0
//   bit 6 pdn_disable      = 1 (UART works while PDN strap is whatever)
//   bit 7 mstep_reg_select = 1 (microsteps from CHOPCONF.MRES, not pins)
//   bit 8 multistep_filt   = 1 (default; filters STEP edges)
//   bit 9 test_mode        = 0
// → 0x1C0 with stealthChop, 0x1C4 with spreadCycle. The chip's power-on
// default is 0x101, which is what we will see if the driver browned
// out and was never reconfigured.
constexpr uint32_t EXPECTED_GCONF =
    0x1C0u | (MOTOR_USE_SPREADCYCLE ? (1u << 2) : 0u);

// Apply the full driver configuration: GCONF bits, currents, microsteps,
// chopper, stealthChop. Idempotent — safe to call repeatedly. Does NOT
// call d.begin() and does NOT do the IFCNT-ack probe; those belong in
// probeDriver(). Caller must hold kUartMutex.
//
// We explicitly write every GCONF bit we care about (especially
// pdn_disable, mstep_reg_select, multistep_filt) so that this function
// can also be used by the watchdog to recover a driver that browned
// out and reverted to the chip's power-on defaults (GCONF=0x101). If
// we relied on the library's begin() to set those bits, watchdog
// recovery would silently leave the driver in MS1/MS2-pin microstep
// mode after a brownout.
void applyDriverConfig(TMC2209Stepper& d, const ControlParams& cp) {
  // GCONF bits — see EXPECTED_GCONF for the target value and rationale.
  d.I_scale_analog(false);
  d.internal_Rsense(false);
  d.en_spreadCycle(MOTOR_USE_SPREADCYCLE);  // see config.h
  d.pdn_disable(true);         // UART takes priority over PDN strap
  d.mstep_reg_select(true);    // microsteps from CHOPCONF.MRES, not pins
  d.multistep_filt(true);

  // Chopper: toff > 0 enables it. 5 is the TMCStepper-recommended default.
  d.toff(5);

  // Microsteps via CHOPCONF.MRES. With mstep_reg_select=1 above, this
  // is the value the chip actually uses (MS1/MS2 pins are then free to
  // serve as UART addresses, which is exactly our wiring).
  uint16_t ms = cp.microsteps;
  if (ms != 8 && ms != 16 && ms != 32 && ms != 64 && ms != 128 && ms != 256) {
    ms = 16;
  }
  d.microsteps(ms);

  // Currents (mA in the TMCStepper API). holdCurrent is honoured as the
  // user-facing param: rms_current's second argument is IHOLD/IRUN ratio.
  // The chip drops from IRUN to IHOLD ~437 ms (default TPOWERDOWN=20)
  // after the last STEP edge. With the velocity deadband + zero-output
  // path, "at rest" means no STEP edges at all, so the chip will sit at
  // IHOLD between commands — set holdCurrent close to runCurrent in the
  // UI if you want full standstill torque.
  const uint16_t runMa  = static_cast<uint16_t>(cp.runCurrent  * 1000.0f);
  const uint16_t holdMa = static_cast<uint16_t>(cp.holdCurrent * 1000.0f);
  const float holdRatio = (runMa > 0) ?
      static_cast<float>(holdMa) / static_cast<float>(runMa) : 0.5f;
  d.rms_current(runMa, holdRatio);

  d.pwm_autoscale(true);
}

// Probe a single TMC2209 for liveness on the shared UART, then apply the
// configuration. Returns true iff the driver acked the IFCNT-increment
// test. Caller must hold kUartMutex.
//
// The IFCNT-ack probe is the only test that actually requires a TMC2209
// to drive the bus — reading GCONF alone is not enough, because with
// nothing connected the UART RX floats and returns whatever idle pattern
// happens to be on the line, which can happily look like a plausible
// register value.
bool probeDriver(TMC2209Stepper& d, const ControlParams& cp,
                 const __FlashStringHelper* tag) {
  d.begin();

  // --- Raw bus diagnostics (logged before any config writes) -------------
  // IOIN's top byte (bits 31:24) is the TMC2209 silicon VERSION and must
  // read 0x21 — a deterministic signature that the addressed driver is
  // actually answering.
  const uint32_t ioin0   = d.IOIN();
  const uint8_t  ifcnt0  = d.IFCNT();
  const uint32_t gconf0  = d.GCONF();
  Serial.print(F("motors: probe ")); Serial.print(tag);
  Serial.print(F(" IOIN=0x"));   Serial.print(ioin0,  HEX);
  Serial.print(F(" (ver=0x"));   Serial.print((ioin0 >> 24) & 0xFF, HEX);
  Serial.print(F(") IFCNT="));   Serial.print(ifcnt0);
  Serial.print(F(" GCONF=0x"));  Serial.println(gconf0, HEX);

  applyDriverConfig(d, cp);

  // Liveness probe — see function comment. Any single register write
  // (GCONF here, harmless: we re-write the value we just configured)
  // must bump IFCNT by 1. With no driver attached, IFCNT reads will be
  // bus-floating noise and almost never satisfy the strict +1 check.
  const uint8_t before = d.IFCNT();
  d.GCONF(d.GCONF()); // round-trip write of current config
  const uint8_t after  = d.IFCNT();
  const bool ok = static_cast<uint8_t>(after - before) == 1;
  Serial.print(F("motors: probe ")); Serial.print(tag);
  Serial.print(F(" IFCNT "));   Serial.print(before);
  Serial.print(F(" -> "));      Serial.print(after);
  Serial.print(F(" ("));        Serial.print(ok ? F("ack") : F("NO ACK"));
  Serial.println(F(")"));
  return ok;
}

// ---------------------------------------------------------------------------
// Driver watchdog task — recovery from brownout / chip reset.
//
// During on-bench bring-up we observed driver R intermittently reverting
// to its power-on default GCONF (0x101) mid-flight, which silently
// switched it to MS1/MS2-pin microstep selection. With R's UART-address
// strap MS1=1, the chip ends up running at 32 microsteps while L stays
// at the configured 16, halving R's commanded rotation rate. The chip
// itself flags this in GSTAT bit 0 (reset latch), but our boot-time
// configureDriver() only ran once.
//
// This task polls each driver once per second, compares GCONF against
// EXPECTED_GCONF, and re-runs applyDriverConfig() on any chip whose
// config has drifted. It also bumps a per-side counter that the / page
// surfaces, so a flaky power supply shows up as a steadily climbing
// number rather than a silent slowness.
//
// We deliberately don't read GSTAT here: that register's reset latch
// is cleared on read, and the status page also reads it. Comparing
// GCONF directly is race-free against any other reader.
constexpr TickType_t WATCHDOG_PERIOD_TICKS = pdMS_TO_TICKS(1000);
constexpr BaseType_t CORE_WATCHDOG = 0; // share the comms core; not timing-critical

void watchdogTask(void* /*arg*/) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, WATCHDOG_PERIOD_TICKS);

    UartGuard g;
    if (readyL) {
      const uint32_t gc = drvL.GCONF();
      if (gc != EXPECTED_GCONF) {
        Serial.print(F("motors: WARN driver L GCONF drift 0x"));
        Serial.print(gc, HEX);
        Serial.print(F(" != 0x"));
        Serial.print(EXPECTED_GCONF, HEX);
        Serial.println(F(" — reapplying full config"));
        applyDriverConfig(drvL, params::current);
        // Verify the rewrite actually stuck before counting it as a
        // recovery; otherwise we'd over-count if the chip is offline.
        if (drvL.GCONF() == EXPECTED_GCONF) {
          ++resetRecoveryL;
        }
      }
    }
    if (readyR) {
      const uint32_t gc = drvR.GCONF();
      if (gc != EXPECTED_GCONF) {
        Serial.print(F("motors: WARN driver R GCONF drift 0x"));
        Serial.print(gc, HEX);
        Serial.print(F(" != 0x"));
        Serial.print(EXPECTED_GCONF, HEX);
        Serial.println(F(" — reapplying full config"));
        applyDriverConfig(drvR, params::current);
        if (drvR.GCONF() == EXPECTED_GCONF) {
          ++resetRecoveryR;
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// printDriverDetailsHtml helpers. Bit decoding follows the TMC2209
// datasheet §5 (DRV_STATUS), §6 (CHOPCONF), §1.3 (IOIN). Anything the
// chip cannot report (e.g. actual delivered current — only the
// configured run/hold scaling is observable via CS_ACTUAL) is either
// omitted or labelled "configured".
// ---------------------------------------------------------------------------

// Helper: decode IOIN low byte into a pin-state list. The bit layout is
// from the TMC2209 datasheet IOIN register; we print only the bits the
// chip actually drives (no inferred state).
void printIoinBits(::Print& out, uint32_t ioin) {
  // bit 0 ENN, 2 MS1, 3 MS2, 4 DIAG, 6 PDN_UART, 7 STEP, 8 SPREAD_EN, 9 DIR
  out.print((ioin & (1u << 0)) ? F("ENN=1 ")        : F("ENN=0 "));
  out.print((ioin & (1u << 2)) ? F("MS1=1 ")        : F("MS1=0 "));
  out.print((ioin & (1u << 3)) ? F("MS2=1 ")        : F("MS2=0 "));
  out.print((ioin & (1u << 4)) ? F("DIAG=1 ")       : F("DIAG=0 "));
  out.print((ioin & (1u << 6)) ? F("PDN_UART=1 ")   : F("PDN_UART=0 "));
  out.print((ioin & (1u << 7)) ? F("STEP=1 ")       : F("STEP=0 "));
  out.print((ioin & (1u << 8)) ? F("SPREAD_EN=1 ")  : F("SPREAD_EN=0 "));
  out.print((ioin & (1u << 9)) ? F("DIR=1")         : F("DIR=0"));
}

// Decode DRV_STATUS error/warning flags into a span list. Only emits
// names for bits that are actually set, plus an "ok" badge if none.
void printDrvStatusFlags(::Print& out, uint32_t st) {
  bool any = false;
  auto emit = [&](bool cond, const __FlashStringHelper* name,
                  const __FlashStringHelper* cls) {
    if (!cond) return;
    out.print(F("<span class=")); out.print(cls);
    out.print('>'); out.print(name); out.print(F("</span> "));
    any = true;
  };
  emit(st & (1u << 1),  F("ot"),    F("fail"));   // overtemp shutdown
  emit(st & (1u << 0),  F("otpw"),  F("warn"));   // overtemp prewarning
  emit(st & (1u << 2),  F("s2ga"),  F("fail"));
  emit(st & (1u << 3),  F("s2gb"),  F("fail"));
  emit(st & (1u << 4),  F("s2vsa"), F("fail"));
  emit(st & (1u << 5),  F("s2vsb"), F("fail"));
  emit(st & (1u << 6),  F("ola"),   F("warn"));   // open load A
  emit(st & (1u << 7),  F("olb"),   F("warn"));   // open load B
  emit(st & (1u << 11), F("t157"),  F("fail"));
  emit(st & (1u << 10), F("t150"),  F("warn"));
  emit(st & (1u << 9),  F("t143"),  F("warn"));
  emit(st & (1u << 8),  F("t120"),  F("warn"));
  if (!any) out.print(F("<span class=ok>ok</span>"));
}

void printDriverDetailsImpl(::Print& out, char side, TMC2209Stepper& d,
                            bool driverReady, uint8_t addr) {
  out.print(F("<tr><td class=k>driver "));
  out.print(side);
  out.print(F(" (addr "));
  out.print(addr);
  out.print(F(")</td><td>"));
  if (!driverReady) {
    out.print(F("<span class=fail>not detected</span> &mdash; no UART ack at boot; "
                "register reads below would be bus-floating noise"));
    out.print(F("</td></tr>"));
    return;
  }
  out.print(F("<span class=ok>OK</span>"));
  // Reset-recovery counter: how many times the watchdog observed this
  // driver's GCONF drifting away from EXPECTED_GCONF (i.e. the chip
  // browned out and we had to re-apply config). Climbing → power supply
  // problem on this driver's VMOT rail.
  const uint32_t recoveries = (side == 'L' || side == 'l')
      ? resetRecoveryL : resetRecoveryR;
  if (recoveries > 0) {
    out.print(F(" <span class=warn>"));
    out.print((unsigned long)recoveries);
    out.print(F(" reset"));
    if (recoveries != 1) out.print('s');
    out.print(F(" recovered</span>"));
  }
  out.print(F("</td></tr>"));

  // ---- Live register reads (UART; serialise with watchdog task) -------
  UartGuard g;
  const uint32_t ioin   = d.IOIN();
  const uint32_t gconf  = d.GCONF();
  const uint8_t  ifcnt  = d.IFCNT();
  const uint8_t  gstat  = d.GSTAT();          // NOTE: read clears the latches
  const uint16_t ms     = d.microsteps();     // CHOPCONF.MRES
  const uint32_t status = d.DRV_STATUS();
  const uint32_t tstep  = d.TSTEP();
  const uint16_t rmsConfigured = d.rms_current();
  const uint8_t  csActual = (status >> 16) & 0x1F;
  const bool     stst     = (status >> 31) & 0x1;
  const bool     stealth  = (status >> 30) & 0x1;

  char buf[64];

  snprintf(buf, sizeof(buf), "0x%08lX (ver=0x%02lX)",
           (unsigned long)ioin, (unsigned long)((ioin >> 24) & 0xFF));
  out.print(F("<tr><td class=k>&nbsp;&nbsp;IOIN</td><td>")); out.print(buf);
  out.print(F("<br><span style='color:#888;font-size:11px'>"));
  printIoinBits(out, ioin);
  out.print(F("</span></td></tr>"));

  out.print(F("<tr><td class=k>&nbsp;&nbsp;IFCNT</td><td>"));
  out.print((unsigned)ifcnt);
  out.print(F(" <span style='color:#888;font-size:11px'>"
              "(register-write counter; should advance on each setParam)"
              "</span></td></tr>"));

  snprintf(buf, sizeof(buf), "0x%X", (unsigned)gstat);
  out.print(F("<tr><td class=k>&nbsp;&nbsp;GSTAT</td><td>")); out.print(buf);
  out.print(' ');
  if (gstat == 0) out.print(F("<span class=ok>clean</span>"));
  else {
    if (gstat & 0x1) out.print(F("<span class=warn>reset</span> "));
    if (gstat & 0x2) out.print(F("<span class=fail>drv_err</span> "));
    if (gstat & 0x4) out.print(F("<span class=fail>uv_cp</span>"));
  }
  out.print(F(" <span style='color:#888;font-size:11px'>"
              "(latches; cleared on read)</span>"));
  out.print(F("</td></tr>"));

  // GCONF — verify the bits we requested actually stuck. The
  // asymmetric-step-rate hypothesis (one driver running at the wrong
  // microstep scaling) hinges on mstep_reg_select=1: when 0, MS1/MS2
  // select microsteps from the strap pins instead of CHOPCONF.MRES,
  // and the two drivers (which have different MS1/MS2 straps for
  // UART addressing!) would silently run at different microstep
  // counts despite identical CHOPCONF writes.
  snprintf(buf, sizeof(buf), "0x%lX", (unsigned long)gconf);
  out.print(F("<tr><td class=k>&nbsp;&nbsp;GCONF</td><td>")); out.print(buf);
  out.print(F("<br><span style='color:#888;font-size:11px'>"));
  out.print((gconf & (1u << 0)) ? F("I_scale_analog=1 ")    : F("I_scale_analog=0 "));
  out.print((gconf & (1u << 1)) ? F("internal_Rsense=1 ")   : F("internal_Rsense=0 "));
  out.print((gconf & (1u << 2)) ? F("en_spreadCycle=1 ")    : F("en_spreadCycle=0 "));
  out.print((gconf & (1u << 3)) ? F("shaft=1 ")             : F("shaft=0 "));
  out.print((gconf & (1u << 4)) ? F("index_otpw=1 ")        : F("index_otpw=0 "));
  out.print((gconf & (1u << 5)) ? F("index_step=1 ")        : F("index_step=0 "));
  out.print((gconf & (1u << 6)) ? F("pdn_disable=1 ")       : F("pdn_disable=0 "));
  if (gconf & (1u << 7)) {
    out.print(F("<span class=ok>mstep_reg_select=1</span> "));
  } else {
    out.print(F("<span class=fail>mstep_reg_select=0 (MRES from MS1/MS2 pins!)</span> "));
  }
  out.print((gconf & (1u << 8)) ? F("multistep_filt=1 ")    : F("multistep_filt=0 "));
  out.print((gconf & (1u << 9)) ? F("test_mode=1")          : F("test_mode=0"));
  out.print(F("</span></td></tr>"));

  out.print(F("<tr><td class=k>&nbsp;&nbsp;microsteps</td><td>"));
  out.print((unsigned)ms);
  out.print(F(" <span style='color:#888;font-size:11px'>"
              "(read from CHOPCONF.MRES)</span></td></tr>"));

  out.print(F("<tr><td class=k>&nbsp;&nbsp;rms_current</td><td>"));
  out.print((unsigned)rmsConfigured);
  out.print(F(" mA <span style='color:#888;font-size:11px'>"
              "(configured; chip does not report actual amps)</span>"
              "</td></tr>"));

  out.print(F("<tr><td class=k>&nbsp;&nbsp;CS_ACTUAL</td><td>"));
  out.print((unsigned)csActual);
  out.print(F("/31 <span style='color:#888;font-size:11px'>"
              "(live 5-bit scaler; reflects IRUN when running, IHOLD at standstill)"
              "</span></td></tr>"));

  out.print(F("<tr><td class=k>&nbsp;&nbsp;TSTEP</td><td>"));
  if ((tstep & 0xFFFFF) == 0xFFFFF) {
    out.print(F("&infin; <span style='color:#888;font-size:11px'>"
                "(0xFFFFF = no step within timeout, i.e. stopped)</span>"));
  } else {
    out.print((unsigned long)(tstep & 0xFFFFF));
    out.print(F(" <span style='color:#888;font-size:11px'>"
                "(internal-clock ticks between last two steps)</span>"));
  }
  out.print(F("</td></tr>"));

  out.print(F("<tr><td class=k>&nbsp;&nbsp;DRV_STATUS</td><td>"));
  snprintf(buf, sizeof(buf), "0x%08lX ", (unsigned long)status);
  out.print(buf);
  printDrvStatusFlags(out, status);
  out.print(F("<br><span style='color:#888;font-size:11px'>"));
  out.print(stst    ? F("stst=1 (standstill) ") : F("stst=0 (moving) "));
  out.print(stealth ? F("stealth=1")            : F("stealth=0 (spreadCycle)"));
  out.print(F("</span></td></tr>"));
}

}  // namespace

bool start(const ControlParams& cp) {
  // 1) UART bring-up. Serial2 defaults to GPIO 16/17, which matches the
  //    pin map; we still pass them explicitly so a future board revision
  //    that remaps UART2 won't silently break things.
  kDrvUart.begin(DRV_UART_BAUD, SERIAL_8N1, PIN_DRV_UART_RX, PIN_DRV_UART_TX);
  // Brief settle — TMC2209 needs a few ms after VCC_IO is up before it
  // accepts UART. The drivers are powered by the 5 V buck which comes up
  // well before this is called, but be defensive.
  delay(10);

  if (kUartMutex == nullptr) {
    kUartMutex = xSemaphoreCreateMutex();
  }

  // --- Address scan -------------------------------------------------------
  // The two drivers share one UART; each is selected by the MS1/MS2 strap
  // pins (00=0, 01=1, 10=2, 11=3). If the wiring is right but the straps
  // aren't, IFCNT/IOIN reads at the *expected* address return 0 (lib's
  // default on read failure), but reads at the *actual* address will
  // return non-zero with VERSION byte 0x21. Scanning all four lets us
  // tell "wrong address" apart from "no signal on the bus at all".
  Serial.println(F("motors: scanning UART addresses 0..3"));
  {
    UartGuard g;
    for (uint8_t a = 0; a < 4; ++a) {
      TMC2209Stepper probe{&kDrvUart, DRV_RSENSE_OHM, a};
      probe.begin();
      const uint32_t ioin = probe.IOIN();
      Serial.print(F("  addr=")); Serial.print(a);
      Serial.print(F(" IOIN=0x")); Serial.print(ioin, HEX);
      Serial.print(F(" ver=0x"));  Serial.print((ioin >> 24) & 0xFF, HEX);
      if (((ioin >> 24) & 0xFF) == 0x21) {
        Serial.println(F("  <-- TMC2209 found"));
      } else {
        Serial.println();
      }
    }
  }

  bool okL, okR;
  {
    UartGuard g;
    okL = probeDriver(drvL, cp, F("L"));
    okR = probeDriver(drvR, cp, F("R"));
  }
  readyL = okL;
  readyR = okR;

  Serial.print(F("motors: driverL="));
  Serial.print(okL ? F("ok") : F("FAIL"));
  Serial.print(F(" driverR="));
  Serial.println(okR ? F("ok") : F("FAIL"));

  // Spawn the GCONF-drift watchdog. Stack is small (it does only a few
  // register reads and an occasional config rewrite), priority is low
  // (recovery is best-effort, the controller must keep its slot).
  if (okL || okR) {
    xTaskCreatePinnedToCore(
      watchdogTask, "drv-wdog", 2048, nullptr, 1, nullptr, CORE_WATCHDOG);
  }

  return okL && okR;
}

bool isReadyL() { return readyL; }
bool isReadyR() { return readyR; }

void applyParamsLive(const ControlParams& cp) {
  UartGuard g;
  // Use the full applyDriverConfig path rather than just rewriting
  // microsteps + currents: this also re-asserts the GCONF bits
  // (mstep_reg_select, pdn_disable, etc.). Cheap, idempotent, and
  // means a setParam from the UI will recover a driver that is
  // sitting in post-reset defaults without waiting for the 1 Hz
  // watchdog tick.
  if (readyL) applyDriverConfig(drvL, cp);
  if (readyR) applyDriverConfig(drvR, cp);
}

void printStatus(::Print& out) {
  out.print(F("motors(drv): "));
  if (readyL) {
    UartGuard g;
    out.print(F("L{ms="));
    out.print(drvL.microsteps());
    out.print(F(",rms="));
    out.print(drvL.rms_current());
    out.print(F("mA,gstat=0x"));
    out.print(drvL.GSTAT(), HEX);
    out.print('}');
  } else {
    out.print(F("L{FAIL}"));
  }
  out.print(' ');
  if (readyR) {
    UartGuard g;
    out.print(F("R{ms="));
    out.print(drvR.microsteps());
    out.print(F(",rms="));
    out.print(drvR.rms_current());
    out.print(F("mA,gstat=0x"));
    out.print(drvR.GSTAT(), HEX);
    out.print('}');
  } else {
    out.print(F("R{FAIL}"));
  }
  out.println();
}

void printDriverDetailsHtml(::Print& out, char side) {
  if (side == 'L' || side == 'l') {
    printDriverDetailsImpl(out, 'L', drvL, readyL, DRV_ADDR_L);
  } else {
    printDriverDetailsImpl(out, 'R', drvR, readyR, DRV_ADDR_R);
  }
}

void readDriverSnapshot(char side, motors::DriverSnapshot& out) {
  const bool left = (side == 'L' || side == 'l');
  TMC2209Stepper& d   = left ? drvL : drvR;
  const bool      rdy = left ? readyL : readyR;
  const uint32_t  rec = left ? resetRecoveryL : resetRecoveryR;

  out = motors::DriverSnapshot{};
  if (!rdy) {
    out.valid = 0;
    return;
  }

  UartGuard g;
  const uint32_t ioin   = d.IOIN();
  const uint32_t gconf  = d.GCONF();
  const uint8_t  ifcnt  = d.IFCNT();
  const uint8_t  gstat  = d.GSTAT();          // read clears latches
  const uint16_t ms     = d.microsteps();     // CHOPCONF.MRES
  const uint32_t status = d.DRV_STATUS();
  const uint32_t tstep  = d.TSTEP();
  const uint16_t rmsCfg = d.rms_current();

  out.valid             = 1;
  out.ioin              = ioin;
  out.gconf             = gconf;
  out.drv_status        = status;
  out.tstep             = tstep;
  out.reset_recoveries  = rec;
  out.microsteps        = ms;
  out.rms_current_ma    = rmsCfg;
  out.ifcnt             = ifcnt;
  out.gstat             = gstat;
  out.cs_actual         = static_cast<uint8_t>((status >> 16) & 0x1F);
}

TMC2209Stepper& driverL() { return drvL; }
TMC2209Stepper& driverR() { return drvR; }

void uartLock()   { if (kUartMutex) xSemaphoreTake(kUartMutex, portMAX_DELAY); }
void uartUnlock() { if (kUartMutex) xSemaphoreGive(kUartMutex); }

}  // namespace drv
}  // namespace motors
