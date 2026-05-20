// main.cpp — F1 bootstrap.
//
// At this checkpoint the firmware does the absolute minimum:
//   - Print a serial banner with version and build timestamp.
//   - Spawn a low-priority FreeRTOS task that blinks the status LED at 1 Hz.
//
// All real subsystems (IMU, motors, controller, comms, joystick, safety,
// battery) are stubbed and will be filled in over TODO items F2–F13.
// The task structure here matches the layout in PLAN.md §3 so subsequent
// tasks can slot into the right core without restructuring.

#include <Arduino.h>

#include "config.h"
#include "params.h"
#include "shared_state.h"
#include "battery.h"
#include "boot_diag.h"
#include "cmdrx.h"
#include "controller.h"
#include "diag.h"
#include "imu.h"
#include "ina226.h"
#include "joystick.h"
#include "motors.h"
#include "safety.h"
#include "telemetry.h"
#include "uartbus.h"

namespace {

// Tasks pinned per PLAN.md §3:
//   Core 1: controlTask (high), safetyTask (highest)
//   Core 0: joystickTask (mid), wsServerTask (mid), telemetryTask (low),
//           plus the blink task while we're still bootstrapping.
constexpr BaseType_t CORE_CONTROL = 1;
constexpr BaseType_t CORE_COMMS   = 0;

void blinkTask(void* /*arg*/) {
  pinMode(PIN_STATUS_LED, OUTPUT);

  // F13: status LED encodes the bot's high-level state. Patterns are
  // 10 × 100 ms ticks (1 Hz cycle) so any single-flag change is
  // visible to a human within a second. Priority order:
  //
  //   FALLEN  -> 3 quick pulses, long gap     (something needs your attention)
  //   LOW_BAT -> 50 % duty fast blink         (charge me)
  //   normal  -> brief heartbeat              (alive, no problems)
  //
  // We deliberately don't show ARMED / PS_CONNECTED here — those are
  // visible in telemetry and on the controller itself, and adding more
  // patterns just makes the LED harder to read at a glance.
  static const bool PATTERN_NORMAL [10] = {1,0,0,0,0,0,0,0,0,0};
  static const bool PATTERN_LOWBAT [10] = {1,0,1,0,1,0,1,0,1,0};
  static const bool PATTERN_FALLEN [10] = {1,0,1,0,1,0,0,0,0,0};

  for (;;) {
    const uint16_t flags = shared::g.flags.load(std::memory_order_relaxed);
    const bool* pattern = PATTERN_NORMAL;
    if (flags & shared::FLAG_FALLEN)        pattern = PATTERN_FALLEN;
    else if (flags & shared::FLAG_LOW_BAT)  pattern = PATTERN_LOWBAT;

    for (int i = 0; i < 10; ++i) {
      digitalWrite(PIN_STATUS_LED, pattern[i] ? HIGH : LOW);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void printBanner() {
  Serial.println();
  Serial.println(F("======================================"));
  Serial.print(F("  balancing-bot firmware v"));
  Serial.println(F(BB_FW_VERSION));
  Serial.print(F("  built "));
  Serial.print(F(__DATE__));
  Serial.print(F(" "));
  Serial.println(F(__TIME__));
  Serial.print(F("  CPU "));
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print(F(" MHz, free heap "));
  Serial.print(ESP.getFreeHeap());
  Serial.println(F(" B"));
  Serial.println(F("======================================"));
}

} // namespace

void setup() {
  Serial.begin(115200);
  // Give the USB-serial bridge a moment to come up after reset so the banner
  // isn't lost. Not strictly required, but nicer during bringup.
  delay(200);
  // Capture the reset reason FIRST, before any other subsystem runs —
  // we want this in the log even if a later init step hangs / panics
  // (in which case the *next* boot will tell us PANIC right at the top
  // and we'll know exactly which run died). Also surfaces on the /
  // status page via boot_diag::reasonStr() / isAlarming() so the
  // operator sees "last reset: BROWNOUT" without scrolling serial.
  boot_diag::init();
  printBanner();

  // Make sure the motor drivers boot disabled. They will be re-enabled by
  // safetyTask once IMU is healthy and the bot is armed (F8).
  pinMode(PIN_DRV_EN, OUTPUT);
  digitalWrite(PIN_DRV_EN, HIGH); // active-low → HIGH = disabled

  // F2: load runtime params. Defaults baked-in; NVS overrides on top.
  params::current = params::defaults();
  const int loaded = params::loadFromNvs(params::current);
  Serial.print(F("params: loaded "));
  Serial.print(loaded);
  Serial.println(F(" field(s) from NVS"));
  Serial.print(F("params: "));
  params::printSummary(params::current, Serial);
  Serial.println();

  // Self-test: roundtrip a single field through setByPath + JSON. Catches
  // table-wiring mistakes early without needing a host to talk to.
  {
    const float orig = params::current.velKp;
    bool ok = params::setByPath(params::current, "velKp", 0.42f);
    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    params::toJson(params::current, obj);
    ControlParams cp2 = params::defaults();
    const int applied = params::fromJson(cp2, obj);
    Serial.print(F("params self-test: setByPath="));
    Serial.print(ok ? F("ok") : F("FAIL"));
    Serial.print(F(", json roundtrip applied="));
    Serial.print(applied);
    Serial.print(F(", velKp roundtrip="));
    Serial.print(cp2.velKp, 3);
    Serial.println();
    params::current.velKp = orig; // don't actually mutate live params
  }

  xTaskCreatePinnedToCore(
    blinkTask, "blink", 1024, nullptr, 1, nullptr, CORE_COMMS);

  // F4: start battery monitor (publishes vBat to shared, drives FLAG_LOW_BAT,
  // logs every 1 s on serial).
  battery::start();

  // F5: start IMU task (200 Hz on core 1). Performs a stand-still gyro bias
  // calibration on boot, then runs the complementary filter and publishes
  // theta / thetaDot to shared state. imu::isReady() flips true once ready.
  imu::start();

  // INA226 power monitor (shared I²C bus with the MPU6050). Sampling task
  // pushes vBus + iBus into shared::g at 50 Hz. Started after the IMU so
  // Wire.begin() has already run; ina226::start() is idempotent in case
  // that ever changes. Init-write failure (chip absent) doesn't block
  // anything else — the rest of the firmware doesn't depend on it; the
  // telemetry frame just shows 0/0 until reads succeed.
  ina226::start();

  // F6: bring up TMC2209 drivers and the LEDC step generator. EN is left
  // HIGH (drivers disabled) — the safety FSM (F8) decides when to drop EN
  // low. Until then, calling motors::setWheelVelocity() still drives the
  // STEP pins but the drivers ignore the edges, which is exactly what we
  // want during bench bring-up.
  motors::start();
  motors::printStatus(Serial);

  // F7: spawn the 200 Hz cascaded controller. It will idle until the IMU
  // task finishes its boot-time bias cal, then start running the math.
  // FLAG_MOTORS_ENABLED is still clear at this point — the controller
  // publishes outputs but tells motors::stop() instead of issuing step
  // commands. F8 will arm.
  controller::start();

  // F8: bring up the arming FSM. From this point on, safetyTask owns the
  // EN line and FLAG_MOTORS_ENABLED. It will move DISARMED → READY once
  // the IMU finishes its bias cal and battery + tilt look sane; the bot
  // will only ARM in response to safety::requestArm() (eventually wired
  // to the WS / joystick channels).
  safety::start();

  // F10' (architecture inversion): WiFi/HTTP/WS lives on the XIAO S3
  // coproc now. Main only owns the UART bus to the coproc. Bring that
  // up before the producers (telemetry, diag) so their send* calls
  // don't no-op on a not-yet-started codec.
  uartbus::start();

  // cmdrx: register the WS-command dispatcher with uartbus. From here on,
  // incoming PKT_WS_CMD packets (forwarded by the coproc from simulator
  // WS clients) get parsed + routed to the handlers we used to run on
  // the AsyncWebSocket dispatch path.
  cmdrx::start();

  // F11: start telemetry broadcaster (60 Hz, core 0). Snapshots shared::g
  // into a packed 116-byte little-endian frame and ships it to the
  // coproc via uartbus::sendTelemetry; coproc relays to WS clients.
  telemetry::start();

  // diag: 1 Hz status snapshot + on-change params dump pushed to coproc
  // over the same UART. Coproc caches both for the status page and for
  // WS `getParams` replies.
  diag::start();

  // F12: bring up PS5 DualSense over classic BT (HamzaYslmn/esp-ps5).
  // Configured via BB_PS5_MAC at build time — empty MAC falls back to
  // discovery. Callbacks set shared::g.targetV/targetTurn and route
  // PS / Share / Options to safety::request*().
  joystick::start();

  // F3: smoke-test SharedState atomics + flag helpers. Cheap, runs once.
  {
    using namespace shared;
    g.targetV.store(0.25f);
    g.theta.store(-0.01f);
    setFlag(FLAG_PS_CONNECTED, true);
    setFlag(FLAG_FALLEN, false);
    Serial.print(F("shared: targetV="));
    Serial.print(g.targetV.load(), 3);
    Serial.print(F(" theta="));
    Serial.print(g.theta.load(), 3);
    Serial.print(F(" flags="));
    printFlags(g.flags.load(), Serial);
    Serial.print(F(" floatLockFree="));
    Serial.print(std::atomic<float>{}.is_lock_free() ? F("yes") : F("no"));
    Serial.println();
    // Reset state so subsequent tasks see zeroed shared state.
    g.targetV.store(0.0f);
    g.theta.store(0.0f);
    setFlag(FLAG_PS_CONNECTED, false);
  }

  Serial.println(F("setup() complete; idle loop running."));
}

void loop() {
  // Nothing here on purpose. All real work happens in FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
