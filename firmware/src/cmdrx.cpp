// cmdrx.cpp — see cmdrx.h.
//
// Handler bodies are migrated from the original net.cpp's WS dispatcher,
// minus the AsyncWebSocketClient* parameter and minus the ack/error/
// reply paths. The handler set:
//
//   "setParam"      — write a field in params::current
//   "setTarget"     — write shared::g.targetV (with vMaxCart saturation)
//   "setTurn"       — write shared::g.targetTurn (with vMaxTurn saturation)
//   "enableMotors"  — call safety::requestArm / requestDisarm
//   "reset"         — call safety::requestReset
//   "saveParams"    — params::saveToNvs (BLOCKING ~10 ms)
//   "loadDefaults"  — reset to defaults + applyParamsLive
//   "calibrate"     — kick imu::requestCalibration + spawn watcher
//                     task to save NVS on completion
//   "getParams"     — handled entirely on coproc from cached params JSON;
//                     never forwarded to main, never reaches this code.

#include "cmdrx.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>

#include "imu.h"
#include "motors.h"
#include "params.h"
#include "safety.h"
#include "shared_state.h"
#include "telemetry.h"
#include "uartbus.h"

namespace cmdrx {

namespace {

bool s_started = false;

// Each handler returns nothing — there's no client to reply to. Failures
// (bad path, unknown type, etc.) get logged to Serial; the UI observes
// the *absence* of effect via the next params/telemetry snapshot.

void handleSetParam(JsonObjectConst msg) {
  const char* path = msg["path"] | static_cast<const char*>(nullptr);
  if (path == nullptr) {
    Serial.println(F("cmdrx: setParam missing path"));
    return;
  }
  if (msg["value"].isNull()) {
    Serial.println(F("cmdrx: setParam missing value"));
    return;
  }
  const float v = msg["value"].as<float>();
  if (!params::setByPath(params::current, path, v)) {
    Serial.print(F("cmdrx: setParam unknown path '"));
    Serial.print(path);
    Serial.println(F("'"));
    return;
  }
  // Driver-affecting params take effect immediately. Controller and
  // motors hot path read params::current directly each tick, so other
  // params don't need any plumbing here.
  motors::applyParamsLive();
}

void handleSetTarget(JsonObjectConst msg) {
  if (msg["v"].isNull()) return;
  float v = msg["v"].as<float>();
  const float vmax = params::current.vMaxCart;
  if (vmax > 0.0f) {
    if (v >  vmax) v =  vmax;
    if (v < -vmax) v = -vmax;
  }
  shared::g.targetV.store(v);
}

void handleSetTurn(JsonObjectConst msg) {
  if (msg["v"].isNull()) return;
  float v = msg["v"].as<float>();
  const float vmax = params::current.vMaxTurn;
  if (vmax > 0.0f) {
    if (v >  vmax) v =  vmax;
    if (v < -vmax) v = -vmax;
  }
  shared::g.targetTurn.store(v);
}

void handleEnableMotors(JsonObjectConst msg) {
  const bool on = msg["value"].as<bool>();
  if (on) safety::requestArm();
  else    safety::requestDisarm();
}

void handleReset() {
  safety::requestReset();
}

void handleSaveParams() {
  params::saveToNvs(params::current);
}

void handleLoadDefaults() {
  params::current = params::defaults();
  motors::applyParamsLive();
  // diag's params-change detector will push the fresh params to coproc
  // on its next 1 Hz tick; coproc forwards to WS clients.
}

// One-shot task: poll the IMU until calibration completes, save the
// new bias to NVS, self-delete. Spawned from handleCalibrate. We avoid
// blocking the uartbus rx task on the 3 s poll — putting it in its own
// task keeps the rx pipeline responsive.
void calibrateWatcherTask(void* /*arg*/) {
  constexpr uint32_t TIMEOUT_MS  = 3500;
  constexpr uint32_t POLL_PERIOD = 50;
  const uint32_t deadline = millis() + TIMEOUT_MS;
  imu::CalState st = imu::CalState::PENDING;
  while (millis() < deadline) {
    st = imu::calibrationState();
    if (st == imu::CalState::DONE_OK || st == imu::CalState::DONE_ERR) break;
    vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD));
  }
  if (st == imu::CalState::DONE_OK) {
    // Persist the fresh bias so next boot doesn't have to re-cal.
    params::saveToNvs(params::current);
    Serial.println(F("cmdrx: calibrate done, NVS saved"));
  } else if (st == imu::CalState::DONE_ERR) {
    // imu task zeroed the bias on failure. NVS unchanged.
    Serial.println(F("cmdrx: calibrate failed (chassis moved?)"));
  } else {
    Serial.println(F("cmdrx: calibrate timed out waiting for IMU"));
  }
  imu::clearCalibrationState();
  vTaskDelete(nullptr);
}

void handleCalibrate() {
  if (shared::getFlag(shared::FLAG_MOTORS_ENABLED)) {
    Serial.println(F("cmdrx: calibrate refused (motors armed)"));
    return;
  }
  if (!imu::isReady()) {
    Serial.println(F("cmdrx: calibrate refused (IMU not ready)"));
    return;
  }
  if (!imu::requestCalibration()) {
    Serial.println(F("cmdrx: calibrate already in progress"));
    return;
  }
  // Spawn the watcher; it self-deletes after the 3 s window. Low
  // priority — non-real-time, on the comms core.
  xTaskCreatePinnedToCore(
      calibrateWatcherTask, "calwatch", 3072, nullptr,
      /*priority=*/1, nullptr, /*core=*/0);
}

void dispatch(const uint8_t* json, size_t len) {
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, json, len);
  if (err) {
    Serial.print(F("cmdrx: bad JSON: "));
    Serial.println(err.c_str());
    return;
  }
  JsonObjectConst msg = doc.as<JsonObjectConst>();
  const char* type = msg["type"] | static_cast<const char*>(nullptr);
  if (type == nullptr) {
    Serial.println(F("cmdrx: missing 'type'"));
    return;
  }

  if      (strcmp(type, "setParam")     == 0) handleSetParam(msg);
  else if (strcmp(type, "setTarget")    == 0) handleSetTarget(msg);
  else if (strcmp(type, "setTurn")      == 0) handleSetTurn(msg);
  else if (strcmp(type, "enableMotors") == 0) handleEnableMotors(msg);
  else if (strcmp(type, "reset")        == 0) handleReset();
  else if (strcmp(type, "saveParams")   == 0) handleSaveParams();
  else if (strcmp(type, "loadDefaults") == 0) handleLoadDefaults();
  else if (strcmp(type, "calibrate")    == 0) handleCalibrate();
  else if (strcmp(type, "setTelemetryEnabled") == 0) {
    telemetry::setEnabled(msg["enabled"].as<bool>());
  }
  else {
    Serial.print(F("cmdrx: unknown type '"));
    Serial.print(type);
    Serial.println(F("'"));
  }
}

} // namespace

void start() {
  if (s_started) return;
  s_started = true;
  uartbus::setCommandHandler(&dispatch);
  Serial.println(F("cmdrx: WS-command dispatcher armed"));
}

} // namespace cmdrx
