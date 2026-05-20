// uartbus.cpp — see uartbus.h.
//
// Implementation notes:
//
// * TX: a single FreeRTOS mutex serializes Serial1.write() calls from
//   any caller AND access to the shared s_txBuf scratch buffer. The
//   buffer is static (file-scope) rather than on the caller's stack
//   because callers come from tasks with 4–6 KB stacks and we don't
//   want a TX path to chew through 1+ KB of that. The mutex makes
//   single-static-buffer safe across producers.
//
// * RX: a single task on CORE_COMMS reads bytes from Serial1 as they
//   arrive, feeds a state-machine parser, and on a valid PKT_WS_CMD
//   calls the registered handler. The state machine resyncs on the
//   magic bytes after any kind of failure (bad CRC, garbage at boot,
//   etc.). The payload buffer (~1 KB) is static; the rx task's stack
//   has room only for the small state vars + the registered handler's
//   needs.
//
// * CRC computation uses the chainable wire::crc8(seed) form so we can
//   compute over the header + payload spans incrementally, without
//   first coalescing them into a contiguous scratch buffer (that's
//   what blew the rx task's stack in the original implementation —
//   MAX_PAYLOAD was 4 KB and the scratch was on the stack).
//
// * Why a mutex instead of a queue: producers are all FreeRTOS tasks
//   and the worst-case TX rate (60 Hz × 122 B + occasional 600 B
//   params + 50 B status) is well below saturation at 460800 baud.
//   Mutex contention is rare (mostly single producer at a time) and
//   eliminates the allocation overhead a queue-of-packets would
//   require.

#include "uartbus.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#include "config.h"
#include "wire_proto.h"

namespace uartbus {

namespace {

constexpr BaseType_t CORE_COMMS = 0;
constexpr int        RX_TASK_PRIORITY = 1;
constexpr size_t     TX_BUFFER_SIZE = 4096;   // Serial1 hardware ring
constexpr size_t     RX_BUFFER_SIZE = 1024;   // Serial1 hardware ring
constexpr int        RX_TASK_STACK  = 4096;

bool                 s_started = false;
SemaphoreHandle_t    s_txMutex = nullptr;
CommandHandler       s_cmdHandler = nullptr;

// File-scope scratch buffers. The TX buffer is protected by s_txMutex;
// the RX payload buffer is only ever touched by the single rx task.
// Keeping both off the task stacks is what fixes the original 4 KB
// stack-overflow regression.
uint8_t              s_txBuf[wire::OVERHEAD + wire::MAX_PAYLOAD];
uint8_t              s_rxPayload[wire::MAX_PAYLOAD];

// RX state machine. Stays inline in the rx task — no global state.
enum RxState {
  RX_WAIT_MAGIC1 = 0,
  RX_WAIT_MAGIC2,
  RX_TYPE,
  RX_LEN_LO,
  RX_LEN_HI,
  RX_PAYLOAD,
  RX_CRC,
};

bool writeFrame(uint8_t type, const uint8_t* body, size_t len) {
  if (!s_started) return false;
  if (len > wire::MAX_PAYLOAD) {
    // Caller passed an oversize payload. Silent drop here used to mask
    // a real bug (params JSON grew past MAX_PAYLOAD as we added fields),
    // so make it loud. Single line per drop is acceptable — these
    // shouldn't happen in normal operation.
    Serial.printf("uartbus: payload too large (type=0x%02x len=%u, max=%u), dropping\n",
                  (unsigned)type, (unsigned)len, (unsigned)wire::MAX_PAYLOAD);
    return false;
  }

  // Take the mutex BEFORE writing to s_txBuf — buffer is shared across
  // producers (telemetry/diag/...). pack + write happen entirely under
  // the lock so concurrent senders never see a half-built frame.
  if (xSemaphoreTake(s_txMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    Serial.println(F("uartbus: tx mutex timeout, dropping frame"));
    return false;
  }
  const size_t framed = wire::packFrame(s_txBuf, type, body, len);
  if (framed > 0) {
    Serial1.write(s_txBuf, framed);
  }
  xSemaphoreGive(s_txMutex);
  return framed > 0;
}

void rxTask(void* /*arg*/) {
  RxState  state = RX_WAIT_MAGIC1;
  uint8_t  type = 0;
  uint16_t payloadLen = 0;
  uint16_t payloadIdx = 0;
  // CRC computed incrementally over header+payload via wire::crc8 chain.
  uint8_t  runningCrc = 0;

  for (;;) {
    while (Serial1.available() > 0) {
      const int b = Serial1.read();
      if (b < 0) break;
      const uint8_t byte = static_cast<uint8_t>(b);

      switch (state) {
        case RX_WAIT_MAGIC1:
          if (byte == wire::MAGIC1) {
            runningCrc = wire::crc8(&byte, 1);  // seed from MAGIC1
            state = RX_WAIT_MAGIC2;
          }
          break;
        case RX_WAIT_MAGIC2:
          if (byte == wire::MAGIC2) {
            runningCrc = wire::crc8(&byte, 1, runningCrc);
            state = RX_TYPE;
          } else if (byte == wire::MAGIC1) {
            // Possible alignment artifact: re-seed with this byte.
            runningCrc = wire::crc8(&byte, 1);
            state = RX_WAIT_MAGIC2;
          } else {
            state = RX_WAIT_MAGIC1;
          }
          break;
        case RX_TYPE:
          type = byte;
          runningCrc = wire::crc8(&byte, 1, runningCrc);
          state = RX_LEN_LO;
          break;
        case RX_LEN_LO:
          payloadLen = byte;
          runningCrc = wire::crc8(&byte, 1, runningCrc);
          state = RX_LEN_HI;
          break;
        case RX_LEN_HI:
          payloadLen |= (static_cast<uint16_t>(byte) << 8);
          runningCrc = wire::crc8(&byte, 1, runningCrc);
          if (payloadLen > wire::MAX_PAYLOAD) {
            state = RX_WAIT_MAGIC1;
            break;
          }
          payloadIdx = 0;
          state = (payloadLen == 0) ? RX_CRC : RX_PAYLOAD;
          break;
        case RX_PAYLOAD:
          s_rxPayload[payloadIdx++] = byte;
          runningCrc = wire::crc8(&byte, 1, runningCrc);
          if (payloadIdx >= payloadLen) state = RX_CRC;
          break;
        case RX_CRC:
          if (runningCrc == byte) {
            if (type == wire::PKT_WS_CMD && s_cmdHandler != nullptr) {
              s_cmdHandler(s_rxPayload, payloadLen);
            }
            // PKT_TELEMETRY/STATUS/PARAMS shouldn't arrive on main
            // (those are main → coproc only). Silently drop.
          }
          // Whether CRC matched or not, resync to magic.
          state = RX_WAIT_MAGIC1;
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

} // namespace

void start() {
  if (s_started) return;

  s_txMutex = xSemaphoreCreateMutex();

  Serial1.setRxBufferSize(RX_BUFFER_SIZE);
  Serial1.setTxBufferSize(TX_BUFFER_SIZE);
  Serial1.begin(COPROC_UART_BAUD, SERIAL_8N1,
                PIN_COPROC_UART_RX, PIN_COPROC_UART_TX);

  s_started = true;

  xTaskCreatePinnedToCore(
      rxTask, "uartbus-rx", 4096, nullptr,
      RX_TASK_PRIORITY, nullptr, CORE_COMMS);

  Serial.print(F("uartbus: started @ "));
  Serial.print(COPROC_UART_BAUD);
  Serial.print(F(" baud (TX=GPIO"));
  Serial.print(PIN_COPROC_UART_TX);
  Serial.print(F(" RX=GPIO"));
  Serial.print(PIN_COPROC_UART_RX);
  Serial.println(F(")"));
}

void setCommandHandler(CommandHandler cb) {
  s_cmdHandler = cb;
}

bool sendTelemetry(const uint8_t* body, size_t len) {
  return writeFrame(wire::PKT_TELEMETRY, body, len);
}

bool sendStatus(const uint8_t* body, size_t len) {
  return writeFrame(wire::PKT_STATUS, body, len);
}

bool sendParams(const uint8_t* json, size_t len) {
  return writeFrame(wire::PKT_PARAMS, json, len);
}

bool sendInputEvent(uint8_t code) {
  return writeFrame(wire::PKT_INPUT_EVENT, &code, 1);
}

} // namespace uartbus
