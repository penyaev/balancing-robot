// cmdrx.h — UART-driven WS command dispatcher.
//
// The simulator continues to send {"type":"setParam",…} etc. over its
// WebSocket. The coproc forwards each JSON payload to main as a
// PKT_WS_CMD UART packet (see wire_proto.h). This module receives those,
// parses the JSON with ArduinoJson, and calls the same handlers that
// used to live in net.cpp on the AsyncWebSocketClient code path.
//
// Differences from the old net.cpp world:
//
//   - No AsyncWebSocketClient*. Replies (sendAck, sendError, sendParams,
//     broadcastParams) are gone — the coproc already acked the WS client
//     immediately on receipt, before forwarding. Any "side effects
//     visible to the UI" land via the diag.cpp params/status push, on
//     the next periodic tick or on-change.
//
//   - handleCalibrate doesn't block the WS handler for 3 s. It kicks off
//     the imu::requestCalibration() flow and spawns a one-shot watcher
//     task that polls for completion and saves the new bias to NVS.
//     The watcher self-deletes when done. The UI sees the new bias when
//     diag's next params snapshot picks up the change.
//
// Lifetime: start() registers a callback with uartbus::setCommandHandler.
// The callback runs on uartbus's rx task; handlers are mostly atomic
// (microseconds), with the notable exception of saveParams (NVS write,
// ~10 ms). Acceptable given the low rate of WS commands.

#pragma once

namespace cmdrx {

// Idempotent. Registers our callback with uartbus.
void start();

} // namespace cmdrx
