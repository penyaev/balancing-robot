// Data source for the bot web UI.
//
// `DeviceSource` is a WebSocket client to the on-bot firmware: it parses
// the binary telemetry frames, decodes JSON params/ack/error/ping
// messages, owns auto-reconnect + the stale-feed watchdog, and exposes
// the result to main.ts as a per-frame `Snapshot`. The `DataSource`
// interface is the seam main.ts consumes — historically there was also
// a local-physics `SimulatorSource` behind it, but the simulator half
// was removed once this became the live web UI for the real bot.
//
// Snapshot design notes:
//   The snapshot bundles every per-tick value the UI needs: the
//   kinematic state, the last sensor measurement, the controller
//   outputs (angle setpoint, inner/outer term decomposition), and the
//   device-only side-channel (battery, flags, wheel telemetry, …).
//   tSec is the device-supplied frame time.

import { State } from "./types";

export interface SnapshotCtrl {
  tau: number;
  angleSet: number;
  inner: { p: number; i: number; d: number };
  outer: { p: number; i: number; d: number };
}

export interface Snapshot {
  tSec: number;
  state: State;
  measurement: { theta: number; thetaDot: number; xDot: number };
  ctrl: SnapshotCtrl;
  // Device-only side-channel: battery voltage, status flag bitfield
  // (shared::Flag layout — see firmware/src/shared_state.h), and the
  // inner-loop output (vWheelCmd, m/s) so we can chart the firmware's
  // velocity-actuated control law next to its three input components.
  // Present once the WS has decoded at least one telemetry frame; UI
  // panels gate on its presence (undefined before first frame).
  device?: {
    vBat: number;
    flags: number;
    vWheelCmd: number;
    // Actual wheel velocity reported by the firmware's stepper engine,
    // averaged across L and R, in m/s. Compare against vWheelCmd
    // to distinguish a stalled engine (commands sent but no pulses) from
    // a downstream issue (pulses sent but motors not reacting).
    wheelActualMps: number;
    // Per-wheel commanded velocities AFTER turn differential split +
    // ±vMaxWheel saturation, in cart frame (positive = forward, mounting
    // invert NOT applied — that's downstream in the firmware's motors
    // layer, and the actual readouts below have it undone). vWheelCmd
    // above is the common-mode pre-split value; these two are what was
    // actually handed to setWheelVelocity.
    vWheelCmdL: number;
    vWheelCmdR: number;
    // Per-wheel actual velocities (LEDC backend: equals last commanded
    // step rate scaled back to m/s with mounting invert undone, so cart
    // frame matches vWheelCmdL/R sign convention). Charted side-by-side
    // with the cmd lines so asymmetric saturation / per-wheel stalls
    // are visible without the L+R average hiding them.
    wheelActualMpsL: number;
    wheelActualMpsR: number;
    // Body-frame X accel from the MPU6050, in g, raw (no gravity removal).
    // Diagnostic for distinguishing real chassis translation vs. gyro
    // pickup of structural vibration during a wobble.
    accelX: number;
    // Yaw-axis (chip Z) gyro reading, bias-corrected, in rad/s. NO sign
    // flip — this is the raw IMU axis, sign convention positive = CCW
    // seen from above (right-hand rule about chip Z). Diagnostic for
    // yaw-rate bleed into the pitch channel during a turn (mounting
    // misalignment + ±2% datasheet cross-axis sensitivity). If this trace
    // tracks thetaDot during a pure-yaw spin (bot held off the floor and
    // rotated about vertical, no real pitch motion), there's
    // cross-coupling and a compensation term will help.
    gyroZ: number;
    // Raw shared::g.targetV from the firmware (m/s, BEFORE targetVAlpha
    // smoothing). The unfiltered velocity command the operator/RC is
    // asking for. Telemetered (not reconstructed client-side) so the
    // outer-loop chart shows what the controller actually saw, not what
    // we think the joystick was doing.
    targetV: number;
    // Raw shared::g.targetTurn (m/s wheel-velocity differential),
    // unfiltered. Sign convention: positive = bot turns right (CW seen
    // from above, left wheel commanded faster). Telemetered (not
    // reconstructed client-side) for the same reason as targetV — the
    // controller's input shouldn't be guessed from joystick state.
    // Charted as "tgtTurn cmd" — what the operator is *requesting*,
    // BEFORE the targetTurnAlpha ramp filter and the ±vMaxTurn clamp.
    targetTurn: number;
    // Post-targetTurnAlpha + post-±vMaxTurn clamp value the controller
    // actually summed into vL/vR this tick. Held at 0 by the firmware
    // while disarmed / IMU not ready (mirrors the ramp-filter reset).
    // Charted alongside targetTurn as "tgtTurn used" so the difference
    // between "operator commanded a step" and "controller is still
    // ramping in" is visually obvious.
    targetTurnUsed: number;
    // LEDC step-pulse diagnostic pair per side. Units: signed steps/sec
    // (Hz at the STEP pin), NOT m/s — the point is to surface the LEDC
    // peripheral's frequency-rounding behaviour in its native domain.
    // Sign = commanded direction at the time of the call (positive =
    // forward, negative = reverse, 0 = channel idle / below deadband /
    // stopped). Today (LEDC_RES_BITS=8) req and got should track
    // exactly; chart divergence is a regression alarm. Reset to 0 by
    // the firmware on stop() so a Disarm clears the trace.
    ledcReqL: number;
    ledcGotL: number;
    ledcReqR: number;
    ledcGotR: number;
    // INA226 high-side power monitor. Independent from vBat above (which
    // is sampled via the ADC voltage divider on GPIO 34) — gives a
    // separate voltage reading plus current draw at the same point in
    // the pack-to-load chain. Sampled at ~50 Hz on the firmware side;
    // 0 if the chip is absent or hasn't completed its first read yet.
    // Positive iBus = current flowing through the shunt in the
    // direction of the IN+/IN− wiring; regen / reverse current shows
    // as negative.
    vBus: number;
    iBus: number;
  };
}

export interface TickResult {
  snap: Snapshot;
  // How much source-clock time was actually integrated/consumed by this
  // tick. 0 means "no progress this frame" (paused, or device produced
  // no new frames). Charts/debug history use this to gate per-frame
  // pushes so a paused sim doesn't keep spamming the same point.
  advancedSec: number;
  // Monotonically-increasing counter that bumps on every "session
  // boundary": initial connect, every reconnect, every detected device
  // reboot or long feed gap (>1 s). The UI watches for changes here
  // and resets chart buffers / debug history so a forward jump in
  // device tSec doesn't orphan the entire pre-reconnect buffer behind
  // the rolling 30 s window.
  sessionEpoch: number;
}

export interface DataSource {
  reset(): void;

  // Drain whatever telemetry frames the WS has queued and return the
  // latest Snapshot. `advanceSec` is ignored — the device sets its own
  // pace — but kept in the signature so the rAF loop can call tick()
  // uniformly.
  tick(advanceSec: number): TickResult;

  // Catastrophic-error sentinel. Caller polls each frame and on a
  // non-null return reacts (e.g. pause + console.warn). Cleared on read
  // so each error surfaces exactly once.
  consumeError(): string | null;
}

// ---------------------------------------------------------------------------
// DeviceSource — S2.
//
// WebSocket client to the on-bot firmware (firmware/src/telemetry.cpp). The
// device pushes a 116-byte little-endian binary frame at 100 Hz (PLAN.md §5.1
// — note: PLAN.md originally said 56 bytes; actual has grown with each
// telemetry add — 60 → 64 → 68 → 84 → 88 → 92 → 108).
// Frame layout:
//
//   off  0  u32  magic = 0xB0B0B0B0
//   off  4  u32  seq (monotonic)
//   off  8  f32  t       (uptime, s)
//   off 12  f32  theta   (rad)
//   off 16  f32  thetaDot(rad/s)
//   off 20  f32  xDot    (m/s)
//   off 24  f32  thetaSet(rad)
//   off 28  f32  vWheelCmd(m/s)        — common-mode (pre-turn-split) inner-loop output
//   off 32  f32  outerP
//   off 36  f32  outerI
//   off 40  f32  outerD
//   off 44  f32  vBat    (V)           — Snapshot.device.vBat
//   off 48  f32  wheelActualMps        — L+R average (cart frame)
//   off 52  f32  accelX  (g)           — Snapshot.device.accelX
//   off 56  f32  gyroZ   (rad/s)       — Snapshot.device.gyroZ (yaw axis)
//   off 60  u16  flags                 — Snapshot.device.flags
//   off 62  u16  reserved
//   off 64  f32  targetV (m/s)         — Snapshot.device.targetV (raw shared::g.targetV)
//   off 68  f32  vWheelCmdL            — per-wheel post-split, post-saturation cmd
//   off 72  f32  vWheelCmdR            //   (cart frame, mounting invert NOT applied)
//   off 76  f32  wheelActualMpsL       — per-wheel actual (LEDC: == last commanded,
//   off 80  f32  wheelActualMpsR       //   scaled to m/s, mounting invert undone)
//   off 84  f32  targetTurn (m/s)      — Snapshot.device.targetTurn (raw shared::g.targetTurn,
//                                       //   pre-targetTurnAlpha; positive=right turn)
//   off 88  f32  targetTurnUsed (m/s)  — Snapshot.device.targetTurnUsed (post-filter,
//                                       //   post-±vMaxTurn clamp; what the controller
//                                       //   actually consumed into vL/vR this tick)
//   off 92  f32  ledcReqL (steps/sec)  — Snapshot.device.ledcReqL (signed; sign =
//                                       //   commanded direction at ledcWriteTone call)
//   off 96  f32  ledcGotL (steps/sec)  — Snapshot.device.ledcGotL (what
//                                       //   ledcWriteTone returned, same sign as req)
//   off 100 f32  ledcReqR (steps/sec)  — Snapshot.device.ledcReqR
//   off 104 f32  ledcGotR (steps/sec)  — Snapshot.device.ledcGotR
//   off 108 f32  vBus    (V)           — Snapshot.device.vBus (INA226 bus voltage)
//   off 112 f32  iBus    (A)           — Snapshot.device.iBus (INA226 current, signed)
//
// Mapping to Snapshot:
//   - state.theta/thetaDot/xDot: directly from frame.
//   - state.x: integrated locally from xDot using device-clock dt
//     (dev clock = uptime float, monotonic, sub-ms resolution). Position is
//     not telemetered — we synthesize it for the position chart so it isn't
//     pinned at 0.
//   - state.phi: 0 (wheel angle isn't telemetered; renderer wheel-spokes
//     won't spin in device mode — acceptable for S2).
//   - measurement.*: same as state.* (the device IS the sensor in this mode).
//   - ctrl.tau: 0 (steppers don't have torque output; the torque chart will
//     read flat in device mode).
//   - ctrl.angleSet, ctrl.outer.{p,i,d}: from frame.
//   - ctrl.inner.{p,i,d}: 0 (inner-PID decomposition not in frame).
//
// Connection lifecycle is exposed via getStatus() and onStatus() so the UI
// can render a status pill without polling internals.
//
// Reset semantics: for S2, reset() only clears local accumulators (xAccum,
// seq tracker, dev-clock anchor). S3 will additionally send a JSON
// {"type":"reset"} control message over the WS.

export type DeviceStatus =
  | "disconnected"  // never connected, or user clicked Disconnect
  | "connecting"    // WebSocket constructor invoked, awaiting open
  | "connected"     // open + receiving (or about to receive) frames
  | "closed"        // remote/network closed
  | "error";        // WebSocket emitted an error event

export interface DeviceStatusInfo {
  status: DeviceStatus;
  detail: string;     // human-readable: host, frame count, gap count, etc.
  framesTotal: number;
  gaps: number;       // count of seq jumps > 1 (one missed frame = 1 gap)
}

export class DeviceSource implements DataSource {
  private ws: WebSocket | null = null;
  private status: DeviceStatus = "disconnected";
  private statusDetail = "";
  private err: string | null = null;

  // Auto-reconnect bookkeeping. autoReconnect is set true by connect()
  // and back to false by disconnect() — that way an explicit user
  // Disconnect tears the socket down for good, while every other path
  // out of OPEN (network drop, device reboot mid-flight, AP roaming)
  // schedules a backoff retry. nextDelayMs is the schedule itself, in
  // ms, capped so we don't hammer a downed bot but still recover within
  // a few seconds when it comes back. attempts is reset to 0 on every
  // successful open() so a brief outage followed by a long one still
  // starts from the short end of the backoff curve.
  private autoReconnect = false;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private reconnectAttempts = 0;
  private static readonly RECONNECT_BACKOFF_MS = [
    250, 500, 1000, 2000, 4000, 8000,
  ];

  // Stale-feed watchdog. The browser's WebSocket close event only fires
  // after the OS gives up on the underlying TCP connection, which on a
  // silent failure (device powered off mid-session, AP roams away,
  // half-open NAT) can take minutes — during which the UI happily
  // reports "connected" with the panel frozen. Workaround: track wall
  // time of the last message of ANY kind from the coproc and, while
  // status is "connected", force-close the socket if nothing arrives
  // for STALE_FRAME_MS. The existing close handler then runs
  // scheduleReconnect() so we recover the moment the device is back.
  //
  // Why "any message" and not just telemetry: telemetry is now an
  // operator-toggleable stream. A muted telemetry stream is a normal
  // operating mode, not a dead connection. The coproc emits a 1 Hz
  // {type:"ping"} JSON frame piggybacked on the status snapshot
  // arrival (see coproc/src/main.cpp:onStatus) precisely so the
  // watchdog has a signal independent of telemetry.
  //
  // Threshold: pings land at 1 Hz, so 5 s = ~5 missed pings —
  // generous against transient Wi-Fi pauses, still small enough that
  // a real device outage gets caught within a handful of seconds.
  // Watchdog cadence 250 ms → worst-case detection ~5.25 s.
  private static readonly STALE_FRAME_MS = 5000;
  private static readonly WATCHDOG_INTERVAL_MS = 250;
  private lastFrameMs: number | null = null;
  private watchdogTimer: ReturnType<typeof setInterval> | null = null;

  // Latest decoded frame as a Snapshot. Null until the first frame arrives;
  // tick() returns a neutral placeholder in the meantime so the renderer
  // doesn't get NaN.
  private latest: Snapshot | null = null;

  // Local x integration. Device telemeters xDot but not x.
  private xAccum = 0;

  // Device-clock anchor for dt computation (and for guarding against device
  // reboot mid-session — t jumps backward, we just zero the dt).
  private lastDevT: number | null = null;

  // Sequence-gap accounting. seq is u32 on the wire; rolls over but the bot
  // would have to run for ~497 days for that to matter.
  private lastSeq: number | null = null;
  private gaps = 0;
  private framesTotal = 0;

  // Per-tick accumulators (consumed and zeroed by tick()).
  private advancedSinceLastTick = 0;

  // Session-boundary counter. Bumps on every fresh WS open (initial
  // connect and every reconnect alike) and on every detected device-side
  // discontinuity (long feed gap >1 s, or device reboot where t jumps
  // backward). The UI subscribes to changes via tick().sessionEpoch and
  // resets chart buffers / debug history when it changes — without that,
  // a forward jump in snap.tSec orphans every pre-reconnect sample
  // behind LiveChart's rolling 30 s window and the chart looks empty
  // until 30 s of fresh frames have streamed in.
  private sessionEpoch = 0;

  private statusListeners: Array<(info: DeviceStatusInfo) => void> = [];

  // Listeners for {type:"params"} JSON pushed by the firmware on connect
  // and after a loadDefaults RPC. Receives the raw `params` object exactly
  // as serialised by firmware/src/params.cpp::toJson — caller is expected
  // to know the schema (DeviceParams in device_params.ts).
  private paramsListeners: Array<(params: Record<string, any>) => void> = [];

  // Listeners for ack/error JSON. Useful for surfacing "setParam: unknown
  // path" or similar in the connection status pill.
  private ackListeners: Array<(of: string) => void> = [];
  private errorListeners: Array<(msg: string) => void> = [];

  constructor(private host: string) {}

  setHost(h: string): void { this.host = h; }
  getHost(): string { return this.host; }

  connect(): void {
    // User-initiated connect: arm auto-reconnect. From here on, only
    // disconnect() can disarm it.
    this.autoReconnect = true;
    this.reconnectAttempts = 0;
    this.openSocket();
  }

  disconnect(): void {
    // Disarm auto-reconnect first so the close-event handler doesn't
    // schedule a fresh attempt behind our back.
    this.autoReconnect = false;
    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    this.stopWatchdog();
    if (this.ws) {
      try { this.ws.close(); } catch { /* ignore */ }
      this.ws = null;
    }
    this.setStatus("disconnected", "");
  }

  // Open a WebSocket to the configured host and wire up listeners. Used
  // both for the initial user-driven connect() and for backoff retries
  // scheduled by scheduleReconnect(). Does NOT touch autoReconnect or
  // reconnectAttempts — those are owned by the caller.
  private openSocket(): void {
    if (this.ws) {
      try { this.ws.close(); } catch { /* ignore */ }
      this.ws = null;
    }
    const url = `ws://${this.host}/ws`;
    this.setStatus("connecting", url);
    try {
      this.ws = new WebSocket(url);
    } catch (e) {
      this.setStatus("error", String(e));
      this.scheduleReconnect();
      return;
    }
    this.ws.binaryType = "arraybuffer";
    this.ws.addEventListener("open", () => {
      // Successful open resets the backoff so a transient drop after
      // a long stable session doesn't punish us with the 8 s cap.
      this.reconnectAttempts = 0;
      // Reconnect deliberately does NOT bump sessionEpoch — the device
      // clock is contiguous across a brief WS drop (firmware never
      // rebooted) so the chart buffers from before the drop are still
      // meaningful. The only sessionEpoch bump that survives is the
      // device-clock-rollback path in onMessage(), which is the real
      // session boundary (firmware reboot or >1 s tSec discontinuity).
      // Charts span the disconnected window via Chart.js's spanGaps
      // setting; an actual gap appears as a line stretched flat across
      // the missing time, which is the truth of what we have.
      //
      // Seed the stale-feed watchdog at open time (not on first frame)
      // so the post-open silence window is bounded by STALE_FRAME_MS
      // rather than indefinite. If the device accepts the WS upgrade
      // but never produces a frame (firmware crash between accept and
      // the first telemetry tick), we still recover.
      this.lastFrameMs = performance.now();
      this.startWatchdog();
      this.setStatus("connected", `${this.host} (waiting for frames…)`);
    });
    this.ws.addEventListener("error", () => {
      // The browser refuses to expose useful detail here for security
      // reasons; the close event that follows usually carries a code.
      this.setStatus("error", "WebSocket error");
    });
    this.ws.addEventListener("close", (e) => {
      this.ws = null;
      this.stopWatchdog();
      const codeStr = `code=${e.code}${e.reason ? ` ${e.reason}` : ""}`;
      if (this.autoReconnect) {
        // Schedule the retry first so getStatus() reflects "reconnecting"
        // rather than a bare "closed" line.
        this.scheduleReconnect(codeStr);
      } else {
        this.setStatus("closed", codeStr);
      }
    });
    this.ws.addEventListener("message", (ev) => this.onMessage(ev));
  }

  private scheduleReconnect(closeReason?: string): void {
    if (!this.autoReconnect) return;
    if (this.reconnectTimer !== null) return; // already scheduled

    const tbl = DeviceSource.RECONNECT_BACKOFF_MS;
    const delay = tbl[Math.min(this.reconnectAttempts, tbl.length - 1)];
    this.reconnectAttempts++;

    const reasonSuffix = closeReason ? ` after ${closeReason}` : "";
    this.setStatus(
      "closed",
      `reconnecting in ${delay} ms${reasonSuffix} (attempt ${this.reconnectAttempts})`
    );

    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      if (!this.autoReconnect) return;
      this.openSocket();
    }, delay);
  }

  // Stale-feed watchdog. See field comments above for rationale and
  // threshold tuning. Only fires while status is "connected" so we
  // don't fight with reconnect scheduling or the post-disconnect grace
  // window. Force-closing the ws routes through the existing close
  // handler, which sets status and calls scheduleReconnect() — so the
  // recovery path is identical to a real network drop.
  private startWatchdog(): void {
    if (this.watchdogTimer !== null) return;
    this.watchdogTimer = setInterval(() => {
      if (this.status !== "connected") return;
      if (this.lastFrameMs === null) return;
      const since = performance.now() - this.lastFrameMs;
      if (since < DeviceSource.STALE_FRAME_MS) return;
      // Surface the reason in the pill before the close event
      // overwrites it with the reconnect-attempt detail. Then close;
      // the close handler will schedule the retry.
      this.setStatus("error", `no frames for ${Math.round(since)} ms — reconnecting`);
      if (this.ws) {
        try { this.ws.close(); } catch { /* ignore */ }
      }
    }, DeviceSource.WATCHDOG_INTERVAL_MS);
  }

  private stopWatchdog(): void {
    if (this.watchdogTimer !== null) {
      clearInterval(this.watchdogTimer);
      this.watchdogTimer = null;
    }
    this.lastFrameMs = null;
  }

  reset(): void {
    // S2: local-only reset (clears integrated x and seq/dt tracking so a
    // fresh physics chart history doesn't have a discontinuity from the
    // pre-reset accumulator). S3 will additionally send a JSON
    // {"type":"reset"} so the device clears its own controller integrators.
    this.xAccum = 0;
    this.lastDevT = null;
    this.lastSeq = null;
    this.gaps = 0;
    this.framesTotal = 0;
    this.advancedSinceLastTick = 0;
  }

  consumeError(): string | null {
    const e = this.err; this.err = null; return e;
  }


  tick(_advanceSec: number): TickResult {
    // The device sets its own pace; advanceSec is ignored. We report how
    // much device-clock time was consumed by frames received since the
    // previous tick so charts/debug-history gate their per-frame pushes.
    const advanced = this.advancedSinceLastTick;
    this.advancedSinceLastTick = 0;
    return {
      snap: this.latest ?? this.placeholder(),
      advancedSec: advanced,
      sessionEpoch: this.sessionEpoch,
    };
  }

  getStatus(): DeviceStatusInfo {
    return {
      status: this.status,
      detail: this.statusDetail,
      framesTotal: this.framesTotal,
      gaps: this.gaps,
    };
  }

  onStatus(cb: (info: DeviceStatusInfo) => void): void {
    this.statusListeners.push(cb);
  }

  onParams(cb: (params: Record<string, any>) => void): void {
    this.paramsListeners.push(cb);
  }

  onAck(cb: (of: string) => void): void { this.ackListeners.push(cb); }
  onError(cb: (msg: string) => void): void { this.errorListeners.push(cb); }

  // Send a JSON control message (setParam, setTarget, enableMotors, reset,
  // saveParams, loadDefaults, getParams). Returns true if the WS was open;
  // false silently drops (caller can poll getStatus() if it cares).
  send(msg: object): boolean {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return false;
    try {
      this.ws.send(JSON.stringify(msg));
      return true;
    } catch {
      return false;
    }
  }

  private setStatus(s: DeviceStatus, detail: string): void {
    this.status = s;
    this.statusDetail = detail;
    const info = this.getStatus();
    for (const cb of this.statusListeners) cb(info);
  }

  // Neutral snapshot used before the first frame arrives (or while
  // disconnected). All zeros so the renderer parks the bot upright at
  // origin and charts plot a clean baseline rather than NaN.
  private placeholder(): Snapshot {
    return {
      tSec: 0,
      state: { x: 0, xDot: 0, theta: 0, thetaDot: 0, phi: 0 },
      measurement: { theta: 0, thetaDot: 0, xDot: 0 },
      ctrl: { tau: 0, angleSet: 0, inner: { p: 0, i: 0, d: 0 }, outer: { p: 0, i: 0, d: 0 } },
    };
  }

  private onMessage(ev: MessageEvent): void {
    // Watchdog timestamp. Any message from the coproc — telemetry,
    // ping, params, ack, error — counts as "the link is alive". This
    // used to be stamped only inside the binary-frame branch (after
    // the magic check), but with telemetry now optional the coproc's
    // 1 Hz {type:"ping"} JSON is the only guaranteed signal. Stamping
    // here covers both cases uniformly.
    this.lastFrameMs = performance.now();

    if (typeof ev.data === "string") {
      // Firmware-side JSON: {type:"params", params:{...}} pushed on
      // every connect (and after loadDefaults), {type:"ack", of:"..."}
      // for accepted commands, {type:"error", msg:"..."} for rejects,
      // {type:"ping", upMs:...} from coproc at 1 Hz as the watchdog
      // signal. Anything we don't recognise we ignore quietly.
      try {
        const m = JSON.parse(ev.data);
        if (m && typeof m === "object") {
          if (m.type === "params" && m.params && typeof m.params === "object") {
            for (const cb of this.paramsListeners) cb(m.params);
          } else if (m.type === "ack" && typeof m.of === "string") {
            for (const cb of this.ackListeners) cb(m.of);
          } else if (m.type === "error" && typeof m.msg === "string") {
            for (const cb of this.errorListeners) cb(m.msg);
          }
          // "ping" needs no further handling — the watchdog stamp at
          // the top of onMessage already did the job.
        }
      } catch {
        // not JSON; not our concern
      }
      return;
    }
    const buf = ev.data instanceof ArrayBuffer ? ev.data : null;
    if (!buf || buf.byteLength !== 116) return;

    const dv = new DataView(buf);
    const magic = dv.getUint32(0, true);
    if (magic !== 0xB0B0B0B0) return; // not our frame; ignore quietly

    const seq      = dv.getUint32(4, true);
    const t        = dv.getFloat32(8, true);
    const theta    = dv.getFloat32(12, true);
    const thetaDot = dv.getFloat32(16, true);
    const xDot     = dv.getFloat32(20, true);
    const thetaSet = dv.getFloat32(24, true);
    const vWheelCmd = dv.getFloat32(28, true);
    const oP       = dv.getFloat32(32, true);
    const oI       = dv.getFloat32(36, true);
    const oD       = dv.getFloat32(40, true);
    const vBat     = dv.getFloat32(44, true);
    const wheelActualMps = dv.getFloat32(48, true);
    const accelX   = dv.getFloat32(52, true);
    const gyroZ    = dv.getFloat32(56, true);
    const flags    = dv.getUint16(60, true);
    /* reserved */   dv.getUint16(62, true);
    const targetV  = dv.getFloat32(64, true);
    const vWheelCmdL      = dv.getFloat32(68, true);
    const vWheelCmdR      = dv.getFloat32(72, true);
    const wheelActualMpsL = dv.getFloat32(76, true);
    const wheelActualMpsR = dv.getFloat32(80, true);
    const targetTurn      = dv.getFloat32(84, true);
    const targetTurnUsed  = dv.getFloat32(88, true);
    const ledcReqL        = dv.getFloat32(92, true);
    const ledcGotL        = dv.getFloat32(96, true);
    const ledcReqR        = dv.getFloat32(100, true);
    const ledcGotR        = dv.getFloat32(104, true);
    const vBus            = dv.getFloat32(108, true);
    const iBus            = dv.getFloat32(112, true);

    // (Watchdog timestamp is set at the top of onMessage on every
    // incoming message, so it doesn't depend on telemetry being on.)

    if (this.lastSeq !== null && seq !== ((this.lastSeq + 1) >>> 0)) {
      this.gaps++;
    }
    this.lastSeq = seq;

    // Compute device-clock dt. Two anomalies to defend against:
    //   1) dt < 0 — device clock ran backwards, i.e. the firmware
    //      rebooted mid-session. tSec is no longer contiguous with what
    //      we plotted, so this is a real session boundary: bump
    //      sessionEpoch and let the UI wipe its rolling buffers.
    //   2) dt > 1.0 — long silent gap, almost always a WS reconnect
    //      (backoff curve runs up to 8 s). The device clock is still
    //      contiguous — the firmware never rebooted, it's just that we
    //      didn't receive frames for a while. We zero dt to avoid
    //      integrating xDot across the silent window, but we DO NOT
    //      bump sessionEpoch — charts keep their pre-disconnect history
    //      and the gap renders as a break in the line (spanGaps in
    //      charts.ts kicks in at 100 ms).
    let dt = 0;
    if (this.lastDevT !== null) {
      dt = t - this.lastDevT;
      if (dt < 0) {
        dt = 0;
        this.sessionEpoch++;
      } else if (dt > 1.0) {
        dt = 0;
      }
    }
    this.lastDevT = t;

    this.xAccum += xDot * dt;
    this.advancedSinceLastTick += dt;
    this.framesTotal++;

    // Refresh status detail occasionally so the pill shows live frame
    // count without a per-frame DOM update.
    if (this.framesTotal === 1 || this.framesTotal % 100 === 0) {
      this.statusDetail = `${this.host} · ${this.framesTotal} frames` +
        (this.gaps > 0 ? ` · ${this.gaps} gaps` : "");
      const info = this.getStatus();
      for (const cb of this.statusListeners) cb(info);
    }

    this.latest = {
      tSec: t,
      state: {
        x: this.xAccum,
        xDot,
        theta,
        thetaDot,
        phi: 0,
      },
      measurement: { theta, thetaDot, xDot },
      ctrl: {
        tau: 0,
        angleSet: thetaSet,
        inner: { p: 0, i: 0, d: 0 },
        outer: { p: oP, i: oI, d: oD },
      },
      device: { vBat, flags, vWheelCmd, wheelActualMps, accelX, gyroZ, targetV,
                vWheelCmdL, vWheelCmdR, wheelActualMpsL, wheelActualMpsR,
                targetTurn, targetTurnUsed,
                ledcReqL, ledcGotL, ledcReqR, ledcGotR,
                vBus, iBus },
    };
  }
}

// Decode the firmware status-flag bitfield (shared::Flag, see
// firmware/src/shared_state.h:28-32) into a short pipe-separated label
// for the HUD. Returns "" if no flags are set so the HUD can render a
// neutral pill rather than an empty "[]".
export function decodeDeviceFlags(flags: number): string {
  const parts: string[] = [];
  if (flags & 0x0001) parts.push("FALLEN");
  if (flags & 0x0002) parts.push("LOW_BAT");
  if (flags & 0x0004) parts.push("MOTORS");
  if (flags & 0x0008) parts.push("PS");
  return parts.join("|");
}
