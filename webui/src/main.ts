// Main entry point for the bot web UI. Owns the rAF loop and DOM
// wiring; the live telemetry feed comes from a DeviceSource (WebSocket
// client to the on-bot firmware). Each frame drains whatever telemetry
// frames arrived and returns a Snapshot the charts / HUD read from.

import { ManualPrefs, defaultManualPrefs } from "./types";
import { LiveChart } from "./charts";
import { buildDeviceUI, applyDeviceSnapshot, SyncMap } from "./ui";
import { decodeDeviceFlags, DeviceSource, DeviceStatusInfo, Snapshot } from "./datasource";
import { defaultDeviceParams, DeviceParams } from "./device_params";
import { AudioBridge } from "./audio";

// Operator manual-control preferences (arrow-key velocity/turn
// magnitudes). Browser-owned; persisted to localStorage. Everything
// else the UI tunes lives in the firmware (DeviceParams).
const MANUAL_KEY = "balancing-robot.manualPrefs";
function loadManualPrefs(): ManualPrefs {
  try {
    const raw = localStorage.getItem(MANUAL_KEY);
    if (raw) {
      const p = JSON.parse(raw);
      const c = p?.control ?? {};
      return {
        control: {
          manualVelocity: Number.isFinite(c.manualVelocity)
            ? c.manualVelocity : defaultManualPrefs.control.manualVelocity,
          manualTurn: Number.isFinite(c.manualTurn)
            ? c.manualTurn : defaultManualPrefs.control.manualTurn,
        },
      };
    }
  } catch { /* fall through to defaults */ }
  return { control: { ...defaultManualPrefs.control } };
}
let manualSaveTimer: ReturnType<typeof setTimeout> | null = null;
function saveManualPrefsDebounced(): void {
  if (manualSaveTimer !== null) clearTimeout(manualSaveTimer);
  manualSaveTimer = setTimeout(() => {
    manualSaveTimer = null;
    localStorage.setItem(MANUAL_KEY, JSON.stringify(manualPrefs));
  }, 500);
}
const manualPrefs: ManualPrefs = loadManualPrefs();

let running = true;

// Device host is persisted so the user doesn't retype it on every
// reload. Default to the mDNS name; on a network where mDNS doesn't
// resolve the user can paste an IP.
const HOST_KEY = "balancing-robot.deviceHost";
const initialHost = localStorage.getItem(HOST_KEY) || "balancebot.local";
const deviceSource = new DeviceSource(initialHost);

// Snapshot from the most recent tick. Initialized from a zero-advance
// tick so the first frame can populate the HUD before any telemetry
// has arrived.
let snap: Snapshot = deviceSource.tick(0).snap;

const angleChart = new LiveChart(document.getElementById("chart-angle") as HTMLCanvasElement, {
  title: "Angle",
  yLabel: "deg",
  unit: "°",
  windowSec: 30,
  series: [
    { label: "θ (raw)", stroke: "#4ea1ff77" },
    // Effective angle: raw IMU θ − thetaTrim, what the controller
    // actually balances around.
    { label: "θ (effective)", stroke: "#4ea1ff" },
    { label: "θ_set", stroke: "#ffb454" },
  ],
});
const ratesChart = new LiveChart(document.getElementById("chart-rates") as HTMLCanvasElement, {
  title: "Rates",
  yLabel: "°/s",
  unit: " °/s",
  windowSec: 30,
  yAxes: [{ id: "yLin", label: "m/s", position: "right", unit: " m/s" }],
  series: [
    { label: "θ̇", stroke: "#6ce5a5" },
    // Yaw-axis gyro, device mode only (sim plant is 1-DoF and has no
    // yaw). Used to diagnose whether thetaDot wobbles during a turn are
    // a real pitch motion or yaw-rate bleeding into the gy channel via
    // mounting misalignment / cross-axis sensitivity. If ψ̇ tracks θ̇
    // when the bot is held off the floor and rotated about vertical,
    // there's cross-coupling.
    { label: "ψ̇ (yaw)", stroke: "#ffd166" },
    // { label: "ẋ", stroke: "#ff5c6c", yAxis: "yLin" },
  ],
});
const posChart = new LiveChart(document.getElementById("chart-pos") as HTMLCanvasElement, {
  title: "Position",
  yLabel: "m",
  unit: " m",
  windowSec: 30,
  series: [{ label: "x", stroke: "#9aa5b4" }],
});
const pidOuterChart = new LiveChart(document.getElementById("chart-pid-outer") as HTMLCanvasElement, {
  title: "Outer PID terms",
  yLabel: "rad (angleSet)",
  unit: " rad",
  windowSec: 30,
  // Right-hand axis for the velocity setpoint, in m/s. Sharing the rad
  // axis would either squash P/I/D against zero (m/s ≫ rad in typical
  // operation) or stretch them off the top — different units, different
  // axis. tgtV / tgtTurn come straight from the device telemetry frame
  // (shared::g.targetV / .targetTurn / .targetTurnUsed, all in m/s); in
  // sim mode they track Snapshot.device?.targetV/Turn/TurnUsed which
  // are undefined, so the series stay flat at NaN/skipped.
  // tgtTurn cmd is the raw operator request (pre-filter, pre-clamp).
  // tgtTurn used is the post-targetTurnAlpha + post-±vMaxTurn value
  // the controller actually summed into vL/vR — what differs from cmd
  // when the ramp filter is engaged or the request hits the clamp. Two
  // similar-but-distinct hues so the eye reads them as a related pair
  // (same family as tgtV's amber but in the purple/violet range so they
  // don't fight tgtV for the right-axis ink).
  yAxes: [{ id: "yMps", label: "m/s", position: "right", unit: " m/s" }],
  series: [
    { label: "P", stroke: "#4ea1ff" },
    { label: "I", stroke: "#6ce5a5" },
    { label: "D", stroke: "#ff5c6c" },
    { label: "tgtV",         stroke: "#ffd166", yAxis: "yMps" },
    { label: "tgtTurn cmd",  stroke: "#c08cff", yAxis: "yMps" },
    { label: "tgtTurn used", stroke: "#7a4fc4", yAxis: "yMps" },
  ],
});
// Device-only: the firmware's inner law isn't a PID; it's a linear blend
// `vWheel = velFF·targetV + Kth·θerr + KthDot·θ̇` (controller.cpp:140).
// We chart the three components computed from telemetry + the live
// DeviceParams, plus vWheelCmd as a sum sanity-check. Hidden in sim mode
// (which has the inner-PID chart instead).
const innerDevChart = new LiveChart(document.getElementById("chart-inner-dev") as HTMLCanvasElement, {
  title: "Inner law (θ → vWheel)",
  yLabel: "m/s",
  unit: " m/s",
  windowSec: 30,
  series: [
    { label: "velFF·targetV",    stroke: "#9aa5b4" },
    { label: "Kth·θerr",         stroke: "#4ea1ff" },
    { label: "KthDot·θ̇",         stroke: "#ff5c6c" },
  ],
});
// Separate chart for the actuation tracking quartet: per-wheel command
// (vWheelCmdL/R, post-turn-split + post-saturation) vs per-wheel actual
// (wheelActualMpsL/R, sign-corrected for mounting invert). Keeping them
// on their own y-axis (free of the term-by-term decomposition above)
// makes saturation/stall episodes obvious — and per-wheel exposure
// catches asymmetric problems (one wheel saturating during a turn, one
// wheel stalling while the other doesn't) that the L+R average hides.
// Click the legend labels to hide individual lines (e.g. mute both R
// to focus on L cmd vs actual).
const wheelDevChart = new LiveChart(document.getElementById("chart-wheel-dev") as HTMLCanvasElement, {
  title: "Wheel cmd vs actual (L/R)",
  yLabel: "m/s",
  unit: " m/s",
  windowSec: 30,
  series: [
    // Cmd in warm hues, actual in cool hues. Within each pair, L is the
    // brighter/saturated tone and R is the muted/desaturated one — same
    // hue family so eye can pair cmd↔actual at a glance.
    { label: "vWheelCmd L",   stroke: "#ffb454" },
    { label: "vWheelCmd R",   stroke: "#c98438" },
    { label: "wheelActual L", stroke: "#7be07b" },
    { label: "wheelActual R", stroke: "#4ea35a" },
  ],
});
// LEDC step-pulse diagnostic: req vs got per channel, in the LEDC's
// native units (steps/sec = Hz at the STEP pin), NOT m/s. Surfaced
// separately from the m/s cmd/actual chart above because the whole
// point of these series is to show the LEDC peripheral's
// frequency-rounding behaviour — converting to m/s would smooth that
// away. Today (LEDC_RES_BITS=8) req and got should track exactly;
// any visible divergence in this chart means an LEDC quantisation
// regression and is a smoking gun, not a tuning concern.
//
// Hue pairing matches wheelDevChart: req in warm tones (req = "what we
// asked for", same family as cmd), got in cool tones (got = "what we
// got back", same family as actual). Click legend labels to mute lines.
const ledcDevChart = new LiveChart(document.getElementById("chart-ledc-dev") as HTMLCanvasElement, {
  title: "LEDC req vs got (L/R)",
  yLabel: "steps/sec",
  unit: " Hz",
  windowSec: 30,
  series: [
    { label: "ledcReq L", stroke: "#ffb454" },
    { label: "ledcReq R", stroke: "#c98438" },
    { label: "ledcGot L", stroke: "#7be07b" },
    { label: "ledcGot R", stroke: "#4ea35a" },
  ],
});
// Compact "is the bot doing the right thing?" overview. Mixes the two
// natural units (degrees for the angle frame, m/s for the wheel frame)
// on dual y-axes so the eye can correlate "what the controller saw"
// (θeff vs θset, Kth·err) with "what it asked the wheels to do" and
// "what the wheels actually did" without flipping between three charts.
//
// θeff  (deg, left)  — raw IMU − thetaTrim, the angle the controller sees
// θset  (deg, left)  — outer-loop angle setpoint
// Kth·err (m/s, rt)  — dominant inner-law term (controller.cpp:140)
// vWheelCmd (m/s)    — full inner-law output (sum of terms + saturation)
// vWheelAct (m/s)    — wheelActualMps from FastAccelStepper
const summaryDevChart = new LiveChart(document.getElementById("chart-summary-dev") as HTMLCanvasElement, {
  title: "Control summary",
  yLabel: "deg",
  unit: "°",
  windowSec: 30,
  yAxes: [{ id: "yMps", label: "m/s", position: "right", unit: " m/s" }],
  series: [
    { label: "θ eff",     stroke: "#4ea1ff" },
    { label: "θ set",     stroke: "#ffb454" },
    { label: "Kth·err",   stroke: "#c08cff",  yAxis: "yMps" },
    { label: "vWheelCmd", stroke: "#ffd166",  yAxis: "yMps" },
    { label: "vWheelAct", stroke: "#7be07b",  yAxis: "yMps" },
  ],
});

// INA226 power monitor — bus voltage on the left axis (volts), current
// on the right axis (amps). Charting them on a single chart with
// dual axes lets the eye correlate voltage sag with current spikes
// (e.g. stepper peaks dragging the pack down). Device-mode only.
const powerDevChart = new LiveChart(document.getElementById("chart-power-dev") as HTMLCanvasElement, {
  title: "Power (INA226)",
  yLabel: "V",
  unit: " V",
  windowSec: 30,
  yAxes: [{ id: "yA", label: "A", position: "right", unit: " A" }],
  series: [
    { label: "vBus", stroke: "#4ea35a" },
    { label: "iBus", stroke: "#ffb454", yAxis: "yA" },
  ],
});

const hud = document.getElementById("hud") as HTMLElement;

const btnPlayPause = document.getElementById("btn-playpause") as HTMLButtonElement;
function updatePlayPauseLabel(): void {
  btnPlayPause.textContent = running ? "Pause" : "Play";
}
function togglePlayPause(): void {
  running = !running;
  updatePlayPauseLabel();
  // If a DeviceSource session boundary (reconnect / feed gap) fired
  // while we were paused, we deferred the chart reset so the operator
  // could keep investigating the frozen view. Now that we're resuming,
  // the chart's lastT is stale and the next push would trim everything
  // as "older than lastT − windowSec"; perform the deferred reset here
  // so resume starts from a clean buffer instead of an empty one.
  if (running && pendingSessionReset) {
    pendingSessionReset = false;
    resetView();
  }
}
btnPlayPause.addEventListener("click", togglePlayPause);
updatePlayPauseLabel();
window.addEventListener("keydown", (e) => {
  if (e.code !== "Space") return;
  const t = e.target as HTMLElement | null;
  const tag = t?.tagName;
  if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || t?.isContentEditable) return;
  e.preventDefault();
  togglePlayPause();
});

// Manual control via arrow keys.
//   ↑/↓  → targetVelocity (forward / back).
//   ←/→  → targetTurn (steering differential, in m/s, added to left and
//          subtracted from right wheel: positive = bot turns right).
// Holding a key sets the value to ±manualVelocity / ±manualTurn; releasing
// returns it to 0. We send {type:"setTarget", v} / {type:"setTurn", v}
// WS messages so the firmware owns the live setpoints (clamped to
// vMaxCart / vMaxTurn on receive). lastDeviceTarget{V,Turn} are cached
// so the inner-law chart can reconstruct velFF·targetV.
const arrowsHeld = { up: false, down: false, left: false, right: false };
let lastDeviceTargetV = 0;
let lastDeviceTargetTurn = 0;
function applyArrowVelocity(): void {
  const m = manualPrefs.control.manualVelocity;
  let v = 0;
  if (arrowsHeld.up) v += m;
  if (arrowsHeld.down) v -= m;
  lastDeviceTargetV = v;
  deviceSource.send({ type: "setTarget", v });
}
function applyArrowTurn(): void {
  const m = manualPrefs.control.manualTurn;
  let v = 0;
  if (arrowsHeld.right) v += m;
  if (arrowsHeld.left) v -= m;
  lastDeviceTargetTurn = v;
  deviceSource.send({ type: "setTurn", v });
}
window.addEventListener("keydown", (e) => {
  const isV = e.code === "ArrowUp" || e.code === "ArrowDown";
  const isT = e.code === "ArrowLeft" || e.code === "ArrowRight";
  if (!isV && !isT) return;
  const t = e.target as HTMLElement | null;
  const tag = t?.tagName;
  if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || t?.isContentEditable) return;
  e.preventDefault();
  if (e.repeat) return;
  if (e.code === "ArrowUp") arrowsHeld.up = true;
  else if (e.code === "ArrowDown") arrowsHeld.down = true;
  else if (e.code === "ArrowLeft") arrowsHeld.left = true;
  else if (e.code === "ArrowRight") arrowsHeld.right = true;
  if (isV) applyArrowVelocity();
  else applyArrowTurn();
});
window.addEventListener("keyup", (e) => {
  const isV = e.code === "ArrowUp" || e.code === "ArrowDown";
  const isT = e.code === "ArrowLeft" || e.code === "ArrowRight";
  if (!isV && !isT) return;
  if (e.code === "ArrowUp") arrowsHeld.up = false;
  else if (e.code === "ArrowDown") arrowsHeld.down = false;
  else if (e.code === "ArrowLeft") arrowsHeld.left = false;
  else if (e.code === "ArrowRight") arrowsHeld.right = false;
  if (isV) applyArrowVelocity();
  else applyArrowTurn();
});
// Safety: if focus leaves the window while an arrow is held, the keyup may be lost.
window.addEventListener("blur", () => {
  arrowsHeld.up = false;
  arrowsHeld.down = false;
  arrowsHeld.left = false;
  arrowsHeld.right = false;
  applyArrowVelocity();
  applyArrowTurn();
});
(document.getElementById("btn-reset") as HTMLButtonElement).addEventListener("click", () => {
  // Tell the firmware to clear its own integrators (controller I-term,
  // x integrator, etc). resetView() additionally wipes chart history
  // so the discontinuity isn't drawn as a long straight line.
  deviceSource.send({ type: "reset" });
  resetView();
  // Reset always un-pauses the UI. The common workflow is "something
  // went wrong → I pause to inspect → I press Reset to start fresh";
  // leaving the UI paused after Reset means the user has to also click
  // Play, which is friction with no upside.
  if (!running) {
    running = true;
    updatePlayPauseLabel();
  }
});

// Debug snapshot for diagnosing controller issues. Copies a compact JSON blob
// with the parameter set, derived quantities, current state, and a rolling
// history of state + controller outputs at the full source rate (every frame
// pushed by the active source — no decimation). Downsampling was removed
// because it occasionally hid the very transients we copy the dump to
// diagnose (a fast spike that landed between 20 ms slots disappeared); the
// JSON is still small enough at 60 Hz × 30 s.
const DEBUG_HISTORY_SEC = 30; // matches chart windowSec; trimmed by time so
                              // the buffer can't grow without bound during
                              // a long session.
interface DebugSample {
  t: number;
  theta: number; thetaDot: number;
  x: number; xDot: number;
  measTheta: number;
  // Sim-only torque-domain fields. Hardcoded zero on device — the firmware
  // commands wheel velocity (m/s), not torque, and never measures torque.
  tau: number; angleSet: number;
  pTerm: number; iTerm: number; dTerm: number;
  // Outer loop is populated in both modes (sim PID outputs in radians of
  // angleSet contribution; device firmware sends the same three values).
  outerP: number; outerI: number; outerD: number;
  // Device-only fields. Undefined in sim mode.
  vWheelCmd?: number;       // commanded common-mode wheel velocity [m/s] (pre-split)
  wheelActualMps?: number;  // L+R average actual step rate [m/s]; on LEDC == vWheelCmd
  vWheelCmdL?: number;      // per-wheel cmd post-split, post-saturation [m/s]
  vWheelCmdR?: number;
  wheelActualMpsL?: number; // per-wheel actual [m/s], cart-frame (mounting invert undone)
  wheelActualMpsR?: number;
  accelX?: number;          // body-frame X accel [g], raw (diagnostic)
  vBat?: number;            // battery [V]
  flags?: number;           // raw bitfield (decoded textually only at top of dump)
  thetaEff?: number;        // raw theta - thetaTrim (what the controller actually sees)
  // LEDC step-pulse diagnostic per channel: signed Hz at the STEP pin
  // (sign reflects commanded direction at the ledcWriteTone call).
  // Captured per sample so the dump exposes any req↔got divergence or
  // got==0 transient anywhere in the rolling window. Undefined in sim
  // mode.
  ledcReqL?: number; ledcGotL?: number;
  ledcReqR?: number; ledcGotR?: number;
}
const debugHistory: DebugSample[] = [];

// Tracks the previous device-telemetry flags byte across frames so we can
// edge-detect transitions. Bit 0 = FALLEN; we want to auto-pause on the
// rising edge so the operator can post-mortem the moment without racing
// the rolling chart window. Undefined until the first device frame arrives.
let prevDeviceFlags: number | undefined;

function addDebugSample(sample: () => DebugSample): void {
  const s = sample();
  debugHistory.push(s);
  while (debugHistory.length && s.t - debugHistory[0].t > DEBUG_HISTORY_SEC) {
    debugHistory.shift();
  }
}

// Round to N significant decimals to keep JSON small while preserving readable precision.
function r(v: number, decimals = 4): number {
  if (!Number.isFinite(v)) return v;
  const f = Math.pow(10, decimals);
  return Math.round(v * f) / f;
}

(document.getElementById("btn-debug") as HTMLButtonElement).addEventListener("click", async () => {
  // Build a mode-specific debug blob. The simulator and the firmware run
  // very different control laws (torque-actuated PID vs velocity-cascade
  // with Kth/KthDot/velFF) and very different parameter schemas, so a
  // shared payload was misleading: in device mode the previous version
  // dumped the simulator's params and zero-filled tau/inner.{p,i,d}
  // because the firmware doesn't even compute torque. Now the two paths
  // emit disjoint, self-describing JSON.

  // Ask the operator how many seconds of recent history to include. The
  // default (5 s) covers the typical "what just happened?" post-mortem
  // window; bump it for a longer crash trail. Cancel aborts the copy
  // entirely; an empty value or a number larger than the buffer means
  // "everything we have". Trimming to a tight tail keeps the dump small
  // and focused — previously every copy dragged in 30 s of pre-event
  // noise.
  const oldest = debugHistory.length ? debugHistory[0].t : 0;
  const newest = debugHistory.length ? debugHistory[debugHistory.length - 1].t : 0;
  const span = newest - oldest;
  const ans = window.prompt(
    `Save last N seconds of history.\n` +
    `Buffer: ${span.toFixed(2)} s (${debugHistory.length} samples, ` +
    `${oldest.toFixed(2)} … ${newest.toFixed(2)}).\n` +
    `Empty = all.`,
    "5"
  );
  if (ans === null) return; // user cancelled
  const trimmed = ans.trim();
  const lastSec = trimmed === "" ? Infinity : parseFloat(trimmed);
  if (Number.isNaN(lastSec) || lastSec < 0) {
    alert(`Invalid duration: "${ans}"`);
    return;
  }
  const startT = newest - lastSec;
  const historyWindow = debugHistory.filter((s) => s.t >= startT);

  let debug: Record<string, unknown>;

  {
    // Device snapshot. snap.ctrl.tau and snap.ctrl.inner.* are NOT
    // populated by the firmware; we omit them entirely rather than
    // emit zeros that look like real measurements. Inner-law term
    // decomposition (ff, pTh, dTh) is recomputed client-side from
    // deviceParams since the firmware sends only the sum vWheelCmd.
    const thetaEff = snap.state.theta - deviceParams.thetaTrim;
    const thetaErr = thetaEff - snap.ctrl.angleSet;
    const ff   = deviceParams.velFF  * lastDeviceTargetV;
    const pTh  = deviceParams.Kth    * thetaErr;
    const dTh  = deviceParams.KthDot * snap.state.thetaDot;
    const flagsLabel = snap.device ? (decodeDeviceFlags(snap.device.flags) || "—") : "n/a";

    debug = {
      source: "device",
      note: "Device firmware actuates wheel velocity (m/s), not torque. There is no tau. Inner law: vWheelCmd = velFF*targetV + Kth*thetaErr + KthDot*thetaDot. thetaEff = rawTheta - thetaTrim is what the controller sees.",
      t: r(snap.tSec, 3),
      running,
      lastDeviceTargetV: r(lastDeviceTargetV, 4),
      lastDeviceTargetTurn: r(lastDeviceTargetTurn, 4),
      // Telemetered raw values from shared::g (pre-targetV/TurnAlpha
      // smoothing). May differ from lastDevice* above if a setParam or
      // RC source recently overrode joystick state — these are what the
      // controller actually saw this tick.
      telemTargetV:    snap.device ? r(snap.device.targetV, 4) : null,
      telemTargetTurn: snap.device ? r(snap.device.targetTurn, 4) : null,
      // Post-targetTurnAlpha + post-±vMaxTurn clamp value the firmware
      // controller actually summed into vL/vR THIS tick. Diverges from
      // telemTargetTurn while the ramp filter is settling or the
      // request hits the clamp; equals it once the filter has converged
      // and the request is below the clamp.
      telemTargetTurnUsed: snap.device ? r(snap.device.targetTurnUsed, 4) : null,
      params: deviceParams,
      state: {
        thetaRaw: r(snap.state.theta, 5),
        thetaEff: r(thetaEff, 5),
        thetaTrim: r(deviceParams.thetaTrim, 5),
        thetaDot: r(snap.state.thetaDot, 4),
        accelX:    snap.device ? r(snap.device.accelX, 4) : null,
        x: r(snap.state.x, 4), xDot: r(snap.state.xDot, 4),
      },
      lastCtrl: {
        angleSet: r(snap.ctrl.angleSet, 5),
        thetaErr: r(thetaErr, 5),
        // Outer-loop terms are real: firmware telemeters them.
        oP: r(snap.ctrl.outer.p, 4),
        oI: r(snap.ctrl.outer.i, 4),
        oD: r(snap.ctrl.outer.d, 4),
        // Inner-law decomposition is reconstructed from deviceParams +
        // measured state. Their sum should equal vWheelCmd modulo
        // saturation/aMaxWheel limiting in motors.cpp.
        innerFF:    r(ff, 4),
        innerKth:   r(pTh, 4),
        innerKthDot: r(dTh, 4),
        vWheelCmd:     snap.device ? r(snap.device.vWheelCmd, 4) : null,
        wheelActualMps: snap.device ? r(snap.device.wheelActualMps, 4) : null,
        vWheelCmdL:      snap.device ? r(snap.device.vWheelCmdL, 4) : null,
        vWheelCmdR:      snap.device ? r(snap.device.vWheelCmdR, 4) : null,
        wheelActualMpsL: snap.device ? r(snap.device.wheelActualMpsL, 4) : null,
        wheelActualMpsR: snap.device ? r(snap.device.wheelActualMpsR, 4) : null,
        // LEDC diagnostic: signed step-pulse frequency [Hz at STEP pin]
        // most recently passed to / returned from ledcWriteTone() per
        // channel. Sign reflects commanded direction at the call. req
        // and got should track exactly at LEDC_RES_BITS=8; any divergence
        // (or got==0 while req!=0) indicates an LEDC issue worth
        // investigating. Surfaced in lastCtrl too so a quick eyeball of
        // the dump tells you the current state without scanning history.
        ledcReqL: snap.device ? r(snap.device.ledcReqL, 0) : null,
        ledcGotL: snap.device ? r(snap.device.ledcGotL, 0) : null,
        ledcReqR: snap.device ? r(snap.device.ledcReqR, 0) : null,
        ledcGotR: snap.device ? r(snap.device.ledcGotR, 0) : null,
        vBat:  snap.device ? r(snap.device.vBat, 2) : null,
        flags: snap.device ? snap.device.flags : null,
        flagsLabel,
      },
      historyStartT: r(historyWindow.length ? historyWindow[0].t : 0, 3),
      historyEndT:   r(historyWindow.length ? historyWindow[historyWindow.length - 1].t : 0, 3),
      history: historyWindow.map((s) => ({
        t: r(s.t, 3),
        thRaw: r(s.theta, 5),
        thEff: s.thetaEff !== undefined ? r(s.thetaEff, 5) : null,
        thD: r(s.thetaDot, 4),
        x: r(s.x, 4), xD: r(s.xDot, 4),
        aSet: r(s.angleSet, 5),
        oP: r(s.outerP, 3), oI: r(s.outerI, 3), oD: r(s.outerD, 3),
        vCmd: s.vWheelCmd !== undefined ? r(s.vWheelCmd, 4) : null,
        vAct: s.wheelActualMps !== undefined ? r(s.wheelActualMps, 4) : null,
        vCmdL: s.vWheelCmdL !== undefined ? r(s.vWheelCmdL, 4) : null,
        vCmdR: s.vWheelCmdR !== undefined ? r(s.vWheelCmdR, 4) : null,
        vActL: s.wheelActualMpsL !== undefined ? r(s.wheelActualMpsL, 4) : null,
        vActR: s.wheelActualMpsR !== undefined ? r(s.wheelActualMpsR, 4) : null,
        // LEDC req/got per side (signed Hz at STEP pin) — captured per
        // sample so a paused dump shows whether req/got diverged or
        // dropped to 0 anywhere in the rolling window. Integer rounding
        // (decimals=0) since these are discrete frequency values.
        lReqL: s.ledcReqL !== undefined ? r(s.ledcReqL, 0) : null,
        lGotL: s.ledcGotL !== undefined ? r(s.ledcGotL, 0) : null,
        lReqR: s.ledcReqR !== undefined ? r(s.ledcReqR, 0) : null,
        lGotR: s.ledcGotR !== undefined ? r(s.ledcGotR, 0) : null,
        aX:   s.accelX !== undefined ? r(s.accelX, 4) : null,
        vBat: s.vBat !== undefined ? r(s.vBat, 2) : null,
        fl:   s.flags ?? null,
      })),
    };
  }

  const text = JSON.stringify(debug);

  // Two sinks, independent: clipboard (for the developer) and a POST to
  // the Vite dev server middleware (for the agent collaborating on tuning,
  // which can Read the file but cannot reach localhost). We attempt both
  // unconditionally so a failure in one never blocks the other; the toast
  // reports the union of outcomes.
  let clipOk = false;
  let saveOk = false;
  try {
    await navigator.clipboard.writeText(text);
    clipOk = true;
  } catch (e) {
    console.error("Clipboard write failed:", e);
    console.log(text);
  }
  try {
    const res = await fetch("/__debug/dump", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: text,
    });
    saveOk = res.ok;
    if (!res.ok) console.error("Debug save HTTP", res.status);
  } catch (e) {
    // Likely the page is being served from somewhere other than the Vite
    // dev server (e.g. opened from disk, or the middleware is absent in a
    // production build). Don't surface a hard error.
    console.warn("Debug save POST failed (non-fatal):", e);
  }

  const btn = document.getElementById("btn-debug") as HTMLButtonElement;
  const orig = btn.textContent;
  const kb = (text.length / 1024).toFixed(1);
  let label: string;
  if (clipOk && saveOk) label = `Copied + saved (${kb} KB)`;
  else if (clipOk)      label = `Copied only (${kb} KB)`;
  else if (saveOk)      label = `Saved only (${kb} KB)`;
  else {
    label = "Failed (see console)";
    alert("Both clipboard and save failed; debug info logged to console.");
  }
  btn.textContent = label;
  setTimeout(() => { btn.textContent = orig; }, 1800);
});

// ---- Connection panel ----
const devHostInput = document.getElementById("dev-host")   as HTMLInputElement;
const btnConnect   = document.getElementById("btn-connect") as HTMLButtonElement;
const connStatus   = document.getElementById("conn-status") as HTMLSpanElement;

devHostInput.value = initialHost;
devHostInput.addEventListener("change", () => {
  const h = devHostInput.value.trim() || "balancebot.local";
  devHostInput.value = h;
  localStorage.setItem(HOST_KEY, h);
  deviceSource.setHost(h);
  audioBridge.setHost(h);
  applyCameraStream();
});

// Camera stream URL management. Independent of the WS connection — the
// browser handles MJPEG via the native multipart/x-mixed-replace path
// on the <img> element and retries on its own when the TCP connection
// drops. We only flip src on / off based on (a) source mode (no point
// in sim mode) and (b) host changes. A cache-buster query string
// forces a fresh connect when the host is re-applied so the browser
// doesn't try to resume a cached stream.
const camStream = document.getElementById("cam-stream") as HTMLImageElement | null;
const camStatus = document.getElementById("cam-status") as HTMLSpanElement  | null;
function setCamStatus(label: string): void {
  if (camStatus) camStatus.textContent = label;
}
function applyCameraStream(): void {
  if (!camStream) return;
  if (!cameraEnabled) {
    camStream.src = "";
    setCamStatus("disabled");
    return;
  }
  const host = deviceSource.getHost();
  camStream.src = `http://${host}/stream?t=${Date.now()}`;
  setCamStatus("connecting…");
}
if (camStream) {
  // The browser fires `load` once when the first JPEG of the MJPEG
  // stream is decoded (it never fires again — the response is
  // effectively a never-ending body). `error` fires on connect failure
  // and on stream drop. Use both to drive the small status pill.
  camStream.addEventListener("load",  () => setCamStatus("live"));
  camStream.addEventListener("error", () => setCamStatus("error"));
}

// ---- Coproc subsystem toggles (camera + mic) ----
//
// Two buttons that command the coproc to fully stop its work on either
// stream. Used to A/B which subsystem is causing the loop() stalls we
// see on coproc — video, audio, both, or neither.
//
// Local-side handling is also non-trivial: when we disable the camera
// we also wipe the <img> src so the browser stops trying to reconnect
// to the now-disabled /stream endpoint; when we disable the mic we
// tear down the AudioBridge so the audio WS drops too. Both states
// persist in localStorage so a reload doesn't surprise the user with
// the bot suddenly streaming again.
const CAM_KEY = "balancing-robot.cameraEnabled";
const MIC_KEY = "balancing-robot.micEnabled";
let cameraEnabled = localStorage.getItem(CAM_KEY) !== "0";
let micEnabled    = localStorage.getItem(MIC_KEY) !== "0";

const btnVideo = document.getElementById("btn-video") as HTMLButtonElement | null;
const btnMic   = document.getElementById("btn-mic")   as HTMLButtonElement | null;
function updateSubsystemButtons(): void {
  if (btnVideo) btnVideo.textContent = cameraEnabled ? "Disable" : "Enable";
  if (btnMic)   btnMic.textContent   = micEnabled    ? "Disable" : "Enable";
}
function sendCameraEnabled(): void {
  deviceSource.send({ type: "setCameraEnabled", enabled: cameraEnabled });
}
function sendMicEnabled(): void {
  deviceSource.send({ type: "setMicEnabled", enabled: micEnabled });
}
if (btnVideo) {
  btnVideo.addEventListener("click", () => {
    cameraEnabled = !cameraEnabled;
    localStorage.setItem(CAM_KEY, cameraEnabled ? "1" : "0");
    updateSubsystemButtons();
    sendCameraEnabled();
    // Local: clear or restore the <img> src so the browser stops
    // pulling from a now-disabled endpoint.
    applyCameraStream();
  });
}
if (btnMic) {
  btnMic.addEventListener("click", () => {
    micEnabled = !micEnabled;
    localStorage.setItem(MIC_KEY, micEnabled ? "1" : "0");
    updateSubsystemButtons();
    sendMicEnabled();
    // Local: if the user was playing audio, stop the bridge so the WS
    // drops and the worklet releases.
    if (!micEnabled && audioBridge.isRunning()) {
      audioBridge.stop();
      updateAudioButton();
    }
  });
}
updateSubsystemButtons();

// ---- Telemetry stream toggle ----
//
// Mutes the 60 Hz telemetry stream from main. Status snapshots (1 Hz,
// PKT_STATUS) keep flowing regardless, so the coproc's battery page +
// HTML status page still update from those. Used when the operator
// wants to cut UART / WiFi traffic without losing safety-critical
// readings.
//
// State persists in localStorage and re-pushes on WS reconnect, same
// pattern as camera/mic. Note the command path: coproc forwards
// setTelemetryEnabled to main via PKT_WS_CMD, where cmdrx applies it
// to telemetry::setEnabled — there's no coproc-local short-circuit.
const TEL_KEY = "balancing-robot.telemetryEnabled";
let telemetryEnabled = localStorage.getItem(TEL_KEY) !== "0";
const btnTelemetry = document.getElementById("btn-telemetry") as HTMLButtonElement | null;
function updateTelemetryButton(): void {
  if (btnTelemetry) btnTelemetry.textContent = telemetryEnabled ? "Disable" : "Enable";
}
function sendTelemetryEnabled(): void {
  deviceSource.send({ type: "setTelemetryEnabled", enabled: telemetryEnabled });
}
if (btnTelemetry) {
  btnTelemetry.addEventListener("click", () => {
    telemetryEnabled = !telemetryEnabled;
    localStorage.setItem(TEL_KEY, telemetryEnabled ? "1" : "0");
    updateTelemetryButton();
    sendTelemetryEnabled();
  });
}
updateTelemetryButton();

// ---- Dot-matrix display controls ----
//
// Coproc owns rendering; we just push setDisplayPage / setDisplayText /
// setDisplayEnabled WS commands. Page selection persists in
// localStorage so a reload preserves the operator's choice — but we
// don't try to mirror page state coming back from the coproc; auto-
// cycle and the Square button can advance it without our knowledge,
// and we'd be lying to highlight a button that no longer matches.
const DISP_ENABLED_KEY = "balancing-robot.displayEnabled";
const DISP_PAGE_KEY    = "balancing-robot.displayPage";
const DISP_CYCLE_KEY   = "balancing-robot.displayCycleMs";
let displayEnabled = localStorage.getItem(DISP_ENABLED_KEY) !== "0";
const dispPages = ["text", "battery", "eyes"] as const;
type DispPage = typeof dispPages[number];
function loadStoredPage(): DispPage | null {
  const v = localStorage.getItem(DISP_PAGE_KEY);
  return v && (dispPages as readonly string[]).includes(v) ? (v as DispPage) : null;
}
function loadStoredCycle(): number {
  const v = parseInt(localStorage.getItem(DISP_CYCLE_KEY) ?? "0", 10);
  return Number.isFinite(v) && v >= 0 ? v : 0;
}

const btnDispEn   = document.getElementById("btn-display-enabled") as HTMLButtonElement | null;
const btnPageText = document.getElementById("btn-page-text")       as HTMLButtonElement | null;
const btnPageBat  = document.getElementById("btn-page-battery")    as HTMLButtonElement | null;
const btnPageEyes = document.getElementById("btn-page-eyes")       as HTMLButtonElement | null;
const btnPageNext = document.getElementById("btn-page-next")       as HTMLButtonElement | null;
const dispText    = document.getElementById("display-text")        as HTMLInputElement  | null;
const btnDispText = document.getElementById("btn-display-text")    as HTMLButtonElement | null;
const dispCycleIn = document.getElementById("display-cycle-ms")    as HTMLInputElement  | null;
const btnDispCyc  = document.getElementById("btn-display-cycle")   as HTMLButtonElement | null;

const pageBtns: Record<DispPage, HTMLButtonElement | null> = {
  text:    btnPageText,
  battery: btnPageBat,
  eyes:    btnPageEyes,
};
let activeDispPage: DispPage | null = loadStoredPage();

function updateDispButtons(): void {
  if (btnDispEn) btnDispEn.textContent = displayEnabled ? "Disable" : "Enable";
  for (const p of dispPages) {
    const btn = pageBtns[p];
    if (btn) btn.classList.toggle("active", activeDispPage === p);
  }
}
function sendDisplayEnabled(): void {
  deviceSource.send({ type: "setDisplayEnabled", enabled: displayEnabled });
}
function sendDisplayPage(page: DispPage | "next"): void {
  deviceSource.send({ type: "setDisplayPage", page });
}
function sendDisplayText(s: string): void {
  deviceSource.send({ type: "setDisplayText", text: s });
}
function sendAutoCycle(ms: number): void {
  // Re-issuing setDisplayPage is the only path the coproc exposes for
  // autoCycleMs (it's a sibling field, not a separate command). Send
  // page="next" with an immediate no-op? No — sending only autoCycleMs
  // requires *some* page action. Cleanest: send the current localStorage
  // page if known; otherwise "next" — which is a cycle hop the operator
  // explicitly clicked Apply for, so the side-effect is OK.
  const page: DispPage | "next" = activeDispPage ?? "next";
  deviceSource.send({ type: "setDisplayPage", page, autoCycleMs: ms });
}

if (btnDispEn) {
  btnDispEn.addEventListener("click", () => {
    displayEnabled = !displayEnabled;
    localStorage.setItem(DISP_ENABLED_KEY, displayEnabled ? "1" : "0");
    updateDispButtons();
    sendDisplayEnabled();
  });
}
for (const p of dispPages) {
  const btn = pageBtns[p];
  if (!btn) continue;
  btn.addEventListener("click", () => {
    activeDispPage = p;
    localStorage.setItem(DISP_PAGE_KEY, p);
    updateDispButtons();
    sendDisplayPage(p);
  });
}
if (btnPageNext) {
  btnPageNext.addEventListener("click", () => {
    // Don't try to predict which page the coproc lands on — auto-cycle
    // or button presses can desync our cached state. Clear the active
    // highlight so the row reads "unknown" until the user clicks an
    // explicit page.
    activeDispPage = null;
    localStorage.removeItem(DISP_PAGE_KEY);
    updateDispButtons();
    sendDisplayPage("next");
  });
}
if (btnDispText && dispText) {
  const submitText = () => {
    const s = dispText.value;
    if (!s) return;
    sendDisplayText(s);
    // setDisplayText implicitly switches the coproc to the text page,
    // so reflect that in our cached state.
    activeDispPage = "text";
    localStorage.setItem(DISP_PAGE_KEY, "text");
    updateDispButtons();
  };
  btnDispText.addEventListener("click", submitText);
  dispText.addEventListener("keydown", (e) => {
    if (e.key === "Enter") submitText();
  });
}
if (dispCycleIn) dispCycleIn.value = String(loadStoredCycle());
if (btnDispCyc && dispCycleIn) {
  btnDispCyc.addEventListener("click", () => {
    const ms = Math.max(0, parseInt(dispCycleIn.value, 10) || 0);
    localStorage.setItem(DISP_CYCLE_KEY, String(ms));
    sendAutoCycle(ms);
  });
}
updateDispButtons();

// Sync coproc state when the WS opens. DeviceSource calls onStatus on
// every successful open AND on every status-detail refresh (~every 100
// frames, see datasource.ts), so we dedupe on the actual disconnected
// → connected edge to avoid re-spamming the coproc once per second.
let lastSyncStatus: string | null = null;
deviceSource.onStatus((info) => {
  const prev = lastSyncStatus;
  lastSyncStatus = info.status;
  if (info.status !== "connected") return;
  if (prev === "connected") return;   // same edge; already synced
  sendCameraEnabled();
  sendMicEnabled();
  sendTelemetryEnabled();
  sendDisplayEnabled();
  // DO NOT push setDisplayPage on reconnect. The coproc has multiple
  // sources of truth for the current page — UI buttons, the PS5
  // Square button, and WS commands — and they desync the moment a
  // joystick press the UI didn't see lands. Pushing our last-clicked
  // page on every reconnect would snap the display back to whatever
  // the UI thinks is "active", overriding the operator's joystick
  // selection. The page state survives WS link drops on the coproc
  // anyway; if it actually reboots, the display starts on Eyes and
  // the operator can re-select.
  //
  // We do re-push autoCycleMs because it's reset to 0 on coproc
  // reboot and the UI's stored value is the only thing that knows
  // the operator wanted it on. No "page" key here, so the handler
  // updates the interval without changing pages.
  const cyc = loadStoredCycle();
  if (cyc > 0) {
    deviceSource.send({ type: "setDisplayPage", autoCycleMs: cyc });
  }
});

// ---- Audio bridge (Phase C) ----
//
// Streaming audio playback requires a user gesture, so the bridge stays
// idle until the operator clicks Unmute. The button toggles between
// idle ⇄ playing — also handles "muted while playing" by flipping the
// gain node inside the bridge without tearing down the WS / context.
const audioBridge = new AudioBridge(initialHost);
const btnAudio    = document.getElementById("btn-audio")    as HTMLButtonElement | null;
const audioStatus = document.getElementById("audio-status") as HTMLSpanElement    | null;
const audioLevel  = document.getElementById("audio-level")  as HTMLCanvasElement  | null;
function setAudioStatus(label: string): void {
  if (audioStatus) audioStatus.textContent = label;
}
function updateAudioButton(): void {
  if (!btnAudio) return;
  if (!audioBridge.isRunning()) { btnAudio.textContent = "Unmute"; return; }
  btnAudio.textContent = audioBridge.isMuted() ? "Unmute" : "Stop";
}
audioBridge.onStatus(setAudioStatus);
audioBridge.onPeak((peak) => {
  if (!audioLevel) return;
  const ctx = audioLevel.getContext("2d");
  if (!ctx) return;
  const w = audioLevel.width;
  const h = audioLevel.height;
  ctx.fillStyle = "#11151c";
  ctx.fillRect(0, 0, w, h);
  // Map peak (0..1) to bar width. Voice peaks ~0.3; clamp at 1.
  const lvl = Math.min(1, peak);
  // Colour gradient: green up to ~0.5, amber to ~0.8, red beyond.
  ctx.fillStyle = lvl > 0.8 ? "#ff6b6b" : (lvl > 0.5 ? "#ffb454" : "#6ce5a5");
  ctx.fillRect(1, 1, Math.max(0, (w - 2) * lvl), h - 2);
});
if (btnAudio) {
  btnAudio.addEventListener("click", async () => {
    if (!audioBridge.isRunning()) {
      audioBridge.setMuted(false);
      await audioBridge.start();
    } else {
      // Toggle mute on the running stream rather than tearing down.
      audioBridge.setMuted(!audioBridge.isMuted());
    }
    updateAudioButton();
  });
}
updateAudioButton();

function setConnStatusUi(info: DeviceStatusInfo): void {
  connStatus.className = `conn-status conn-${info.status}`;
  connStatus.textContent = info.detail
    ? `${info.status} · ${info.detail}`
    : info.status;
  // Toggle the Connect button label to mirror the WS state so the user
  // always sees the action that pressing it will perform.
  btnConnect.textContent =
    info.status === "connected" || info.status === "connecting"
      ? "Disconnect"
      : "Connect";
}
deviceSource.onStatus(setConnStatusUi);
setConnStatusUi(deviceSource.getStatus());

btnConnect.addEventListener("click", () => {
  const s = deviceSource.getStatus().status;
  if (s === "connected" || s === "connecting") deviceSource.disconnect();
  else deviceSource.connect();
});

// Initial camera-stream attach — kicks off the MJPEG <img> on first
// paint (subject to the persisted camera-enabled toggle).
applyCameraStream();
// ---- Parameter panel ----
//
// We render DeviceParams (firmware schema) via buildDeviceUI;
// every edit is forwarded as a {type:"setParam", path, value} WS message.
//
// `deviceParams` is overwritten in-place when the firmware pushes its
// {type:"params"} on connect (and after loadDefaults); the panel then
// syncs its inputs to the live device state.
const controlsEl = document.getElementById("controls")!;
const deviceParams: DeviceParams = defaultDeviceParams();

// Trim capture: when the user clicks "Capture trim" on the panel,
// average the last ~1 s of telemetered θ and push the mean to firmware
// as thetaTrim. Sampled by polling snap.state.theta at ~50 ms intervals
// rather than reading from a rolling buffer; that way the user gets a
// clear "hold still NOW" window rather than capturing whatever was in
// the buffer (which might include the tilt that prompted the press).
const TRIM_CAPTURE_MS = 1000;
const TRIM_SAMPLE_MS  = 50;
let trimCaptureActive = false;

function captureTrim(setLabel: (text: string, busy: boolean) => void): void {
  if (trimCaptureActive) return;
  trimCaptureActive = true;
  const samples: number[] = [];
  const start = performance.now();
  setLabel("Capturing 1.0 s…", true);
  const id = window.setInterval(() => {
    const elapsed = performance.now() - start;
    samples.push(snap.state.theta);
    const remaining = Math.max(0, TRIM_CAPTURE_MS - elapsed);
    setLabel(`Capturing ${(remaining / 1000).toFixed(1)} s…`, true);
    if (elapsed >= TRIM_CAPTURE_MS) {
      window.clearInterval(id);
      trimCaptureActive = false;
      if (samples.length === 0) {
        setLabel("no samples", false);
        return;
      }
      const mean = samples.reduce((a, b) => a + b, 0) / samples.length;
      // Send to firmware; mirror locally and refresh the θ-trim input
      // in place via the sync map. The full-panel rebuild we used to
      // do here would steal focus from whatever the operator was
      // editing at the moment the trim averager finished its ~1 s
      // sample window.
      const ok = deviceSource.send({ type: "setParam", path: "thetaTrim", value: mean });
      deviceParams.thetaTrim = mean;
      scheduleDeviceAutosave();
      deviceSyncMap?.get("thetaTrim")?.(mean);
      const RAD2DEG = 180 / Math.PI;
      setLabel(ok ? `θtrim=${(mean * RAD2DEG).toFixed(2)}°` : "send failed", false);
    }
  }, TRIM_SAMPLE_MS);
}

// Debounced auto-persist. Every setParam edit (number input,
// captureTrim) trails by DEVICE_AUTOSAVE_MS of inactivity; once the
// user pauses, we fire a single {type:"saveParams"} so the firmware
// writes NVS. Clicking "Save params" explicitly cancels the pending
// timer (the explicit save covers the same intent). We debounce
// instead of saving per-edit to avoid hammering NVS during a rapid
// run of edits.
const DEVICE_AUTOSAVE_MS = 2000;
let deviceAutosaveTimer: ReturnType<typeof setTimeout> | null = null;
function scheduleDeviceAutosave(): void {
  if (deviceAutosaveTimer !== null) clearTimeout(deviceAutosaveTimer);
  deviceAutosaveTimer = setTimeout(() => {
    deviceAutosaveTimer = null;
    deviceSource.send({ type: "saveParams" });
  }, DEVICE_AUTOSAVE_MS);
}
function cancelDeviceAutosave(): void {
  if (deviceAutosaveTimer !== null) {
    clearTimeout(deviceAutosaveTimer);
    deviceAutosaveTimer = null;
  }
}

// IMU calibration: device-side. We send {type:"calibrate"} and wait for
// either an ack or an error. The firmware's handler blocks for ~2 s
// (gyro-bias measurement + NVS write + params broadcast) before
// responding, so we just need a one-shot listener pair plus a watchdog
// for the case where the WS dies mid-calibration.
//
// Why not poll a "calibration state" field? The firmware doesn't expose
// one — and we don't need it: ack semantically means "done, persisted,
// and the broadcast you're about to receive contains the new bias."
let calibrateActive = false;
function calibrateIMU(setLabel: (text: string, busy: boolean) => void): void {
  if (calibrateActive) return;
  calibrateActive = true;
  const ok = deviceSource.send({ type: "calibrate" });
  if (!ok) {
    calibrateActive = false;
    setLabel("send failed", false);
    return;
  }
  setLabel("Calibrating ~2 s…", true);

  // Watchdog: firmware budget is 3.5 s end-to-end; allow a bit more for
  // network latency.
  const WATCHDOG_MS = 5000;
  let done = false;
  const finish = (text: string): void => {
    if (done) return;
    done = true;
    calibrateActive = false;
    // Best-effort listener cleanup — the source layer doesn't expose
    // remove APIs, so we just guard on `done` inside the closures. This
    // means the listeners stay registered for the rest of the session
    // but become inert after one shot. Acceptable for a manual button.
    setLabel(text, false);
  };
  const timer = window.setTimeout(() => finish("timeout"), WATCHDOG_MS);
  deviceSource.onAck((of) => {
    if (done || of !== "calibrate") return;
    window.clearTimeout(timer);
    finish("calibrated ✓");
  });
  deviceSource.onError((msg) => {
    if (done) return;
    // Filter to errors that mention calibrate so an unrelated error
    // (e.g. a stray setParam fail) doesn't end our spinner. Firmware
    // error strings all begin with "calibrate:" for this code path.
    if (!msg.startsWith("calibrate")) return;
    window.clearTimeout(timer);
    finish(msg.replace(/^calibrate:\s*/, ""));
  });
}

// Sync map for the device panel. Populated by buildPanel(); used by
// onParams to refresh inputs in place instead of tearing down the DOM.
let deviceSyncMap: SyncMap | null = null;

function buildPanel(): void {
  deviceSyncMap = buildDeviceUI(
    controlsEl,
    deviceParams,
    manualPrefs,
    (path, value) => {
      deviceSource.send({ type: "setParam", path, value });
      scheduleDeviceAutosave();
    },
    () => { saveManualPrefsDebounced(); },
    {
      onArm:          () => deviceSource.send({ type: "enableMotors", value: true }),
      onDisarm:       () => deviceSource.send({ type: "enableMotors", value: false }),
      onSave:         () => { cancelDeviceAutosave(); deviceSource.send({ type: "saveParams" }); },
      onLoadDefaults: () => deviceSource.send({ type: "loadDefaults" }),
      onCaptureTrim:  captureTrim,
      onCalibrate:    calibrateIMU,
    },
  );
}

// Firmware pushes {type:"params"} on every WS_EVT_CONNECT, after a
// loadDefaults RPC, and every PARAMS_REFRESH_MS (~5 s periodic).
// Update local DeviceParams and sync each input in place — applying
// the snapshot through deviceSyncMap leaves the DOM intact and skips
// any input that currently has focus, so an in-progress edit is
// never overwritten by a refresh that crossed paths with it on the
// wire.
deviceSource.onParams((p) => {
  if (deviceSyncMap === null) {
    // First params dump before the panel was built. Fall back to a
    // full build — there's no user edit in flight at this point.
    Object.assign(deviceParams, p);
    buildPanel();
    return;
  }
  applyDeviceSnapshot(deviceSyncMap, deviceParams, p);
});
// Surface device acks/errors in the console for now. A future step could
// route these into the connection-status pill or a transient toast.
deviceSource.onAck((of) => console.log(`device ack: ${of}`));
deviceSource.onError((msg) => console.warn(`device error: ${msg}`));

buildPanel();

// Auto-connect on page load. Listeners (onStatus/onParams/onAck/onError)
// are already wired above, so the connecting->connected->frames
// sequence surfaces in the UI. connect() arms autoReconnect for
// transient WiFi drops.
deviceSource.connect();

// ---- Main loop ----
let prevWall = performance.now();
let fpsAcc = 0, fpsCount = 0, fps = 0;
// Last seen tick().sessionEpoch. The DeviceSource bumps it on every
// (re)connect, device reboot, or detected feed gap >1 s; on a change
// we wipe chart buffers + debug history so the rolling 30 s window
// doesn't show a stale frozen segment from before the boundary.
let lastSessionEpoch = 0;
// Set when a session boundary (reconnect / feed gap) was detected
// while the UI was paused. We deliberately skip the chart reset in
// that case so the operator's mid-investigation view stays frozen;
// the reset is then performed at the moment of unpause (the chart's
// lastT is otherwise stale and the next push would trim every
// retained point as "older than lastT − windowSec"). Pressing Reset
// also clears the flag because resetView() is called there too.
let pendingSessionReset = false;

function resetView(): void {
  deviceSource.reset();
  // Re-prime the snapshot so the HUD sees the post-reset state on the
  // very next frame, even before tick() is called with advance > 0.
  snap = deviceSource.tick(0).snap;
  angleChart.reset();
  ratesChart.reset();
  posChart.reset();
  pidOuterChart.reset();
  innerDevChart.reset();
  wheelDevChart.reset();
  ledcDevChart.reset();
  summaryDevChart.reset();
  powerDevChart.reset();
  debugHistory.length = 0;
  // Any pending deferred session reset is now satisfied; don't fire it
  // again on the next unpause.
  pendingSessionReset = false;
}

function frame(now: number): void {
  const wallDt = (now - prevWall) / 1000;
  prevWall = now;
  fpsAcc += wallDt; fpsCount++;
  if (fpsAcc >= 0.5) { fps = fpsCount / fpsAcc; fpsAcc = 0; fpsCount = 0; }

  let advance = 0;
  // consumeFrame: whether to apply this frame's tick result to snap, charts,
  // and debug history. False when paused. We still call tick()
  // unconditionally so DeviceSource can drain its accumulator —
  // otherwise resuming would dump the entire pause duration as a
  // single chart point. When paused, snap and all charts stay frozen
  // on the moment-of-pause; pressing Play consumes the next frame.
  let consumeFrame = false;
  if (running) {
    advance = wallDt;
    consumeFrame = true;
  }

  const tr = deviceSource.tick(advance);
  if (consumeFrame) snap = tr.snap;

  // Session-boundary handling. DeviceSource bumps sessionEpoch on every
  // (re)connect and on detected feed discontinuities; if we don't reset
  // chart buffers here, the next chart push slides lastT forward by the
  // gap duration and LiveChart trims everything older than
  // lastT − windowSec, leaving the chart effectively empty for one full
  // window before fresh samples backfill it. Calling resetSim() also
  // clears debug history (which would otherwise mix pre- and
  // post-reconnect samples with a discontinuous tSec axis). We do this
  // *before* the chart push gate below so the post-reconnect frame goes
  // straight into a clean buffer.
  //
  // Exception: if the UI is paused, the operator is mid-investigation
  // and a reconnect-driven reset would wipe the very data they're
  // looking at. Defer the reset to the moment of unpause (handled in
  // the play/pause toggle below). Pressing Reset explicitly also
  // clears the pending flag since resetSim() is called there too.
  if (tr.sessionEpoch !== lastSessionEpoch) {
    lastSessionEpoch = tr.sessionEpoch;
    if (running) {
      resetView();
    } else {
      pendingSessionReset = true;
    }
  }

  // Auto-pause on FALLEN rising edge. When the safety FSM
  // latches FLAG_FALLEN (bit 0), freeze the UI immediately so the chart
  // window still contains the lead-up to the fall and the operator can
  // copy a meaningful debug dump. Without this, by the time the user
  // notices and clicks Pause the interesting moments have scrolled past
  // the rolling window. We only auto-pause on the 0->1 transition so a
  // resumed-while-fallen state does not re-pause every frame.
  if (consumeFrame && snap.device) {
    const flags = snap.device.flags;
    if (running && prevDeviceFlags !== undefined && (prevDeviceFlags & 1) === 0 && (flags & 1) !== 0) {
      running = false;
      updatePlayPauseLabel();
      console.warn("FALLEN flag set; auto-pausing for post-mortem.");
    }
    prevDeviceFlags = flags;
  } else if (!snap.device) {
    prevDeviceFlags = undefined;
  }

  // Surface any catastrophic error from the source.
  const err = deviceSource.consumeError();
  if (err) {
    running = false;
    updatePlayPauseLabel();
    console.warn(`Source error: ${err}; pausing.`);
  }

  if (consumeFrame && tr.advancedSec > 0) {
    // Push chart samples once per frame (downsampled by rAF cadence).
    const RAD2DEG = 180 / Math.PI;
    // Series 1 of the angle chart is the displayed θ. There is no longer
    // a separate display-only filter on-device; we plot the raw fused
    // theta the controller actually sees.
    const thetaDisp = snap.state.theta;
    angleChart.push(snap.tSec, [
      thetaDisp * RAD2DEG,
      // Second series: the trim-applied effective angle the controller
      // actually operates on (raw IMU θ − thetaTrim), so the chart
      // shows the gap between the fused IMU output and the controller
      // input while bench-tuning thetaTrim.
      (snap.measurement.theta - deviceParams.thetaTrim) * RAD2DEG,
      snap.ctrl.angleSet * RAD2DEG,
    ]);
    ratesChart.push(snap.tSec, [
      snap.state.thetaDot * RAD2DEG,
      snap.device ? snap.device.gyroZ * RAD2DEG : 0,
    ]);
    posChart.push(snap.tSec, [snap.state.x]);
    // tgtV / tgtTurn come straight from the firmware telemetry frame
    // (shared::g.targetV / .targetTurn / .targetTurnUsed) — NOT a
    // client-side reconstruction from joystick state, which would
    // silently disagree with what the controller saw whenever a
    // setParam or RC source intervened. tgtTurn cmd = raw operator
    // request; tgtTurn used = post-ramp-filter, post-±vMaxTurn-clamp
    // value the controller summed into vL/vR.
    const tgtV        = snap.device ? snap.device.targetV        : 0;
    const tgtTurnCmd  = snap.device ? snap.device.targetTurn     : 0;
    const tgtTurnUsed = snap.device ? snap.device.targetTurnUsed : 0;
    pidOuterChart.push(snap.tSec, [snap.ctrl.outer.p, snap.ctrl.outer.i, snap.ctrl.outer.d, tgtV, tgtTurnCmd, tgtTurnUsed]);
    if (snap.device) {
      // Inner law components, computed simulator-side from telemetry +
      // the live DeviceParams (firmware doesn't telemeter the
      // decomposition, only the sum vWheelCmd). Mirrors
      // controller.cpp:139-146:
      //   thetaErr = (theta - thetaTrim) - thetaSet
      //   vWheel   = velFF·targetV + Kth·thetaErr + KthDot·thDot
      const thetaErr = (snap.state.theta - deviceParams.thetaTrim) - snap.ctrl.angleSet;
      const ff   = deviceParams.velFF  * lastDeviceTargetV;
      const pTh  = deviceParams.Kth    * thetaErr;
      const dTh  = deviceParams.KthDot * snap.state.thetaDot;
      innerDevChart.push(snap.tSec, [ff, pTh, dTh, snap.device.vWheelCmd]);
      // L/R per-wheel: cmd values are post-split + post-saturation
      // (i.e. exactly what setWheelVelocity received); actual values are
      // sign-corrected for mounting invert so they share the cart-frame
      // sign convention with cmd.
      wheelDevChart.push(snap.tSec, [
        snap.device.vWheelCmdL,
        snap.device.vWheelCmdR,
        snap.device.wheelActualMpsL,
        snap.device.wheelActualMpsR,
      ]);
      // LEDC diagnostic: in steps/sec (Hz at the STEP pin). Series order
      // matches the chart definition above (req L, req R, got L, got R).
      // Always pushed in device mode — even when channels are idle (all
      // four = 0), so the chart shows a flat 0 baseline rather than
      // freezing on the last active spin's values.
      ledcDevChart.push(snap.tSec, [
        snap.device.ledcReqL,
        snap.device.ledcReqR,
        snap.device.ledcGotL,
        snap.device.ledcGotR,
      ]);
      // Single-glance summary: angle frame on the left axis, wheel-velocity
      // frame on the right. thetaEff is what the controller sees (after
      // trim subtraction); pTh = Kth·err is the dominant control term.
      const thetaEffDeg = (snap.state.theta - deviceParams.thetaTrim) * RAD2DEG;
      summaryDevChart.push(snap.tSec, [
        thetaEffDeg,
        snap.ctrl.angleSet * RAD2DEG,
        pTh,
        snap.device.vWheelCmd,
        snap.device.wheelActualMps,
      ]);

      // Power: INA226 vBus + iBus on dual axes. Independent of vBat
      // (which lives in the device HUD pill), so chart-only here.
      powerDevChart.push(snap.tSec, [snap.device.vBus, snap.device.iBus]);
    }

    // Rolling history for the debug-copy button. Pushed every frame at the
    // source rate (no decimation) so transients aren't hidden between slots.
    addDebugSample(() => ({
      t: snap.tSec,
      theta: snap.state.theta, thetaDot: snap.state.thetaDot,
      x: snap.state.x, xDot: snap.state.xDot,
      measTheta: snap.measurement.theta,
      tau: snap.ctrl.tau, angleSet: snap.ctrl.angleSet,
      pTerm: snap.ctrl.inner.p, iTerm: snap.ctrl.inner.i, dTerm: snap.ctrl.inner.d,
      outerP: snap.ctrl.outer.p, outerI: snap.ctrl.outer.i, outerD: snap.ctrl.outer.d,
      // Device-only extras. Captured per-sample so a paused dump shows the
      // actuation history (vWheelCmd vs wheelActualMps) and the effective
      // angle the controller saw, not just the raw IMU reading.
      ...(snap.device ? {
        vWheelCmd: snap.device.vWheelCmd,
        wheelActualMps: snap.device.wheelActualMps,
        vWheelCmdL: snap.device.vWheelCmdL,
        vWheelCmdR: snap.device.vWheelCmdR,
        wheelActualMpsL: snap.device.wheelActualMpsL,
        wheelActualMpsR: snap.device.wheelActualMpsR,
        accelX: snap.device.accelX,
        vBat: snap.device.vBat,
        flags: snap.device.flags,
        thetaEff: snap.state.theta - deviceParams.thetaTrim,
        // LEDC diagnostic: signed steps/sec (Hz at STEP pin) per channel.
        // Captured per-sample so a paused dump shows whether req/got
        // diverged at any point in the rolling window.
        ledcReqL: snap.device.ledcReqL,
        ledcGotL: snap.device.ledcGotL,
        ledcReqR: snap.device.ledcReqR,
        ledcGotR: snap.device.ledcGotR,
      } : {}),
    }));
  }

  // Charts redraw.
  angleChart.redraw();
  ratesChart.redraw();
  posChart.redraw();
  pidOuterChart.redraw();
  innerDevChart.redraw();
  wheelDevChart.redraw();
  ledcDevChart.redraw();
  summaryDevChart.redraw();
  powerDevChart.redraw();

  // HUD. θ is the trim-applied effective angle — what the controller
  // actually balances around.
  const RAD2DEG_HUD = 180 / Math.PI;
  const thetaEff = snap.state.theta - deviceParams.thetaTrim;
  let line =
    `t=${snap.tSec.toFixed(2)}s  ` +
    `θ=${(thetaEff * RAD2DEG_HUD).toFixed(2)}°  ` +
    `θ̇=${(snap.state.thetaDot * RAD2DEG_HUD).toFixed(1)}°/s  ` +
    `x=${snap.state.x.toFixed(3)}m  ` +
    `ẋ=${snap.state.xDot.toFixed(2)}m/s  ` +
    `fps=${fps.toFixed(0)}`;
  // Battery + status flags — appended once the WS has decoded a
  // telemetry frame.
  if (snap.device) {
    const flagsLabel = decodeDeviceFlags(snap.device.flags) || "—";
    // Battery percentage is a linear interpolation from the configured
    // cutoff (0%) up to a derived "full" voltage (100%), assuming a
    // 4.2V/cell full vs 3.3V/cell cutoff ratio (≈1.27×). LiPo discharge
    // is famously non-linear so this is rough — a 4S pack at 50% by
    // voltage is not at 50% by remaining charge — but it gives the
    // operator a quick "is the cutoff close?" signal without adding a
    // separate vBatFull param. Clamped to [0, 100] so a freshly-charged
    // pack reading slightly above the linear-full point doesn't show
    // 103%, and so a pack that has tripped the cutoff doesn't show
    // negative percentages.
    const full = deviceParams.vBatCutoff * (4.2 / 3.3);
    const span = Math.max(full - deviceParams.vBatCutoff, 0.01);
    const pctRaw = ((snap.device.vBat - deviceParams.vBatCutoff) / span) * 100;
    const pct = Math.max(0, Math.min(100, pctRaw));
    line += `  vBat=${snap.device.vBat.toFixed(2)}V (${pct.toFixed(0)}%)  flags=${flagsLabel}`;
  }
  hud.textContent = line + (running ? "" : "  [paused]");

  requestAnimationFrame(frame);
}

resetView();
requestAnimationFrame((t) => { prevWall = t; frame(t); });
