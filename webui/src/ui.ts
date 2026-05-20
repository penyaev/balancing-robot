// Declarative parameter UI builder. Reads/writes a target object via paths.
//
// Each control row has a label, a numeric input (and slider when bounded),
// and a value display. Changes mutate the bound target object in place and
// fire onChange(path, valueInModelUnits) so the caller can react (persist,
// or send a setParam WS message in Device mode).

import { ManualPrefs } from "./types";

type Path = string; // dotted, e.g. "control.manualVelocity"; or bare for flat objects

export interface NumOpts {
  min?: number;
  max?: number;
  step?: number;
  unit?: string;
  decimals?: number;
  // Display-side scaling: the value shown to the user (in slider/number input) is
  // (storedValue * displayMul). On input we divide by displayMul before storing.
  // Used e.g. to display radians as degrees while keeping radians in the model.
  displayMul?: number;
}

export interface GroupSpec {
  title: string;
  rows: RowSpec[];
}

export type RowSpec =
  | { kind: "num"; label: string; path: Path; opts?: NumOpts; tooltip?: string; target?: any; onChange?: (path: string, value: any) => void }
  | { kind: "select"; label: string; path: Path; options: string[]; tooltip?: string; target?: any; onChange?: (path: string, value: any) => void }
  | { kind: "bool"; label: string; path: Path; tooltip?: string; numeric?: boolean; target?: any; onChange?: (path: string, value: any) => void };

function getPath(obj: any, path: string): any {
  return path.split(".").reduce((o, k) => o?.[k], obj);
}
function setPath(obj: any, path: string, val: any): void {
  const parts = path.split(".");
  let o = obj;
  for (let i = 0; i < parts.length - 1; i++) o = o[parts[i]];
  o[parts[parts.length - 1]] = val;
}

// Render a list of GroupSpecs against `target` (any object whose paths the
// rows reference) into `container`. Each user edit mutates target in place
// and calls onChange(path, valueInModelUnits) — display scaling has already
// been undone, so the value passed is exactly what now lives in target.
//
// Drives buildDeviceUI; factored out so the row/input rendering code
// is defined once.
//
// Returns a sync map: for each row, a closure that takes a fresh stored
// value (the raw, undisplayMul'd, untouched-by-the-UI value the
// owning object would carry) and writes it into the corresponding
// input element WITHOUT recreating any DOM. Callers (notably
// applyDeviceSnapshot) use this to refresh the panel from an incoming
// device-side params dump without ripping the DOM out from under a
// focused input. The closures internally skip rewriting an input that
// is currently `document.activeElement`, so a user edit in progress
// is never trampled by a periodic refresh.
export type RowSync = (newStored: any) => void;
export type SyncMap = Map<string, RowSync>;

export function buildPanelGroups(
  container: HTMLElement,
  target: any,
  groups: GroupSpec[],
  onChange: (path: string, valueInModelUnits: any) => void,
): SyncMap {
  const syncMap: SyncMap = new Map();
  for (const g of groups) {
    const groupEl = document.createElement("div");
    groupEl.className = "group";
    const header = document.createElement("h3");
    header.textContent = g.title;
    header.addEventListener("click", () => groupEl.classList.toggle("collapsed"));
    groupEl.appendChild(header);
    const body = document.createElement("div");
    body.className = "group-body";
    groupEl.appendChild(body);

    for (const row of g.rows) {
      // Per-row target/onChange override. Lets a single GroupSpec mix
      // rows that bind to different objects (e.g. the device panel's
      // "Manual control" group, where some rows mutate the simulator's
      // local Params and others mutate DeviceParams via setParam WS).
      const rowTarget = (row as any).target ?? target;
      const rowOnChange: (path: string, value: any) => void =
        (row as any).onChange ?? onChange;

      const r = document.createElement("div");
      r.className = "row";
      const lab = document.createElement("label");
      lab.textContent = row.label;
      // Discrete help indicator. Browser-native tooltip on hover/long-press;
      // no positioning logic needed and works on touch. The glyph is in the
      // label so the dotted-underline cursor:help affordance applies to the
      // whole label row rather than just the icon.
      if (row.tooltip) {
        lab.title = row.tooltip;
        lab.style.cursor = "help";
        const hint = document.createElement("span");
        hint.textContent = " ⓘ";
        hint.style.color = "var(--muted)";
        hint.style.opacity = "0.6";
        lab.appendChild(hint);
      }
      r.appendChild(lab);

      if (row.kind === "num") {
        const opts = row.opts ?? {};
        const decimals = opts.decimals ?? 3;
        const mul = opts.displayMul ?? 1;
        const cur = Number(getPath(rowTarget, row.path)) * mul;

        // Number-only input: the slider was visual noise (range was often
        // arbitrary, drag steps too coarse, screen real-estate cost too
        // high) and most tuning happens via direct typing or arrow-key
        // increments on the number field.
        const num = document.createElement("input");
        num.type = "number";
        num.step = String(opts.step ?? 0.01);
        if (opts.min !== undefined) num.min = String(opts.min);
        if (opts.max !== undefined) num.max = String(opts.max);
        num.value = cur.toFixed(decimals);

        const valEl = document.createElement("span");
        valEl.className = "val";
        valEl.textContent = opts.unit ? opts.unit : "";

        // `v` is in *display* units; we divide by mul before storing.
        const apply = (v: number) => {
          if (Number.isFinite(v)) {
            const stored = v / mul;
            setPath(rowTarget, row.path, stored);
            num.value = v.toFixed(decimals);
            rowOnChange(row.path, stored);
          }
        };
        num.addEventListener("change", () => apply(parseFloat(num.value)));

        // External-sync entry: refresh `num.value` from a fresh stored
        // value, unless the user is currently focused on it. The string
        // comparison after rounding avoids no-op writes (which would
        // also reset cursor position in some browsers even on the same
        // string).
        syncMap.set(row.path, (newStored: any) => {
          if (document.activeElement === num) return;
          const n = Number(newStored);
          if (!Number.isFinite(n)) return;
          const formatted = (n * mul).toFixed(decimals);
          if (num.value !== formatted) num.value = formatted;
        });

        r.appendChild(num);
        r.appendChild(valEl);
      } else if (row.kind === "select") {
        const sel = document.createElement("select");
        for (const o of row.options) {
          const opt = document.createElement("option");
          opt.value = o;
          opt.textContent = o;
          sel.appendChild(opt);
        }
        sel.value = String(getPath(rowTarget, row.path));
        sel.addEventListener("change", () => {
          setPath(rowTarget, row.path, sel.value);
          rowOnChange(row.path, sel.value);
        });
        syncMap.set(row.path, (newStored: any) => {
          if (document.activeElement === sel) return;
          const s = String(newStored);
          if (sel.value !== s) sel.value = s;
        });
        const filler = document.createElement("span");
        filler.className = "val";
        r.appendChild(sel);
        r.appendChild(filler);
      } else {
        // bool. When `numeric: true` we coerce to 1/0 on write — used for
        // firmware fields stored as float-with-bool-semantics (e.g.
        // autoArmEnabled), so the UI can present a checkbox while the
        // underlying schema stays numeric and serialises identically over
        // the WS.
        const cb = document.createElement("input");
        cb.type = "checkbox";
        cb.checked = !!getPath(rowTarget, row.path);
        cb.addEventListener("change", () => {
          const stored: any = row.numeric ? (cb.checked ? 1 : 0) : cb.checked;
          setPath(rowTarget, row.path, stored);
          rowOnChange(row.path, stored);
        });
        syncMap.set(row.path, (newStored: any) => {
          if (document.activeElement === cb) return;
          // For numeric (float-with-bool semantics) the firmware can
          // send 0.0 / 1.0 — anything ≥ 0.5 reads as true to match the
          // safety::isEnabled style threshold used on-device.
          const want = row.numeric ? (Number(newStored) >= 0.5) : !!newStored;
          if (cb.checked !== want) cb.checked = want;
        });
        const wrap = document.createElement("label");
        wrap.style.display = "inline-flex";
        wrap.style.alignItems = "center";
        wrap.style.gap = "4px";
        wrap.appendChild(cb);
        const txt = document.createElement("span");
        txt.style.color = "var(--muted)";
        txt.style.fontSize = "11px";
        txt.textContent = "enabled";
        wrap.appendChild(txt);
        const filler = document.createElement("span");
        filler.className = "val";
        r.appendChild(wrap);
        r.appendChild(filler);
      }

      body.appendChild(r);
    }

    container.appendChild(groupEl);
  }
  return syncMap;
}

// Device parameter panel. Renders DeviceParams (firmware schema) and
// routes every edit to onParamChange(path, value); callers send a
// setParam WS message from there. The Actions section hosts Arm /
// Disarm / Save / Load defaults buttons that fire the corresponding WS
// commands directly. The "Manual control" group additionally carries a
// couple of browser-owned operator preferences (manualVelocity /
// manualTurn) bound to a separate ManualPrefs object.
import { DeviceParams, deviceGroups } from "./device_params";

export interface DeviceActions {
  onArm: () => void;
  onDisarm: () => void;
  onSave: () => void;
  onLoadDefaults: () => void;
  // Capture-trim is a browser-side averager: hold the bot physically
  // upright and click; the UI samples telemetered θ over ~1 s and sends
  // {type:"setParam", path:"thetaTrim", value:<mean>}. The button is
  // passed an updater so it can render its own progress label without
  // each callsite reimplementing the timer.
  onCaptureTrim: (setLabel: (text: string, busy: boolean) => void) => void;
  // IMU calibration: fires {type:"calibrate"}; the firmware runs a 2 s
  // gyro-bias measurement (chassis must be still + motors disarmed) and
  // persists the result to NVS as gyroBias{X,Y,Z}. Unlike captureTrim
  // (browser-side averaging) the heavy lifting is on the device — this
  // button just kicks it off and updates the label from the ack/error.
  onCalibrate: (setLabel: (text: string, busy: boolean) => void) => void;
}

export function buildDeviceUI(
  container: HTMLElement,
  deviceParams: DeviceParams,
  manualPrefs: ManualPrefs,
  onParamChange: (path: string, value: any) => void,
  onManualPrefChange: () => void,
  actions: DeviceActions,
): SyncMap {
  container.innerHTML = "";

  // Manual control: a mixed group whose rows bind to two different
  // targets. Manual {vel,turn} are operator preferences owned by the
  // browser (ManualPrefs, persisted to localStorage) — the firmware
  // never sees them; the arrow-key handlers in main.ts read them and
  // translate held keys into setTarget / setTurn WS messages. The two
  // α filters live in DeviceParams (firmware schema) and are pushed
  // via setParam. Per-row target/onChange overrides keep the rendering
  // pipeline single-pass while letting each row route its writes
  // correctly.
  const manualRowOnChange = (_: string, __: any) => onManualPrefChange();
  const manualGroup: GroupSpec = {
    title: "Manual control",
    rows: [
      { kind: "num", label: "Manual vel (↑/↓)", path: "control.manualVelocity",
        opts: { min: 0, max: 3, step: 0.05, unit: "m/s", decimals: 2 },
        tooltip: "Magnitude (m/s) sent as setTarget while ↑/↓ are held. ↑ = +, ↓ = −. Firmware clamps the value to vMaxCart on receive.",
        target: manualPrefs, onChange: manualRowOnChange },
      { kind: "num", label: "targetV α", path: "targetVAlpha",
        opts: { min: 0, max: 1, step: 0.01, decimals: 3 },
        tooltip: "1-pole IIR on the raw shared targetV before the controller consumes it (both outer-loop err and inner-loop velFF·targetV see the filtered value). 1.0 = passthrough — joystick lands on the controller verbatim every tick. Lower = the bot ramps gradually into a new commanded velocity instead of jumping. Filter state resets to 0 on disarm and IMU-not-ready, so every arm ramps from rest toward the live joystick value. Trade-off: lower α = smoother but slower joystick response.",
        target: deviceParams, onChange: onParamChange },
      { kind: "num", label: "Manual turn (←/→)", path: "control.manualTurn",
        opts: { min: 0, max: 1.5, step: 0.05, unit: "m/s", decimals: 2 },
        tooltip: "Steering differential magnitude (m/s) sent as setTurn while ←/→ are held. → = +turn (left wheel faster, bot turns right), ← = −turn. Firmware clamps to vMaxTurn and applies the differential after the vWheelAlpha filter.",
        target: manualPrefs, onChange: manualRowOnChange },
      { kind: "num", label: "targetTurn α", path: "targetTurnAlpha",
        opts: { min: 0, max: 1, step: 0.01, decimals: 3 },
        tooltip: "1-pole IIR on the raw shared targetTurn before the controller applies the steering differential. Same recurrence and reset rules as targetV α (resets to 0 on disarm / IMU-not-ready). Independent of targetV α so steering and forward-velocity ramps tune separately. 1.0 = passthrough; lower α makes the bot ease into a turn rate.",
        target: deviceParams, onChange: onParamChange },
      { kind: "num", label: "stick deadband", path: "stickDeadband",
        opts: { min: 0, max: 0.5, step: 0.005, decimals: 3 },
        tooltip: "Symmetric dead zone applied to both joystick axes (leftY → targetV, rightX → targetTurn) in firmware/src/joystick.cpp. Normalized half-width: 0.05 = 5% on either side of center. Translated form — inside the band output is exactly 0; just past the edge output STARTS at 0 and ramps up (no jump to ±deadband). Soaks up the DualSense's at-rest analog jitter without making small intentional movements feel sluggish. 0 disables (raw stick goes straight into the expo curve)." },
      { kind: "num", label: "stick expo", path: "stickExpo",
        opts: { min: 1, max: 5, step: 0.05, decimals: 2 },
        tooltip: "Response-curve exponent applied AFTER the deadband. Output = sign(x)·|x|^expo on the [0..1]-remapped stick. 1.0 = linear (no curve). 2.0 (default) = gentle expo — tiny stick at low end → tiny velocity, full stick → full vMax, with accelerating sensitivity in between. 3.0+ = aggressive expo. Same exponent applies to both axes so steering and throttle feel matched. The curve runs ONCE before scaling by vMaxCart / vMaxTurn." },
    ],
  };
  // The container target/onChange below are unused for these rows because
  // each row carries its own overrides — pass deviceParams + onParamChange
  // as harmless defaults.
  const syncMap: SyncMap = new Map();
  for (const [k, v] of buildPanelGroups(container, deviceParams, [manualGroup], onParamChange)) {
    syncMap.set(k, v);
  }
  for (const [k, v] of buildPanelGroups(container, deviceParams, deviceGroups, onParamChange)) {
    syncMap.set(k, v);
  }

  // Device-only actions group.
  const ag = document.createElement("div");
  ag.className = "group";
  const aHeader = document.createElement("h3");
  aHeader.textContent = "Device actions";
  aHeader.addEventListener("click", () => ag.classList.toggle("collapsed"));
  ag.appendChild(aHeader);
  const aBody = document.createElement("div");
  aBody.className = "group-body";
  aBody.style.display = "flex";
  aBody.style.flexWrap = "wrap";
  aBody.style.gap = "6px";

  const mkBtn = (label: string, title: string, fn: () => void): HTMLButtonElement => {
    const b = document.createElement("button");
    b.textContent = label;
    b.title = title;
    b.addEventListener("click", fn);
    return b;
  };
  aBody.appendChild(mkBtn("Arm",            "Send {type:enableMotors,value:true} — safety FSM ARM request",  actions.onArm));
  aBody.appendChild(mkBtn("Disarm",         "Send {type:enableMotors,value:false} — safety FSM DISARM",       actions.onDisarm));
  aBody.appendChild(mkBtn("Save params",    "Send {type:saveParams} — persist to NVS",                        actions.onSave));
  aBody.appendChild(mkBtn("Load defaults",  "Send {type:loadDefaults} — reset firmware ControlParams",        actions.onLoadDefaults));

  // Trim capture: averages telemetered theta over ~1 s and writes the
  // result to thetaTrim. Hold the bot physically upright before pressing.
  // Button label and disabled state are managed by the action via the
  // setLabel callback so the user sees a live "Capturing…" countdown.
  const trimBtn = mkBtn(
    "Capture trim",
    "Hold bot upright; samples θ for ~1 s and writes the mean to thetaTrim",
    () => {
      const orig = trimBtn.textContent ?? "Capture trim";
      actions.onCaptureTrim((text, busy) => {
        trimBtn.textContent = text;
        trimBtn.disabled = busy;
        if (!busy) {
          // Restore the canonical label after a brief result display.
          setTimeout(() => { trimBtn.textContent = orig; }, 1500);
        }
      });
    },
  );
  aBody.appendChild(trimBtn);

  // Calibrate IMU: device-side 2 s gyro-bias measurement. Refused by the
  // firmware while motors are armed (would lose theta updates and drop
  // the bot mid-balance). Result is persisted to NVS by the firmware
  // before the ack returns, so the user does NOT need to press "Save
  // params" afterwards.
  const calBtn = mkBtn(
    "Calibrate IMU",
    "Hold bot still (and disarmed); device samples gyro for ~2 s and persists the bias",
    () => {
      const orig = calBtn.textContent ?? "Calibrate IMU";
      actions.onCalibrate((text, busy) => {
        calBtn.textContent = text;
        calBtn.disabled = busy;
        if (!busy) {
          setTimeout(() => { calBtn.textContent = orig; }, 2000);
        }
      });
    },
  );
  aBody.appendChild(calBtn);

  ag.appendChild(aBody);
  container.appendChild(ag);
  return syncMap;
}

// Apply a flat key/value snapshot (the firmware's {type:"params"}.params
// envelope) into both `target` (the local DeviceParams object that
// other parts of the app read) and the live DOM (via the per-row sync
// closures in `syncMap`). Inputs that currently hold focus are
// untouched by sync — the in-progress edit wins. Local target is
// always updated; when the user blurs, their `change` handler will
// overwrite target with their value and fire setParam, which is the
// correct "last writer wins" behaviour.
export function applyDeviceSnapshot(
  syncMap: SyncMap,
  target: any,
  snapshot: Record<string, any>,
): void {
  for (const [path, value] of Object.entries(snapshot)) {
    // Only touch keys the panel actually rendered. Device may carry
    // fields the schema doesn't model yet; those go into target
    // anyway via the outer Object.assign in main.ts, but they have no
    // visual representation so syncMap won't have an entry.
    setPath(target, path, value);
    const sync = syncMap.get(path);
    if (sync) sync(value);
  }
}
