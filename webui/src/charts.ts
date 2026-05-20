// Live scrolling time-series charts using Chart.js.
//
// We disable all animations and use the 'linear' x scale with raw {x,y} points so we
// can mutate data arrays in place and call chart.update('none') for an animation-free
// repaint. Each chart owns ring-buffer-style sliding windows trimmed by simTime.

import {
  Chart,
  LineController,
  LineElement,
  PointElement,
  LinearScale,
  Tooltip,
  Legend,
  Title,
  ChartConfiguration,
  Filler,
} from "chart.js";
import zoomPlugin from "chartjs-plugin-zoom";

Chart.register(LineController, LineElement, PointElement, LinearScale, Tooltip, Legend, Title, Filler, zoomPlugin);

// Format a y-axis tick value with decimals derived from the tick spacing.
// Avoids Chart.js' default behavior of falling back to scientific notation for
// values very close to zero (e.g. floating-point dust like 3.5e-17 on a zero
// crossing of a scale stepping by 0.1).
function fmtYTick(value: any, _index: number, ticks: any[]): string {
  const v = Number(value);
  if (!Number.isFinite(v)) return String(value);
  let step = 1;
  if (ticks && ticks.length > 1) {
    // Use the median absolute gap to be robust against duplicate edge ticks.
    const gaps: number[] = [];
    for (let i = 1; i < ticks.length; i++) {
      const g = Math.abs(Number(ticks[i].value) - Number(ticks[i - 1].value));
      if (g > 0) gaps.push(g);
    }
    if (gaps.length > 0) {
      gaps.sort((a, b) => a - b);
      step = gaps[Math.floor(gaps.length / 2)];
    }
  }
  // Treat values much smaller than the tick spacing as exactly zero.
  if (Math.abs(v) < step * 1e-6) return "0";
  // Decimals: 0 if step >= 1, else enough to resolve the step (capped at 6).
  let decimals = 0;
  if (step < 1) {
    decimals = Math.min(6, Math.max(0, -Math.floor(Math.log10(step))));
  }
  return v.toFixed(decimals);
}

// Reasonable global defaults for a dark, dense dashboard look.
Chart.defaults.color = "#8a93a3";
Chart.defaults.borderColor = "#3a3f4844";
Chart.defaults.font.family = "ui-monospace, Menlo, monospace";
Chart.defaults.font.size = 10;
Chart.defaults.animation = false;
Chart.defaults.responsive = true;
Chart.defaults.maintainAspectRatio = false;

export interface SeriesDef {
  label: string;
  stroke: string;
  // Optional y-axis id (must match a key in ChartConfig.yAxes). Defaults to "y".
  yAxis?: string;
}

export interface YAxisDef {
  id: string;
  label?: string;
  // "left" (default for the primary axis) or "right" (default for extras)
  position?: "left" | "right";
  // Optional unit suffix used in legend/tooltip for series bound to this axis.
  unit?: string;
}

export interface ChartConfig {
  title: string;
  yLabel: string;
  series: SeriesDef[];
  windowSec: number;
  // Optional unit suffix appended to formatted y-values in the legend and tooltip.
  unit?: string;
  // Optional additional y-axes. The default axis "y" is always present using yLabel/unit.
  // Provide entries here to add more (e.g. a right-hand axis with a different scale).
  yAxes?: YAxisDef[];
}

interface Point { x: number; y: number; }

export class LiveChart {
  private chart: Chart<"line", Point[]>;
  private windowSec: number;
  private buffers: Point[][];
  private lastT = 0;

  // Registry of all live LiveChart instances, used to implement
  // cross-chart synchronised tooltips: hovering on chart A finds the
  // nearest sample by x (= simulation/device time) in every other
  // chart's buffers and programmatically shows that chart's tooltip at
  // the same time. Lets the operator cross-correlate a transient across
  // multiple charts (e.g. "what was vWheelCmdR doing at the moment θ
  // crossed zero?") without having to read off tSec and eyeball each
  // chart manually.
  //
  // Implementation note: we drive synchronisation off real DOM
  // mousemove/mouseleave events on each canvas (NOT off Chart.js'
  // onHover, which would re-fire when we programmatically set active
  // elements on a sibling and create a feedback loop). Programmatic
  // setActiveElements does not synthesise DOM events, so the broadcast
  // is one-way per real hover.
  private static instances: LiveChart[] = [];

  constructor(canvas: HTMLCanvasElement, cfg: ChartConfig) {
    this.windowSec = cfg.windowSec;
    this.buffers = cfg.series.map(() => []);

    const datasets = cfg.series.map((s, i) => ({
      label: s.label,
      data: this.buffers[i],
      borderColor: s.stroke,
      backgroundColor: s.stroke,
      borderWidth: 1.25,
      pointRadius: 0,
      pointHoverRadius: 3,
      tension: 0,
      // spanGaps in same units as x (seconds, since we feed tSec). Bridge
      // single-frame hiccups but break the line when there's a real gap
      // — disconnects, paused-simulator frame holds, etc. 0.1 s = ~6
      // frames at 60 Hz, comfortable margin over inter-frame jitter.
      spanGaps: 0.1,
      parsing: false as const, // we feed {x,y} directly
      yAxisID: s.yAxis ?? "y",
    }));

    // Format a number compactly for legends/tooltips: enough precision to read,
    // but never wider than ~8 chars so the legend doesn't reflow.
    const defaultUnit = cfg.unit ?? "";
    const axisUnits: Record<string, string> = { y: defaultUnit };
    for (const ax of cfg.yAxes ?? []) axisUnits[ax.id] = ax.unit ?? defaultUnit;
    // Uniform 2-decimal formatting for legend/tooltip values. Tiny
    // numbers like 2.3e-11 are noise from float math; rendering them as
    // "0.00" is more readable while tuning. Large numbers still get two
    // decimals (e.g. "1234.56") — chart axis labels handle ranging.
    // Uniform 2-decimal formatting for legend/tooltip values. Tiny
    // numbers like 2.3e-11 are noise from float math; rendering them as
    // "0.00" is more readable while tuning. Large numbers still get two
    // decimals (e.g. "1234.56") — chart axis labels handle ranging.
    const fmtBody = (v: number): string => v.toFixed(2);
    const fmtForAxis = (v: number, axisId: string): string => {
      if (!Number.isFinite(v)) return "—";
      return fmtBody(v) + (axisUnits[axisId] ?? defaultUnit);
    };

    const config: ChartConfiguration<"line", Point[]> = {
      type: "line",
      data: { datasets },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        normalized: true,
        // Chart-level fallback in case a dataset is added without its
        // own spanGaps override. Same 100 ms threshold rationale.
        spanGaps: 0.1,
        // Nearest-x crosshair: hovering snaps to the closest sample on the x axis
        // and shows all series in the tooltip at that time.
        interaction: { mode: "index", intersect: false, axis: "x" },
        plugins: {
          title: {
            display: true,
            text: cfg.title,
            color: "#d8dde6",
            font: { size: 11, weight: "bold" },
            padding: { top: 0, bottom: 2 },
          },
          legend: {
            display: true,
            position: "top",
            align: "end",
            // Click-to-toggle. Chart.js' default click handler reads
            // legendItem.datasetIndex (which our generateLabels supplies)
            // and flips dataset visibility — but spelling it out here is
            // defensive against future Chart.js changes and lets us also
            // crosshatch the disabled label in generateLabels via
            // hidden:true (Chart.js auto-renders hidden labels with
            // strikethrough text + dimmed swatch). The toggle persists
            // for the lifetime of the LiveChart instance: redraw() goes
            // through chart.update("none") which preserves per-dataset
            // visibility metadata, and reset() doesn't touch it either.
            // Effect: fewer accidental "lost" series when the operator
            // hides one to declutter and then resets the chart.
            onClick: (_e, legendItem, legend) => {
              const idx = legendItem.datasetIndex;
              if (idx === undefined) return;
              const ch = legend.chart;
              ch.setDatasetVisibility(idx, !ch.isDatasetVisible(idx));
              ch.update("none");
            },
            // Crosshair cursor on legend items so the click affordance
            // is discoverable.
            onHover: (e) => {
              const tgt = (e.native?.target as HTMLElement | undefined);
              if (tgt) tgt.style.cursor = "pointer";
            },
            onLeave: (e) => {
              const tgt = (e.native?.target as HTMLElement | undefined);
              if (tgt) tgt.style.cursor = "default";
            },
            labels: {
              color: "#d8dde6",
              boxWidth: 8,
              boxHeight: 2,
              padding: 6,
              font: { size: 10 },
              // Append the most recent y value to each label.
              generateLabels: (chart) => {
                const items: any[] = [];
                chart.data.datasets.forEach((ds: any, i: number) => {
                  const buf = ds.data as Point[];
                  const last = buf.length ? buf[buf.length - 1].y : NaN;
                  const axisId = ds.yAxisID ?? "y";
                  items.push({
                    text: `${ds.label}: ${fmtForAxis(last, axisId)}`,
                    fillStyle: ds.borderColor,
                    strokeStyle: ds.borderColor,
                    fontColor: "#d8dde6",
                    lineWidth: 0,
                    hidden: !chart.isDatasetVisible(i),
                    datasetIndex: i,
                  });
                });
                return items;
              },
            },
          },
          tooltip: {
            enabled: true,
            mode: "index",
            intersect: false,
            axis: "x",
            backgroundColor: "#1a1d22ee",
            borderColor: "#3a3f48",
            borderWidth: 1,
            titleColor: "#d8dde6",
            bodyColor: "#d8dde6",
            titleFont: { size: 10 },
            bodyFont: { size: 10, family: "ui-monospace, Menlo, monospace" },
            padding: 6,
            callbacks: {
              title: (items) => {
                if (!items.length) return "";
                const x = (items[0].raw as Point).x;
                return `t = ${x.toFixed(2)} s`;
              },
              label: (item) => {
                const y = (item.raw as Point).y;
                const axisId = (item.dataset as any).yAxisID ?? "y";
                return `${item.dataset.label}: ${fmtForAxis(y, axisId)}`;
              },
            },
          },
          // Interactive zoom/pan. Wheel = zoom around cursor (x+y),
          // shift-drag = box zoom, plain drag is left to chart.js
          // defaults (no pan) so it doesn't fight tooltip hover. Double
          // click resets zoom (see canvas dblclick handler below).
          // While the user is zoomed, the live-scroll auto-pin in
          // redraw() is suppressed so newly-pushed data doesn't yank
          // the view back to the leading edge.
          zoom: {
            limits: {
              // Don't allow zooming further than ~10x of the window in
              // either direction; keeps single-step zooms feeling
              // bounded and avoids degenerate empty views.
              x: { minRange: cfg.windowSec / 100 },
            },
            pan: {
              enabled: true,
              mode: "xy",
              modifierKey: "shift",
            },
            zoom: {
              wheel: { enabled: true, speed: 0.1 },
              pinch: { enabled: true },
              drag: {
                enabled: true,
                modifierKey: "alt",
                backgroundColor: "rgba(78,161,255,0.15)",
                borderColor: "#4ea1ff",
                borderWidth: 1,
              },
              mode: "xy",
            },
          },
        },
        scales: (() => {
          const scales: Record<string, any> = {
            x: {
              type: "linear",
              min: -cfg.windowSec,
              max: 0,
              bounds: "data",
              ticks: {
                maxRotation: 0,
                autoSkip: true,
                maxTicksLimit: 6,
                callback: (v: any) => Number(v).toFixed(1),
              },
              grid: { color: "#3a3f4844" },
            },
            y: {
              type: "linear",
              position: "left",
              title: { display: true, text: cfg.yLabel, color: "#8a93a3", font: { size: 10 } },
              ticks: { maxTicksLimit: 5, callback: fmtYTick as any },
              grid: { color: "#3a3f4844" },
            },
          };
          for (const ax of cfg.yAxes ?? []) {
            scales[ax.id] = {
              type: "linear",
              position: ax.position ?? "right",
              title: ax.label
                ? { display: true, text: ax.label, color: "#8a93a3", font: { size: 10 } }
                : { display: false },
              ticks: { maxTicksLimit: 5, callback: fmtYTick as any },
              // Only the primary axis draws gridlines; extra axes draw nothing on the
              // plot area to avoid visual clutter.
              grid: { drawOnChartArea: false, color: "#3a3f4844" },
            };
          }
          return scales;
        })(),
      },
    };

    this.chart = new Chart(canvas, config);

    // Double-click anywhere on the chart canvas resets any active
    // wheel/pinch/box zoom and re-engages the live-scroll auto-pin in
    // redraw() (it skips while the user is zoomed; clearing the zoom
    // releases that gate).
    canvas.addEventListener("dblclick", () => {
      (this.chart as any).resetZoom?.();
    });

    // Cross-chart tooltip sync. See LiveChart.instances comment for
    // rationale. We attach DOM listeners (not Chart.js onHover) so
    // programmatic setActiveElements on siblings doesn't re-trigger
    // the broadcast.
    LiveChart.instances.push(this);
    canvas.addEventListener("mousemove", (e) => {
      const rect = canvas.getBoundingClientRect();
      const px = e.clientX - rect.left;
      // Translate pixel → data x (= time in seconds) using THIS chart's
      // x scale. Each chart's x scale shares the same source clock
      // (simTime / device tSec), so the value is directly meaningful as
      // an index key on every other chart's buffers.
      const xVal = this.chart.scales.x?.getValueForPixel?.(px);
      if (xVal === undefined || !Number.isFinite(xVal)) return;
      for (const inst of LiveChart.instances) {
        if (inst === this) continue;
        inst.showSyncTooltipAtX(xVal);
      }
    });
    canvas.addEventListener("mouseleave", () => {
      for (const inst of LiveChart.instances) {
        if (inst === this) continue;
        inst.clearSyncTooltip();
      }
    });
  }

  // Programmatically display this chart's "index" tooltip at the sample
  // closest to the given x value (in source-clock seconds). Called by
  // sibling LiveCharts on hover. Does nothing if no visible dataset has
  // any data — leaves whatever tooltip state the chart already had
  // (the sibling will clear via clearSyncTooltip on mouseleave).
  private showSyncTooltipAtX(x: number): void {
    const datasets = this.chart.data.datasets;
    const active: { datasetIndex: number; index: number }[] = [];
    let posPx = -1;
    let posPy = -1;
    for (let di = 0; di < datasets.length; di++) {
      if (!this.chart.isDatasetVisible(di)) continue;
      const buf = this.buffers[di];
      if (!buf || buf.length === 0) continue;
      // Linear nearest-neighbour search. Buffers are sliding 30 s
      // windows at ~100 Hz → ~3000 points worst case; this is sub-ms
      // per chart and runs only on real mousemove events. If we ever
      // need more, the buffer is monotonic in x so a binary search
      // would be a drop-in upgrade.
      let bestIdx = 0;
      let bestDist = Math.abs(buf[0].x - x);
      for (let i = 1; i < buf.length; i++) {
        const d = Math.abs(buf[i].x - x);
        if (d < bestDist) { bestDist = d; bestIdx = i; }
      }
      active.push({ datasetIndex: di, index: bestIdx });
      // Use the first dataset's pixel position to anchor the tooltip
      // box. Chart.js needs *some* (x,y) pixel coordinate for the
      // tooltip's caret; using the first active element keeps the
      // caret on a real point of a real visible series.
      if (posPx < 0) {
        const meta = this.chart.getDatasetMeta(di);
        const elem = meta.data[bestIdx] as any;
        if (elem && Number.isFinite(elem.x) && Number.isFinite(elem.y)) {
          posPx = elem.x;
          posPy = elem.y;
        }
      }
    }
    if (active.length === 0 || posPx < 0) return;
    this.chart.setActiveElements(active);
    if (this.chart.tooltip) {
      this.chart.tooltip.setActiveElements(active, { x: posPx, y: posPy });
    }
    // 'none' = no transition; the next animation-frame redraw paints
    // the tooltip overlay. Cheap because options.normalized + no
    // animation skips the data-recalc path.
    this.chart.update("none");
  }

  // Remove the synchronised-tooltip overlay this chart was showing on
  // behalf of a sibling's hover. Idempotent.
  private clearSyncTooltip(): void {
    this.chart.setActiveElements([]);
    if (this.chart.tooltip) {
      this.chart.tooltip.setActiveElements([], { x: 0, y: 0 });
    }
    this.chart.update("none");
  }

  push(tSec: number, values: number[]): void {
    this.lastT = tSec;
    const cutoff = tSec - this.windowSec;
    for (let i = 0; i < this.buffers.length; i++) {
      const buf = this.buffers[i];
      const v = values[i];
      buf.push({ x: tSec, y: Number.isFinite(v) ? v : NaN });
      // Trim from front. For modest buffers (~600 pts) shift is fast enough; for large
      // ones we'd swap to a circular buffer. Currently fine.
      while (buf.length > 0 && buf[0].x < cutoff) buf.shift();
    }
  }

  redraw(): void {
    // Pin the visible x range to exactly the data window so the line spans the full
    // chart width even before the buffer is full, and the leading edge never floats.
    // Skip the pin while the user has an active zoom/pan — otherwise newly-pushed
    // points would constantly snap the view back to the leading edge, defeating
    // the whole point of zooming in to inspect a region.
    if (!(this.chart as any).isZoomedOrPanned?.()) {
      const xScale = this.chart.options.scales!.x!;
      xScale.min = this.lastT - this.windowSec;
      xScale.max = this.lastT;
    }
    // 'none' skips the animation transition for an instant repaint.
    this.chart.update("none");
  }

  // Update the legend labels of existing series in place. Lengths must
  // match; missing entries are left untouched. Used to repurpose chart
  // semantics across source modes (e.g. "θ (meas)" in sim becomes
  // "θ (effective)" in device mode where the second slot carries
  // theta - thetaTrim instead of a separate noisy measurement).
  setSeriesLabels(labels: string[]): void {
    const datasets = this.chart.data.datasets;
    for (let i = 0; i < Math.min(labels.length, datasets.length); i++) {
      datasets[i].label = labels[i];
    }
    this.chart.update("none");
  }

  reset(): void {
    for (const buf of this.buffers) buf.length = 0;
    this.lastT = 0;
    // Clear any active wheel/pinch/box zoom so the freshly-cleared
    // chart starts from the live-scroll auto-pin again instead of
    // hanging on a stale zoomed-in view of empty buffers.
    (this.chart as any).resetZoom?.();
    const xScale = this.chart.options.scales!.x!;
    xScale.min = -this.windowSec;
    xScale.max = 0;
    this.chart.update("none");
  }
}
