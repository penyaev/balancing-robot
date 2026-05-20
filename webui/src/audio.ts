// audio.ts — WebSocket → Web Audio bridge for the coproc /audio stream.
//
// Receives 16-bit little-endian PCM chunks (16 kHz mono) from the
// coproc at ws://<host>/audio, decodes them to Float32, and routes
// them to an AudioWorklet that ring-buffers and plays them through
// the default audio output.
//
// Lifecycle:
//   * Construct once at startup. No WebSocket is opened yet.
//   * start() (called when the operator clicks "Unmute" in the UI)
//     opens the AudioContext + registers the worklet + opens the WS.
//     A user gesture is required because all browsers block autoplay
//     until the user has interacted with the page.
//   * stop() closes the WS, suspends the AudioContext, and zeroes the
//     ring. start() can be called again later — the worklet stays
//     registered.
//   * setHost() updates the URL the next start() will use; if currently
//     running, restarts.
//
// Why an AudioWorklet rather than ScriptProcessorNode: the latter is
// deprecated and runs on the main thread (UI jank → audio glitch).
// AudioWorklet runs on a dedicated audio thread and is the supported
// path on every browser we care about.

// The worklet code lives in this same file as a string template, packed
// into a Blob, registered via URL.createObjectURL. Avoids a second
// build artifact and keeps the WS↔worklet glue colocated. The worklet
// has its own `globalThis` — no access to closures from audio.ts.
const WORKLET_SRC = `
class PcmPlayer extends AudioWorkletProcessor {
  constructor() {
    super();
    // Ring buffer of Float32 samples. Sized for ~250 ms at 16 kHz —
    // generous enough to absorb WiFi jitter without piling up so much
    // latency that the audio sounds laggy.
    this.RING = 4096;
    this.ring = new Float32Array(this.RING);
    this.rp = 0;
    this.wp = 0;
    this.count = 0;
    // For the level meter: peak |amplitude| over the last block.
    this.peak = 0;
    this.peakReportEvery = 16;   // ~16 blocks (~42 ms at 48 kHz) between posts
    this.peakReportTick = 0;
    this.port.onmessage = (ev) => {
      // ev.data is a Float32Array of incoming samples (already at the
      // SAME rate as the AudioContext — we resample on the JS side
      // before posting so the worklet's process() output cadence
      // matches AudioContext.sampleRate exactly).
      const s = ev.data;
      for (let i = 0; i < s.length; i++) {
        this.ring[this.wp] = s[i];
        this.wp = (this.wp + 1) % this.RING;
        if (this.count < this.RING) this.count++;
        else this.rp = (this.rp + 1) % this.RING; // overwrite oldest
      }
    };
  }
  process(_inputs, outputs) {
    const out = outputs[0][0]; // mono → first channel
    let p = 0;
    for (; p < out.length && this.count > 0; p++) {
      const v = this.ring[this.rp];
      out[p] = v;
      this.rp = (this.rp + 1) % this.RING;
      this.count--;
      const av = v < 0 ? -v : v;
      if (av > this.peak) this.peak = av;
    }
    // Underrun → output silence for the rest of the block.
    for (; p < out.length; p++) out[p] = 0;
    // Periodically post the running peak back to the main thread for
    // the level meter, then reset it. Don't post on every block — that
    // floods the messaging channel for a UI redraw that only happens
    // at ~30 fps anyway.
    this.peakReportTick++;
    if (this.peakReportTick >= this.peakReportEvery) {
      this.port.postMessage({ peak: this.peak });
      this.peak = 0;
      this.peakReportTick = 0;
    }
    return true;
  }
}
registerProcessor("pcm-player", PcmPlayer);
`;

export class AudioBridge {
  private ctx: AudioContext | null = null;
  private node: AudioWorkletNode | null = null;
  private gain: GainNode | null = null;
  private ws: WebSocket | null = null;
  private running = false;
  private muted = true;
  private host: string;
  private lastPeak = 0;
  private peakListeners: Array<(peak: number) => void> = [];
  private statusListeners: Array<(label: string) => void> = [];

  // Source sample rate on the wire — must match coproc/src/audio.cpp.
  private static readonly SRC_SR = 16000;

  constructor(host: string) {
    this.host = host;
  }

  setHost(h: string): void {
    this.host = h;
    if (this.running) {
      // Restart against the new host.
      this.stop();
      // Don't auto-start; the user explicitly unmuted once already, so
      // re-arming is appropriate. Caller can also pre-empt with stop().
      void this.start();
    }
  }

  getHost(): string { return this.host; }
  isRunning(): boolean { return this.running; }
  getPeak(): number { return this.lastPeak; }

  onPeak(cb: (peak: number) => void): void { this.peakListeners.push(cb); }
  onStatus(cb: (label: string) => void): void { this.statusListeners.push(cb); }

  private setStatus(label: string): void {
    for (const cb of this.statusListeners) cb(label);
  }

  // Open the AudioContext + WS. Must be called from a user gesture
  // handler — the AudioContext stays suspended otherwise and silence is
  // the only audible output.
  async start(): Promise<void> {
    if (this.running) return;
    this.running = true;
    this.setStatus("connecting…");

    try {
      // 1) AudioContext + worklet.
      if (!this.ctx) {
        this.ctx = new AudioContext({ latencyHint: "interactive" });
        const blob = new Blob([WORKLET_SRC], { type: "application/javascript" });
        const url  = URL.createObjectURL(blob);
        await this.ctx.audioWorklet.addModule(url);
        URL.revokeObjectURL(url);
      }
      if (this.ctx.state === "suspended") await this.ctx.resume();

      // 2) Build the node + gain chain. Gain at 0 == muted (start that
      // way; the UI unmutes after we report "live").
      this.node = new AudioWorkletNode(this.ctx, "pcm-player", {
        outputChannelCount: [1],
      });
      this.node.port.onmessage = (ev) => {
        const peak = (ev.data && typeof ev.data.peak === "number")
          ? ev.data.peak : 0;
        this.lastPeak = peak;
        for (const cb of this.peakListeners) cb(peak);
      };
      this.gain = this.ctx.createGain();
      this.gain.gain.value = this.muted ? 0 : 1;
      this.node.connect(this.gain).connect(this.ctx.destination);

      // 3) WebSocket. binaryType=arraybuffer so we get an ArrayBuffer
      // (not a Blob) and can DataView straight into it.
      const url = `ws://${this.host}/audio`;
      this.ws = new WebSocket(url);
      this.ws.binaryType = "arraybuffer";
      this.ws.addEventListener("open", () => this.setStatus("live"));
      this.ws.addEventListener("error", () => this.setStatus("error"));
      this.ws.addEventListener("close", () => {
        if (this.running) this.setStatus("closed");
      });
      this.ws.addEventListener("message", (ev) => this.onMessage(ev));
    } catch (e) {
      this.setStatus(`error: ${String(e)}`);
      this.running = false;
    }
  }

  stop(): void {
    if (!this.running) return;
    this.running = false;
    if (this.ws) {
      try { this.ws.close(); } catch { /* ignore */ }
      this.ws = null;
    }
    if (this.node) {
      try { this.node.disconnect(); } catch { /* ignore */ }
      this.node = null;
    }
    if (this.gain) {
      try { this.gain.disconnect(); } catch { /* ignore */ }
      this.gain = null;
    }
    if (this.ctx) {
      // Suspend rather than close — closing would mean another addModule
      // call on the next start(). Suspending keeps the worklet
      // registered and start() becomes cheap.
      void this.ctx.suspend();
    }
    this.lastPeak = 0;
    this.setStatus("idle");
  }

  setMuted(m: boolean): void {
    this.muted = m;
    if (this.gain) this.gain.gain.value = m ? 0 : 1;
  }
  isMuted(): boolean { return this.muted; }

  private onMessage(ev: MessageEvent): void {
    if (!(ev.data instanceof ArrayBuffer)) return;
    if (!this.node || !this.ctx) return;
    const buf = ev.data;
    // Reinterpret as int16 little-endian (the coproc uses native LE).
    const i16 = new Int16Array(buf);
    // Convert int16 → float32 in [-1, 1] AND resample from 16 kHz
    // (source) to the AudioContext rate (typically 48 kHz). Linear
    // interpolation is plenty for voice-band audio.
    const dstRate = this.ctx.sampleRate;
    const ratio = dstRate / AudioBridge.SRC_SR;
    const dstLen = Math.floor(i16.length * ratio);
    const f32 = new Float32Array(dstLen);
    for (let i = 0; i < dstLen; i++) {
      const srcIdx = i / ratio;
      const i0 = Math.floor(srcIdx);
      const i1 = Math.min(i16.length - 1, i0 + 1);
      const frac = srcIdx - i0;
      const s0 = i16[i0] / 32768;
      const s1 = i16[i1] / 32768;
      f32[i] = s0 + (s1 - s0) * frac;
    }
    this.node.port.postMessage(f32, [f32.buffer]);
  }
}
