// Core shared types for the bot web UI.

// Kinematic state the charts / HUD read out of a telemetry Snapshot.
//   x       : axle horizontal position [m] (integrated from xDot)
//   xDot    : axle horizontal velocity [m/s]
//   theta   : chassis tilt from vertical [rad]
//   thetaDot: chassis tilt rate [rad/s]
//   phi     : wheel rotation angle [rad] — unused on device, kept 0
export interface State {
  x: number;
  xDot: number;
  theta: number;
  thetaDot: number;
  phi: number;
}

// Operator manual-control preferences. Browser-owned — the firmware
// never sees these; the arrow-key handlers in main.ts read them to
// size the setTarget / setTurn commands sent while a key is held.
// Persisted to localStorage so they survive a reload.
export interface ManualPrefs {
  control: {
    manualVelocity: number; // [m/s] magnitude sent while ↑/↓ held
    manualTurn: number;     // [m/s] differential sent while ←/→ held
  };
}

export const defaultManualPrefs: ManualPrefs = {
  control: { manualVelocity: 0.05, manualTurn: 0.1 },
};
