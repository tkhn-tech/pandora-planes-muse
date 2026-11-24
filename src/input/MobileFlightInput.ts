// MobileFlightInput — DeviceOrientation tilt steering for phones/tablets.
// Ported from the v2 template tilt block (pandora-template.html ~1755–1767
// toggleTilt + playerInput tilt merge):
//   pitch: (beta - beta0), roll: (gamma - gamma0), deadzone 3 deg,
//   scale /25, clamped to +-1. Top tilts back toward you = nose up.
// The touch stick itself is wired in main.ts from the DOM (joy/joyKnob),
// as in v2 — this provider only owns tilt. When tilt is off (or the
// device has no orientation sensor) getControls() returns zero stick, so
// main falls back to the touch stick.

import type { FlightControls } from '../physics/IAerodynamicsSolver';
import type { IFlightInputProvider } from './IFlightInputProvider';

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

/** On-screen joystick (touch stick, wired in main.ts) authority scale
 *  vs indicated airspeed, pass 12 worker D, curve steepened pass 13
 *  worker B. At low speed the stick is 1:1; at high speed its
 *  authority falls as 110/ias (floored at 0.15), so a full drag at
 *  586 kt no longer commands the ±10 G / instant-loop response the
 *  v4.6.1 live check saw — the stick's FEEL stays roughly constant
 *  across the speed range instead of getting twitchier as q rises.
 *  The pass-12 curve (150/ias, floor 0.30) was still too strong
 *  live: a full 1.5 s drag at 700 kt rolled the F-35A 92 deg
 *  (measured, probe-roll section F — the v4.7 live check saw
 *  59–81 deg from 1.5–2 s drags), because the BET roll rate at full
 *  deflection is ~150 deg/s at that speed and 0.30 of it is still
 *  ~45 deg/s, plus ~13 deg of post-release roll inertia. 110/ias
 *  puts a full drag at ~25 deg/s at 700 kt, close to a full
 *  keyboard hold there (kbRollScale, ~30 deg/s). main.ts applies
 *  this in the touch branch of the playerInput merge (patch spec —
 *  MobileFlightInput only owns tilt; the touch stick itself lives
 *  in main.ts). */
export function touchStickScale(iasKt: number): number {
  if (!Number.isFinite(iasKt) || iasKt <= 150) return 1;
  return Math.max(0.15, Math.min(1, 110 / iasKt));
}

/** Pitch-only touch-stick scale. Pitch needs a STRONGER curve than
 *  roll: probe-roll section E measured that the linear 150/ias scale
 *  (0.30 at 586 kt) only moved a full drag-up's G@0.5 s from 9.9 G
 *  to 8.5 G — a pull-up's G builds with v * pitch-rate, and at
 *  586 kt even a 0.3-stick pull reaches high G in half a second,
 *  because the AoA the jet integrates makes huge lift at that q.
 *  The squared curve (0.27 at 287 kt, 0.12 at 450+ kt) keeps a
 *  0.5 s full drag under ~5 G at any speed while a 2 s drag still
 *  loops the jet — it tames the gain, it does not neuter the axis.
 *  The keyboard keeps full pitch authority (ArrowUp = full rate)
 *  for when the pilot wants it. */
export function touchPitchScale(iasKt: number): number {
  if (!Number.isFinite(iasKt) || iasKt <= 150) return 1;
  return Math.max(0.12, Math.min(1, (150 / iasKt) ** 2));
}

/** On-screen joystick deadzone (stick units): a resting finger or a
 *  5% offset must not roll/pitch the jet. Rescaled so full deflection
 *  is still full deflection. */
export function touchDeadzone(v: number): number {
  const DZ = 0.06;
  if (Math.abs(v) < DZ) return 0;
  return (v - Math.sign(v) * DZ) / (1 - DZ);
}

export class MobileFlightInput implements IFlightInputProvider {
  private enabled = false;
  private have = false;
  private beta = 0;
  private gamma = 0;
  private beta0 = 0;
  private gamma0 = 0;
  private kind: 'fixed-wing' | 'vtol' | 'rotary' = 'fixed-wing';

  constructor() {
    window.addEventListener('deviceorientation', this.onOrientation);
  }

  /** iOS 13+ needs an explicit user-gesture permission request
   *  (DeviceOrientationEvent.requestPermission). Resolves when granted;
   *  rejects when denied / unsupported. On grant, tilt turns on and the
   *  current attitude becomes neutral (calibrate). */
  async requestPermission(): Promise<void> {
    const doe = DeviceOrientationEvent as unknown as {
      requestPermission?: () => Promise<string>;
    };
    if (typeof DeviceOrientationEvent !== 'undefined' && doe.requestPermission) {
      const result = await doe.requestPermission();
      if (result !== 'granted') {
        throw new Error('Motion permission was not granted.');
      }
    }
    this.calibrate();
    this.enabled = true;
  }

  /** Toggle tilt on/off (the tiltBtn handler in main.ts). When turning on
   *  without the iOS permission flow, requestPermission() runs first. */
  async toggle(): Promise<boolean> {
    if (!this.enabled) {
      await this.requestPermission();
    } else {
      this.enabled = false;
    }
    return this.enabled;
  }

  get isEnabled(): boolean {
    return this.enabled;
  }

  get hasSensor(): boolean {
    return this.have;
  }

  setAircraftKind(kind: 'fixed-wing' | 'vtol' | 'rotary'): void {
    this.kind = kind;
  }

  /** Re-centre: the current attitude becomes the neutral stick position.
   *  Template: "hold level, then tap again to re-centre". */
  calibrate(): void {
    this.beta0 = this.beta;
    this.gamma0 = this.gamma;
  }

  getControls(): FlightControls {
    let pitch = 0;
    let roll = 0;
    if (this.enabled && this.have) {
      const db = this.beta - this.beta0;
      const dg = this.gamma - this.gamma0;
      // VTOL hover benefits from a gentler tilt response.
      const scale = this.kind === 'vtol' ? 32 : 25;
      // Pass 11: both signs were inverted. Beta: top tilts BACK
      // toward you (db < 0) = nose up = pitchIn < 0, so pitch =
      // +db/scale. Gamma: device tilts RIGHT (dg > 0) = roll right
      // = rollIn > 0 (the physics maps rollIn > 0 to a right
      // bank), so roll = +dg/scale.
      if (Math.abs(db) > 3) pitch = clamp(db / scale, -1, 1);
      if (Math.abs(dg) > 3) roll = clamp(dg / scale, -1, 1);
    }
    return { pitch, roll, yaw: 0, throttle: null };
  }

  private readonly onOrientation = (e: DeviceOrientationEvent): void => {
    if (e.beta !== null) {
      this.beta = e.beta;
      this.gamma = e.gamma ?? 0;
      this.have = true;
    }
  };

  dispose(): void {
    window.removeEventListener('deviceorientation', this.onOrientation);
    this.enabled = false;
  }
}
