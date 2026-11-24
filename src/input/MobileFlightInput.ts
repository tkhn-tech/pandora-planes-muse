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

/** Pitch-only touch-stick scale. Pass 14 (worker A): the squared
 *  curve (150/ias)^2, floored at 0.12, was too weak to fly with —
 *  a full drag DOWN at 240 kt commanded pitchIn 0.39 and pitched
 *  the jet down 4.4 deg in 5 s at altitude (probe-pass14 E2b),
 *  while the keyboard at pitchIn 0.96 pitched 34 deg in the same
 *  time; the v4.8 live check read that as "pitch locked" through
 *  24-45 s full-down holds. The pitch axis needs a STRONGER curve
 *  than roll (a pull-up's G builds with v * pitch-rate), but not
 *  the squared one: (200/ias)^1.5, floored at 0.15 — 1.0 at/below
 *  200 kt, ~0.76 at 240 kt, ~0.58 at 287 kt, ~0.20 at 586 kt.
 *  The 586 kt full-drag-up transient stays inside the probe-roll
 *  E2 band (G@0.5s <= 5): the squared curve measured 1.3 G there,
 *  and the new 0.20 scale measures G@0.5s 1.3, peak 1.8 G
 *  (probe-roll E2, fa18 586 kt, full drag up). A full drag down at
 *  240 kt now pitches 12.5 deg in 5 s at altitude (probe-pass14
 *  E2b, was 4.4 deg). The keyboard keeps full pitch authority
 *  (ArrowUp = full rate) for when the pilot wants it. */
export function touchPitchScale(iasKt: number): number {
  if (!Number.isFinite(iasKt) || iasKt <= 150) return 1;
  return Math.max(0.15, Math.min(1, (200 / iasKt) ** 1.5));
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
