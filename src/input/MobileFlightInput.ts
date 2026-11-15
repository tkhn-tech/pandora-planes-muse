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
      if (Math.abs(db) > 3) pitch = clamp(-db / scale, -1, 1);
      if (Math.abs(dg) > 3) roll = clamp(-dg / scale, -1, 1);
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
