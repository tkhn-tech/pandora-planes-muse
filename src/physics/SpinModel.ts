// SpinModel — the spin as a YAW-RATE state (BET report §2), shared by
// the curve solver and the blade-element solver so both spin the same
// way.
//
// Real F/A-18: departure-resistant; a sustained spin needs pro-spin
// inputs (full rudder + differential surfaces + asymmetric thrust,
// yaw rates to 90 deg/s in test entries). NATOPS spin recovery
// (A1-F18AC-NFM-500): controls released, feet off rudders, throttles
// idle → if a command arrow is present, lateral stick FULL WITH ARROW;
// if no arrow, spin-recovery switch to RCVY then lateral stick full
// with the arrow → when yaw stops, stick smoothly neutral → recover
// when AOA/yaw tones stop and speed > 180 KCAS; if dive recovery has
// not started by 6,000 ft AGL, eject. A spin recovery costs roughly
// 1,500–3,000 ft.
//
// Model: `spin` 0..1 builds while stalled AND yawed (pro-spin rudder
// or asymmetric wing damage), decays ~0.5/s on neutral controls and
// faster (1.5/s) with the correct anti-spin input (rudder opposite the
// spin + nose down). While spinning, the solver adds a yaw rate up to
// ~90 deg/s in the spin direction plus a coupled roll rate, and a
// lateral drift force. The HUD command arrow (spinArrow) points the
// anti-spin lateral-stick direction — opposite the spin's yaw — and
// shows only when spin > 0.5.

const SPIN_BUILD = 0.8; // per second while stalled and yawed
const SPIN_DECAY_NEUTRAL = 0.5; // per second on neutral controls
const SPIN_DECAY_ANTISPIN = 1.5; // per second with correct recovery input
export const SPIN_YAW_RATE_DEG = 90; // max spin yaw rate, deg/s
export const SPIN_ROLL_RATE_DEG = 55; // coupled roll rate, deg/s
const SPIN_SIDE_ACCEL = 6; // m/s^2 lateral drift at full spin

export class SpinModel {
  /** Spin amount 0..1. */
  spin = 0;
  /** Spin direction: +1 = yawing right (nose right), −1 = yawing left. */
  spinDir: 1 | -1 = 1;

  /**
   * Step the spin state for one fixed step.
   *
   * @param dt fixed-step length, s
   * @param stalled wing-section stall (AoA past the linear region)
   * @param yawIn rudder input −1..1 (positive = right rudder)
   * @param pitchIn stick pitch −1..1 (negative = pull up)
   * @param rollIn stick roll −1..1
   * @param asymWing (wingR − wingL) damage, −1..1 — asymmetric damage
   *   can start a spin without rudder input (a dead left wing gives
   *   a negative value and spins the nose left, toward the dead wing).
   * @returns the spin yaw rate (rad/s) and roll rate (rad/s) to add to
   *   the solver's rates, plus the lateral drift acceleration (m/s²)
   *   for the force sum. All zero when not spinning.
   */
  step(
    dt: number,
    stalled: boolean,
    yawIn: number,
    pitchIn: number,
    rollIn: number,
    asymWing: number,
  ): { yawRate: number; rollRate: number; sideAccel: number } {
    const D2R = Math.PI / 180;
    const neutral =
      Math.abs(yawIn) < 0.1 && Math.abs(pitchIn) < 0.1 && Math.abs(rollIn) < 0.1;
    const antiSpin =
      Math.sign(yawIn) === -this.spinDir && yawIn !== 0 && pitchIn > 0.1;
    if (stalled && (Math.abs(yawIn) > 0.2 || Math.abs(asymWing) > 0.3)) {
      // Pro-spin: rudder into the spin, or a dead wing dragging.
      if (Math.abs(yawIn) > 0.2) this.spinDir = yawIn > 0 ? 1 : -1;
      else if (Math.abs(asymWing) > 0.3) this.spinDir = asymWing > 0 ? -1 : 1;
      this.spin = Math.min(1, this.spin + dt * SPIN_BUILD);
    } else if (antiSpin) {
      this.spin = Math.max(0, this.spin - dt * SPIN_DECAY_ANTISPIN);
    } else if (neutral || !stalled) {
      this.spin = Math.max(0, this.spin - dt * SPIN_DECAY_NEUTRAL);
    } else {
      this.spin = Math.max(0, this.spin - dt * SPIN_DECAY_NEUTRAL * 0.5);
    }
    if (this.spin <= 0.01) return { yawRate: 0, rollRate: 0, sideAccel: 0 };
    return {
      yawRate: this.spinDir * this.spin * SPIN_YAW_RATE_DEG * D2R,
      rollRate: this.spinDir * this.spin * SPIN_ROLL_RATE_DEG * D2R,
      sideAccel: -this.spin * SPIN_SIDE_ACCEL,
    };
  }

  /** HUD command arrow: −1 = push the stick left, +1 = right, 0 = none.
   *  Shown only when the spin is developed (spin > 0.5); the arrow
   *  points opposite the spin's yaw (the anti-spin stick direction). */
  get arrow(): -1 | 0 | 1 {
    if (this.spin <= 0.5) return 0;
    return this.spinDir === 1 ? -1 : 1;
  }

  /** Reset the spin state (respawn — a fresh jet is not spinning). */
  reset(): void {
    this.spin = 0;
    this.spinDir = 1;
  }
}
