// CurveAerodynamicsSolver — the v2 flight model, ported exactly from
// pandora-template.html ~958–1075 (Aircraft.step).
//
// v2 constants (do not tune without re-running the takeoff check):
//   CL = 2π·0.9·α + 0.15 + (flaps/30)·0.35, stall at 15° + (flaps/30)·3°
//   past stall: CL = 1.35 − (α−stall)·0.045, clamped to [-0.8, 1.5]
//   CD = 0.028 + 0.06·CL² + (flaps/30)·0.06 + airbrake·0.12 + gear·0.03
//   ground effect (h < span, v > 30): CL ×1.15, CD ×0.9
//   ρ = 1.225·exp(−alt/8500), q = ½ρv²
//   thrust spool 0.55/s lives in the CONTROLLER; the solver takes the
//   spooled throttle in AeroState.throttle.
//   VTOL: thrustDir = fwd·cos(vec) + up·sin(vec); at 90° throttle is lift.
//   rotary: rotorLift = throttle·thrust·(1 + clamp(v/40,0,1)·0.3);
//   autorotation: idle throttle + v > 20 keeps 0.35·thrust lift.
//   G-load = lift / weight, floored at 0.2.
//
// v1.1 AoA fix (the takeoff killer): AoA is the BODY-frame angle
// atan2(−vBody.y, vBody.z) from the AIR velocity (ground velocity minus
// wind). v1 used asin(vDir·up), which returned NEGATIVE AoA when the nose
// was above the velocity — CL went negative and the jet was pressed into
// the runway with −811 kN of downforce at 189 m/s.
//
// TORQUE. The v2 template has no aerodynamic moment model — rotation is
// rate authority (pitch 55 deg/s, yaw 22 deg/s, roll spec.roll deg/s,
// scaled by qAuth). This solver therefore returns the rate command as a
// body-frame torque, torque = I·rate / TAU (TAU = 0.25 s), with a
// per-axis inertia estimate from mass / span / length. The controller
// recovers the rates with torqueToRates() and sets them as the Rapier
// body's angular velocity. torqueToRates(torque) is the exact inverse,
// so the loop reproduces the template's rates bit-for-bit at any dt.
//
// Differences vs the template's two scalars (flagged, deliberate):
//   * wingDmg was one scalar, so asymmetric roll was always same-signed.
//     Here asym roll uses (wingL − wingR): symmetric damage flies
//     straight, one-sided damage rolls toward the dead side. Authority
//     loss still uses the average.
//   * engineDmg was one scalar. Here thrust uses the L/R average and the
//     single-engine-out yaw uses (engineR − engineL): a dead left engine
//     yaws left, toward the dead engine.

import * as THREE from 'three';
import {
  controlPitch,
  controlRoll,
  controlYaw,
  type AeroResult,
  type AeroState,
  type DamageZones,
  type FlightControls,
  type IAerodynamicsSolver,
} from './IAerodynamicsSolver';

const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;

/** v2 CL/CD constants. */
const CL_SLOPE = 2 * Math.PI * 0.9;
const CL_0 = 0.15;
const CL_MAX = 1.5;
const CL_MIN = -0.8;
const STALL_BASE_DEG = 15;
const STALL_CL = 1.35;
const STALL_DROP = 0.045;
const STALL_HYSTERESIS_DEG = 3;
const FLAP_CL = 0.35;
const FLAP_STALL_SHIFT_DEG = 3;
const CD_0 = 0.028;
const CD_INDUCED = 0.06;
const CD_FLAP = 0.06;
const CD_AIRBRAKE = 0.12;
const CD_GEAR = 0.03;
const GE_CL = 1.15;
const GE_CD = 0.9;
const RHO_0 = 1.225;
const RHO_SCALE_H = 8500;
const G = 9.81;

/** Rate-authority constants (v2). */
const PITCH_RATE_DEG = 55;
const YAW_RATE_DEG = 22;
const ROLL_COORD_DEG = -8; // auto-coordinated turn: roll feeds yaw
const ENGINE_OUT_YAW_DEG = 5; // single-engine-out yaw, deg/s
const STALL_NOSE_DROP = 4.0; // v2: pitchRate -= 0.5·dt·8 per step
const SPIN_BUILD = 0.8; // per second while stalled and yawed
const SPIN_DECAY = 0.5; // per second otherwise
const SPIN_SIDE_ACCEL = 6; // m/s^2 lateral drift at full spin
const ASYM_ROLL = 0.9; // roll rate per unit wing damage, × qAuth
const WING_DMG_ROLL_AUTHORITY = 0.5; // roll authority loss per unit damage
const ENGINE_DMG_THRUST = 0.7; // thrust loss per unit engine damage
const HYDRO_PITCH = 0.4; // pitch authority with hydraulic failure
const ROTOR_TRANSLIFT = 0.3; // translational lift gain, v/40 clamped 0..1
const ROTOR_DRAG = 0.05; // extra CD term for the rotor disc
const ROTOR_THRUST_EFF = 0.15; // forward thrust fraction for a heli

/** Torque response time: torque = I·rate / TAU. */
export const TORQUE_TAU = 0.25;

export interface Inertia {
  /** About body X (pitch). */
  ix: number;
  /** About body Y (yaw). */
  iy: number;
  /** About body Z / forward axis (roll). */
  iz: number;
}

/** Slender-body inertia estimate from mass / span / length. */
export function inertias(mass: number, span: number, length: number): Inertia {
  return {
    ix: (mass * length * length) / 12,
    iy: (mass * (span * span + length * length)) / 12,
    iz: (mass * span * span) / 12,
  };
}

/** Inverse of the solver's torque model: recover the v2 pitch / yaw /
 *  roll rates (rad/s) from a body-frame torque. */
export function torqueToRates(
  torque: THREE.Vector3,
  mass: number,
  span: number,
  length: number,
): { pitchRate: number; yawRate: number; rollRate: number } {
  const i = inertias(mass, span, length);
  return {
    pitchRate: (torque.x * TORQUE_TAU) / i.ix,
    yawRate: (torque.y * TORQUE_TAU) / i.iy,
    rollRate: (torque.z * TORQUE_TAU) / i.iz,
  };
}

function clamp(v: number, a: number, b: number): number {
  return Math.max(a, Math.min(b, v));
}

export class CurveAerodynamicsSolver implements IAerodynamicsSolver {
  /** Stall hysteresis (v2: stalled clears only below stall−3°). */
  private stalled = false;
  /** Spin amount 0..1. */
  private spin = 0;

  solve(state: AeroState): AeroResult {
    const dt = state.dt ?? 1 / 60;
    const m = state.mass;
    const S = state.wingArea;
    const span = state.span;
    const chord = state.chord ?? S / span;
    void chord; // chord is accepted for future moment models; v2 has none.
    const length = state.length ?? span * 1.4;
    const dmg: DamageZones = state.damage;
    const wingDmgAvg = (dmg.wingL + dmg.wingR) / 2;
    const engineDmgAvg = (dmg.engineL + dmg.engineR) / 2;
    const controls: FlightControls | undefined = state.controls;
    const pitchIn = controls ? controlPitch(controls) : 0;
    const rollIn = controls ? controlRoll(controls) : 0;
    const yawIn = controls ? controlYaw(controls) : 0;
    const auth = state.controlAuthority ?? { turn: 1, rollDeg: 150, pitch: 1 };
    const fail = state.failures ?? {};

    const vAir = state.velocityAir;
    const v = vAir.length();
    const quat = state.quat;
    const fwd = new THREE.Vector3(0, 0, 1).applyQuaternion(quat);
    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(quat);
    const right = new THREE.Vector3(1, 0, 0).applyQuaternion(quat);

    // --- AoA (v1.1): body-frame angle between forward axis and the AIR
    // velocity, in the pitch plane. vBody.y < 0 (air arriving from below
    // the nose) is positive AoA. ---
    let aoaDeg = 0;
    if (v > 5) {
      const vBody = vAir.clone().applyQuaternion(quat.clone().invert());
      aoaDeg = Math.atan2(-vBody.y, vBody.z) * R2D;
    }
    const aoaR = aoaDeg * D2R;

    // --- Density / dynamic pressure ---
    const alt = Math.max(state.altitude, 0);
    const rho =
      state.density > 0 ? state.density : RHO_0 * Math.exp(-alt / RHO_SCALE_H);
    const q = 0.5 * rho * v * v;

    // --- CL curve with stall break + hysteresis ---
    const flapFrac = state.flapsDeg / 30;
    let cl = CL_SLOPE * aoaR + CL_0 + flapFrac * FLAP_CL;
    const stallA = STALL_BASE_DEG + flapFrac * FLAP_STALL_SHIFT_DEG;
    if (aoaDeg > stallA) {
      const ex = aoaDeg - stallA;
      cl = STALL_CL - ex * STALL_DROP;
      this.stalled = true;
    } else if (aoaDeg < stallA - STALL_HYSTERESIS_DEG) {
      this.stalled = false;
    }
    cl = clamp(cl, CL_MIN, CL_MAX);

    // --- CD: parasitic + induced + flaps + airbrake + gear ---
    let cd =
      CD_0 +
      CD_INDUCED * cl * cl +
      flapFrac * CD_FLAP +
      state.airbrake * CD_AIRBRAKE +
      (state.gearDown ? CD_GEAR : 0);

    // --- Ground effect inside one wingspan of the surface ---
    if (
      state.heightAboveGround !== undefined &&
      state.heightAboveGround < span &&
      v > 30
    ) {
      cl *= GE_CL;
      cd *= GE_CD;
    }

    let lift = q * S * cl;
    let drag = q * S * cd;

    // --- Thrust (spooled throttle arrives in state.throttle) ---
    let thrust =
      state.throttle *
      state.thrustMax *
      (1 - engineDmgAvg * ENGINE_DMG_THRUST) *
      (1 - 0.5 * (fail.engineOut ? 1 : 0));
    let thrustEff = thrust;

    // --- Rotary archetype: rotor-disc lift replaces wing lift ---
    if (state.archetype === 'rotary') {
      let rotorLift =
        state.throttle * state.thrustMax * (1 + clamp(v / 40, 0, 1) * ROTOR_TRANSLIFT);
      if (state.throttle < 0.15 && v > 20) {
        // Autorotation: idle throttle + 20 m/s keeps ~35% lift.
        rotorLift = Math.max(rotorLift, 0.35 * state.thrustMax);
      }
      lift = rotorLift;
      drag += q * S * ROTOR_DRAG;
      thrustEff = thrust * ROTOR_THRUST_EFF;
    }

    // --- Force vectors (world frame) ---
    const liftVec = new THREE.Vector3();
    const dragVec = new THREE.Vector3();
    const thrustVec = new THREE.Vector3();
    const vec =
      state.archetype === 'vtol' ? state.vectorDeg * D2R : 0;
    const thrustDir = fwd
      .clone()
      .multiplyScalar(Math.cos(vec))
      .add(up.clone().multiplyScalar(Math.sin(vec)));
    thrustVec.addScaledVector(thrustDir, thrustEff);
    if (v > 1) {
      liftVec.addScaledVector(up, lift);
      dragVec.addScaledVector(vAir.clone().normalize(), -drag);
    }

    // --- G-load (lift / weight, floored at 0.2) ---
    const gLoad = Math.max(0.2, lift / (m * G));

    // --- Rotation: rate authority scales with q ---
    const qAuth =
      (state.archetype === 'rotary'
        ? clamp(Math.max(v, 25) / 60, 0.4, 1.1)
        : clamp(v / 60, 0.25, 1.15)) * auth.turn;
    const pitchAuth = (fail.hydroDegraded ? HYDRO_PITCH : 1) * auth.pitch;
    const rollRate =
      rollIn * auth.rollDeg * D2R * qAuth * (1 - wingDmgAvg * WING_DMG_ROLL_AUTHORITY);
    let pitchRate = pitchIn * PITCH_RATE_DEG * D2R * qAuth * pitchAuth;
    // Stall: nose drops, wing-drop + spin entry if yawed while stalled.
    // v2 applies the drop per step as pitchRate -= 0.5·dt·8 (= −4·dt);
    // at the 60 Hz fixed step that is −0.067 rad/s. Reproduced exactly.
    if (this.stalled) {
      pitchRate -= STALL_NOSE_DROP * dt;
      this.spin = clamp(
        this.spin + (Math.abs(yawIn) > 0.2 ? dt * SPIN_BUILD : -dt * SPIN_DECAY),
        0,
        1,
      );
      if (this.spin > 0.3) {
        // v2: vel += right · (−spin·6·dt) — as a force on the drag vector.
        dragVec.addScaledVector(right, -this.spin * SPIN_SIDE_ACCEL * m);
      }
    } else {
      this.spin = Math.max(0, this.spin - dt * SPIN_DECAY);
    }
    // Single-engine-out yaw: toward the dead engine (v2 failEngineSide).
    const engineSide =
      fail.engineSide ?? clamp(dmg.engineR - dmg.engineL, -1, 1);
    const yawRate =
      (yawIn * YAW_RATE_DEG * D2R +
        rollIn * ROLL_COORD_DEG * D2R +
        engineSide * ENGINE_OUT_YAW_DEG * D2R) *
      qAuth;
    // Wing damage: asymmetric roll (per-side in this port).
    const asymRoll = (dmg.wingL - dmg.wingR) * ASYM_ROLL * qAuth;

    // --- Torque = I·rate / TAU (body frame: x pitch, y yaw, z roll) ---
    const inertia = inertias(m, span, length);
    const torque = new THREE.Vector3(
      (inertia.ix * pitchRate) / TORQUE_TAU,
      (inertia.iy * yawRate) / TORQUE_TAU,
      (inertia.iz * (rollRate + asymRoll)) / TORQUE_TAU,
    );

    return {
      lift: liftVec,
      drag: dragVec,
      thrust: thrustVec,
      torque,
      aoaDeg,
      cl,
      cd,
      stalled: this.stalled,
      spin: this.spin,
      gLoad,
    };
  }
}
