// CurveAerodynamicsSolver — the v2 flight model, ported exactly from
// pandora-template.html ~958–1075 (Aircraft.step), then corrected to the
// BET research report (Docs/flight-behaviour-research.md) and the
// per-model supplement (Docs/flight-behaviour-per-model.md).
//
// v2 constants kept (do not tune without re-running the takeoff check):
//   CL slope / flap increment / ground effect / ISA density / VTOL /
//   rotary branch / G-overstress path.
//   CD = 0.028 + k·CL² + (flaps/30)·0.06 + airbrake·0.12 + gear·0.03
//   ground effect (h < span, v > 30): CL ×1.15, CD ×0.9
//   ρ = 1.225·exp(−alt/8500), q = ½ρv²
//   thrust spool 0.55/s lives in the CONTROLLER; the solver takes the
//   spooled throttle in AeroState.throttle.
//
// BET corrections applied here (report §0–§4, §6; supplement §1–§6):
//   §0  induced drag k is PER SPEC (AeroState.inducedK = 1/(π·e·AR),
//       F-18 0.100, not the shared 0.06) + thrust lapse (ρ/ρ₀)^0.7.
//       Sustained-turn check (F-18, 20,242 kg, 400 kt SL): drag =
//       thrust → CL 1.16 → n 7.05 g → ω 19.0°/s vs published 19.2°/s.
//   §1  coordinated yaw from ACTUAL bank and speed: ω = g·tan φ/V
//       (φ from the quaternion), capped at the sustained-turn envelope
//       (sustainedTurnRate) — replaces the fixed −8°/s ROLL_COORD fudge.
//       Sideslip β = atan2(vBody.x, vBody.z) + a weathercock term
//       driving yaw toward β = 0.
//   §2  per-model vortex-lift CL curve (AeroState.clCurve, shared
//       clFromCurve): linear to the section stall, rising to the peak
//       (F-18 1.6 at 35°, F-22 1.75 at 35°, F-15E 1.36 at 25° — sharp
//       stall, steep fall), then falling to the floor. Separation drag
//       CD_sep = cdSep·max(0,(α−20)/15)². Spin is a YAW-RATE state
//       (SpinModel, up to 90°/s) with NATOPS recovery (neutral controls
//       decay 0.5/s, opposite rudder + nose down decays 1.5/s) and a
//       HUD command arrow when spin > 0.5. The stall nose-drop is a
//       rate that grows past the CL peak, not a constant from the stall.
//   §3  rotation gates are per spec (AeroState has rotateSpeedKt — the
//       CONTROLLER enforces the 12°/s cap at Vr; the solver keeps the
//       q-scheduled 55°/s max authority). ROTATE cue is the controller's.
//   §4  adverse-yaw transient while rolling (nose away from the turn,
//       cancelled by rudder into the turn); Q/E rudder is wired through
//       main.ts (p.yawIn = pc.yaw) — the solver just uses yawIn.
//       MiG-29 roll authority is capped by its airspeed schedule
//       (AeroState.rollCapSchedule), not by q-scaling.
//   §6  qAuth floor REMOVED (0 at zero airspeed — no airflow, no
//       control); F-22 TVC keeps pitch authority at ~0 kt (tvcDeg).
//       gLoad = 1.0 grounded is the controller's override (the
//       solver's 0.2 floor stays). Ground friction 0.3 is the
//       controller's. Survival score gated on airborne is main.ts's.
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
// The BladeElementSolver is the one with a real moment model
// (ω̇ = I⁻¹(M − ω × Iω)); this curve solver stays the rate-authority
// reference, now with the per-model curves and the §1/§2/§4 physics.
//
// Differences vs the template's two scalars (flagged, deliberate):
//   * wingDmg was one scalar, so asymmetric roll was always same-signed.
//     Here asym roll uses (wingR − wingL): symmetric damage flies
//     straight, one-sided damage rolls toward the dead side — the
//     physical left wing is +X, so a dead left wing (wingL) must roll
//     the jet LEFT (negative roll rate), which (wingR − wingL) gives.
//     Authority loss still uses the average.
//   * engineDmg was one scalar. Here thrust uses the L/R average and the
//     single-engine-out yaw uses (engineL − engineR): a dead left engine
//     yaws left, toward the dead engine (positive yawRate = nose left).

import * as THREE from 'three';
import {
  clFromCurve,
  controlPitch,
  controlRoll,
  controlYaw,
  rollCapAtSpeed,
  sustainedTurnRate,
  type AeroResult,
  type AeroState,
  type DamageZones,
  type FlightControls,
  type IAerodynamicsSolver,
} from './IAerodynamicsSolver';
import { SpinModel } from './SpinModel';

const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;

/** v2 CL/CD constants (kept). The CL curve itself is per model now
 *  (AeroState.clCurve — see clFromCurve). */
const CL_MIN = -0.8;
const FLAP_CL = 0.35;
const FLAP_STALL_SHIFT_DEG = 3;
const CD_0 = 0.028;
const CD_FLAP = 0.06;
const CD_AIRBRAKE = 0.12;
const CD_GEAR = 0.03;
const GE_CL = 1.15;
const GE_CD = 0.9;
const RHO_0 = 1.225;
const RHO_SCALE_H = 8500;
const G = 9.81;
/** Default induced-drag k (F-18) when the state omits it. */
const DEFAULT_INDUCED_K = 0.1;
/** Default F-18 CL curve when the state omits it. */
const DEFAULT_CL_CURVE = {
  linearEndAoA: 15,
  linearEndCL: 1.35,
  peak: 1.6,
  peakAoA: 35,
  floor: 0.8,
  postSlope: 0.05,
  cdSep: 0.35,
};

/** Rate-authority constants (v2, kept). */
const PITCH_RATE_DEG = 55;
const YAW_RATE_DEG = 22;
const ENGINE_OUT_YAW_DEG = 5; // single-engine-out yaw, deg/s
const STALL_NOSE_DROP_RATE = 0.3; // pass 12: nose-drop RATE, rad/s.
// Was STALL_NOSE_DROP = 18.0 multiplied by dt at the use site:
// at the fixed dt=1/60 that gave 18/60 = 0.3 rad/s (the value the
// comment quoted), but at a live dt=0.05 s it gave 0.9 rad/s — 3x
// stronger at low FPS. The BET stall nose-drop is a dt-independent
// rate bias (BET_STALL_NOSE_DROP_RATE 0.3 in the controller); the
// curve solver's must be too, so the stall break is identical at
// fixed and live step sizes. The (1 + (aoaDeg - peak)/10) growth
// past the peak is unchanged.
const ASYM_ROLL = 0.9; // roll rate per unit wing damage, × qAuth
const WING_DMG_ROLL_AUTHORITY = 0.5; // roll authority loss per unit damage
const ENGINE_DMG_THRUST = 0.7; // thrust loss per unit engine damage
const HYDRO_PITCH = 0.4; // pitch authority with hydraulic failure
const ROTOR_TRANSLIFT = 0.3; // translational lift gain, v/40 clamped 0..1
const ROTOR_DRAG = 0.05; // extra CD term for the rotor disc
const ROTOR_THRUST_EFF = 0.15; // forward thrust fraction for a heli
/** Adverse yaw: yaw rate per unit roll rate, scaled by the induced-drag
 *  share of total drag (BET §4, k_adv ≈ 0.15). Sign: rollRate > 0 =
 *  rolling RIGHT (Euler YXZ z > 0; the right wing is -X), and adverse
 *  yaw yaws the nose AWAY from the turn — rolling right, the nose yaws
 *  LEFT (positive yawRate: +ω.y swings the nose toward +X, the left
 *  wing) — so the term is +sign(rollRate)·k·|rollRate|. Pass 11: the
 *  old comment had rollRate > 0 as rolling left; the code was right. */
const ADV_YAW_K = 0.15;
/** Weathercock: yaw rate per radian of sideslip β, driving β → 0
 *  (BET §1 — the curve-model stand-in for Cn_β). */
const WEATHERCOCK = 1.5;
/** F-22 TVC: fraction of full pitch authority kept at zero airspeed
 *  with full throttle (BET §2 — TVC makes pitch authority nearly
 *  airspeed-independent at low speed). */
const TVC_AUTH = 0.6;

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

/** Slender-body inertia estimate from mass / span / length. The roll
 *  inertia (iz) carries a 0.5 factor: the uniform-rod m·span²/12
 *  assumes the mass is spread evenly along the span, but a
 *  fighter's mass is concentrated in the fuselage — the wings are
 *  light — so the true roll inertia is roughly half the rod
 *  value (the F/A-18's published Ixx is ~half the uniform-rod
 *  estimate). The old factor-1.0 iz made the BET roll far too
 *  sluggish: the aileron moment could only bank ~2.4 deg in the
 *  first 0.2 s (probe-roll needs >5 deg), while the curve solver
 *  banks ~30 deg. The curve solver is invariant to this: its
 *  rates are prescribed and torqueToRates() is the exact inverse
 *  of its torque model, so changing iz changes its torque and its
 *  recovered rate by the same factor. Pitch (ix) and yaw (iy)
 *  keep the rod estimate — their calibrations (takeoff rotation,
 *  max turns, stall) were done with it. */
export function inertias(mass: number, span: number, length: number): Inertia {
  return {
    ix: (mass * length * length) / 12,
    iy: (mass * (span * span + length * length)) / 12,
    iz: ((mass * span * span) / 12) * 0.5,
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
  /** Spin as a yaw-rate state (BET §2). */
  private readonly spinModel = new SpinModel();

  /** Reset stall/spin hysteresis (respawn — a fresh jet is not
   *  stalled and not spinning, whatever the wreck was doing; the
   *  v2 play-test saw the STALL warning survive three restarts). */
  reset(): void {
    this.stalled = false;
    this.spinModel.reset();
  }

  solve(state: AeroState): AeroResult {
    const dt = state.dt ?? 1 / 60;
    const m = state.mass;
    const S = state.wingArea;
    // Lift reference area: the MiG-29's blended fuselage + LERX make
    // ~40% of total lift, so lift uses 45 m² while drag uses 38 m²
    // (supplement §5). Everyone else: lift area = wing area.
    const S_lift = state.wingAreaLift ?? S;
    const span = state.span;
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
    const curve = state.clCurve ?? DEFAULT_CL_CURVE;
    const inducedK = state.inducedK ?? DEFAULT_INDUCED_K;

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
    let betaDeg = 0;
    if (v > 5) {
      const vBody = vAir.clone().applyQuaternion(quat.clone().invert());
      aoaDeg = Math.atan2(-vBody.y, vBody.z) * R2D;
      // Sideslip β (BET §1): positive = air arriving from the right
      // (nose left of the velocity vector).
      betaDeg = Math.atan2(vBody.x, vBody.z) * R2D;
    }

    // --- Density / dynamic pressure ---
    const alt = Math.max(state.altitude, 0);
    const rho =
      state.density > 0 ? state.density : RHO_0 * Math.exp(-alt / RHO_SCALE_H);
    const q = 0.5 * rho * v * v;

    // --- Per-model CL curve (BET §2) with stall break + hysteresis ---
    // The stall FLAG is the PEAK-CL stall (AoA > peakAoA, clears below
    // peakAoA - 3 deg) — NOT the section stall at linearEndAoA. The
    // section stall (~15-18 deg) is where the wing root first
    // separates, but the vortex-lift region (linearEndAoA -> peakAoA)
    // keeps CL rising past it, so the aircraft flies on past the
    // section stall. The old flag fired the STALL warning at 15-18
    // deg AoA — 130-150 kt in a turn, where the real jet is flying
    // fine on vortex lift (issue 6). The nose-drop / spin already
    // live past the peak (line ~397), so the flag now matches them.
    const flapFrac = state.flapsDeg / 30;
    const stallA = curve.peakAoA + flapFrac * FLAP_STALL_SHIFT_DEG;
    let cl = clFromCurve(aoaDeg, flapFrac, curve);
    if (aoaDeg > stallA) this.stalled = true;
    else if (aoaDeg < stallA - 3) this.stalled = false;
    cl = clamp(cl, CL_MIN, curve.peak + flapFrac * FLAP_CL);

    // --- CD: parasitic (wing area) + induced (lift area, per-spec k)
    // + separation drag past 20° AoA + flaps + airbrake + gear ---
    const cdSep =
      curve.cdSep * Math.pow(Math.max(0, (aoaDeg - 20) / 15), 2);
    let cd =
      CD_0 +
      flapFrac * CD_FLAP +
      state.airbrake * CD_AIRBRAKE +
      (state.gearDown ? CD_GEAR : 0);
    // Ground effect inside one wingspan of the surface.
    let geCL = 1;
    let geCD = 1;
    if (
      state.heightAboveGround !== undefined &&
      state.heightAboveGround < span &&
      v > 30
    ) {
      geCL = GE_CL;
      geCD = GE_CD;
    }
    const clEff = cl * geCL;
    const cdEff =
      (cd + (S_lift / S) * inducedK * cl * cl + cdSep) * geCD;

    // Lift scale (damage-model §6 step 1): a DYING airframe (the
    // controller passes liftScale ~0.3) can no longer hold 1 g — the
    // lift is cut, the drag is NOT (a broken wing has high drag), so
    // the jet freefalls instead of gliding on with zero controls.
    const liftScale = state.liftScale ?? 1;
    let lift = q * S_lift * clEff * liftScale;
    // Induced + separation drag ride on the lift area; parasitic on the
    // wing area (the MiG-29 split — everyone else S_lift == S).
    let drag =
      q * S * cd * geCD + q * S_lift * (inducedK * cl * cl + cdSep) * geCD;

    // --- Thrust (spooled throttle arrives in state.throttle) ---
    // Thrust lapse (BET §0): T = T_max·(ρ/ρ₀)^0.7 — the F-18's 196 kN
    // is ~134 kN at 15,000 ft, which is what makes the sustained turn
    // rate fall from 19.0°/s SL to 12.7°/s there (published 12.3°/s).
    const lapse = Math.pow(Math.max(rho / RHO_0, 0.05), 0.7);
    let thrust =
      state.throttle *
      state.thrustMax *
      lapse *
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
      // LIFT DIRECTION (pass 11): perpendicular to the airflow, NOT
      // the body-up axis. The body-up axis sits sin(AoA) aft of the
      // perpendicular to the flight path, so applying the lift along
      // it put a spurious L·sin(AoA) component backward along the
      // flight path — ~20% of the lift at a 12 deg turn AoA, more
      // than the thrust — which capped every sustained turn ~25%
      // below the §0 band and bled speed in any high-AoA manoeuvre.
      // The magnitude is unchanged (q·S·CL); only the direction is
      // corrected: the component of body-up perpendicular to vAir.
      // Rotary keeps body-up (rotor thrust is along the mast).
      let liftDir = up;
      if (state.archetype !== 'rotary') {
        const vHat = vAir.clone().normalize();
        const perp = up.clone().addScaledVector(vHat, -up.dot(vHat));
        if (perp.lengthSq() > 0.01) liftDir = perp.normalize();
      }
      liftVec.addScaledVector(liftDir, lift);
      dragVec.addScaledVector(vAir.clone().normalize(), -drag);
    }

    // --- G-load (lift / weight, floored at 0.2) ---
    // The controller overrides this to 1.0 when grounded (BET §6).
    const gLoad = Math.max(0.2, lift / (m * G));

    // --- Rotation: rate authority scales with q ---
    // qAuth floor REMOVED (BET §2/§6): clamp(v/60, 0, 1.15) — no
    // airflow = no control. v2 kept 25% authority at zero airspeed.
    let qAuth =
      (state.archetype === 'rotary'
        ? clamp(Math.max(v, 25) / 60, 0.4, 1.1)
        : clamp(v / 60, 0, 1.15)) * auth.turn;
    const pitchAuth = (fail.hydroDegraded ? HYDRO_PITCH : 1) * auth.pitch;
    // F-22 TVC (supplement §3): the nozzles give pitch authority at
    // ~0 kt — a floor that does not depend on q, scaled by throttle.
    // At zero airspeed with full throttle the F-22 keeps ~60% pitch
    // authority; at 60 m/s the TVC term is gone (the wing does it).
    let qAuthPitch = qAuth;
    if (state.tvcDeg) {
      const tvcAuth =
        (state.tvcDeg / 20) * TVC_AUTH * state.throttle * (1 - clamp(v / 60, 0, 1));
      qAuthPitch = Math.max(qAuth, tvcAuth);
    }
    // F-35A hover FCS (Issue 2): at high thrust-vector the lift fan +
    // roll posts give control authority at ~zero airspeed, where the
    // wing has none (qAuth -> 0). Without this the jet cannot be
    // steered in hover at all — the v2 hover could only balance
    // vertically. Mirrors the F-22 TVC floor above: a throttle-scaled
    // floor that fades in from vector 45 deg (hoverBlend, the same
    // blend the BET solver uses) and is gone at vector 0 (forward
    // flight, where the wing does it). 60% authority at full vector +
    // full throttle; the rudder gets it too (yawRate uses qAuth).
    let hoverBlend = 0;
    if (state.archetype === 'vtol') {
      hoverBlend = clamp((state.vectorDeg - 45) / 45, 0, 1);
      const hoverAuth = 0.6 * hoverBlend * state.throttle;
      qAuth = Math.max(qAuth, hoverAuth);
      qAuthPitch = Math.max(qAuthPitch, hoverAuth);
    }
    // MiG-29 roll cap (supplement §5): the airspeed schedule caps the
    // roll rate — 20°/s near the stall, 105°/s at Mach 0.4, <60°/s at
    // high speed / low altitude, 40°/s supersonic. q-scaling alone gets
    // the top end backwards.
    const rollCapDeg = rollCapAtSpeed(v, state.rollCapSchedule);
    const rollRateMax = Math.min(auth.rollDeg, rollCapDeg) * D2R;
    let rollRate =
      rollIn * rollRateMax * qAuth * (1 - wingDmgAvg * WING_DMG_ROLL_AUTHORITY);
    let pitchRate = pitchIn * PITCH_RATE_DEG * D2R * qAuthPitch * pitchAuth;
    // Stall: past the CL peak the nose drops, and the drop grows with
    // how far past the peak the jet is (BET §2 — a real stall break
    // is a pitch acceleration of roughly −0.5 to −1 rad/s², nose
    // down). NOTE the sign: in this codebase a NEGATIVE pitchRate is
    // nose UP (pull = pitchIn < 0), so a nose-DOWN break ADDS to
    // pitchRate. v2 had this backwards (pitchRate -= 4·dt, a nose-up
    // bias when stalled) and the port reproduced it exactly; with the
    // per-model peak scaling that became a runaway — the F-35A flaps 0
    // pitched up past the peak, the "drop" pushed it further up, and
    // it departed to AoA −74.5°. The vortex-lift region (stall → peak)
    // is flyable, the break is past the peak.
    if (this.stalled && aoaDeg > curve.peakAoA) {
      pitchRate +=
        STALL_NOSE_DROP_RATE * (1 + (aoaDeg - curve.peakAoA) / 10);
    }
    // Spin (BET §2): a yaw-rate state, not a lateral drift. Builds
    // while stalled and yawed; NATOPS recovery (neutral controls, or
    // opposite rudder + nose down) decays it. The spin yaw/roll rates
    // are added to the control rates below.
    const spinStep = this.spinModel.step(
      dt,
      this.stalled,
      yawIn,
      pitchIn,
      rollIn,
      dmg.wingR - dmg.wingL,
    );
    if (this.spinModel.spin > 0.01) {
      // v2's lateral drift force, kept as a force on the drag vector.
      dragVec.addScaledVector(right, spinStep.sideAccel * m);
    }

    // --- Yaw: coordinated turn + weathercock + adverse yaw + rudder ---
    // Coordinated yaw (BET §1): ω = g·tan φ/V from the ACTUAL bank
    // and true airspeed — the nose follows the lift-tilted velocity
    // vector. φ is the bank from the quaternion (Euler YXZ z; positive
    // = right bank, so the coordinated yaw is negative — nose right).
    // Capped at the sustained-turn envelope (drag = thrust at this
    // speed/altitude), so the jet cannot coordinate a turn it cannot
    // sustain.
    const bank = new THREE.Euler().setFromQuaternion(quat, 'YXZ').z;
    let coordYaw = 0;
    if (v > 5) {
      // Pass 11: cap the CL at this aircraft's peak — the old
      // default 1.6 overstated the envelope for the F-15E (1.10)
      // and the F-22 (1.40), letting the coordinated yaw (and the
      // AI bank command) demand turns the wing cannot hold.
      const omegaMax = sustainedTurnRate(
        v,
        rho,
        thrust * (1 - engineDmgAvg * ENGINE_DMG_THRUST),
        S_lift,
        inducedK,
        m,
        curve.peak,
      );
      coordYaw = clamp((-G * Math.tan(bank)) / v, -omegaMax, omegaMax);
    }
    // Weathercock (BET §1): Cn_β drives the nose toward β = 0.
    // SIGN: β > 0 means the velocity is to the LEFT of the nose
    // (vBody.x > 0, body +X = the LEFT wing), so the tail pushes the
    // nose LEFT (positive yawRate) to align — the term is
    // +WEATHERCOCK·β. Pass 11: the old comment said right/right.
    // A − sign here is positive feedback: it yaws the nose AWAY from
    // the velocity, the sideslip grows, the drag spikes, and the jet
    // bleeds speed and departs in a hard turn (the 60 deg coordinated
    // turn read 8.7 deg/s instead of 4.7 deg/s and bled 400 -> 148 kt;
    // the 80 deg max-performance turn and the AI probe's hard turns
    // tumbled to AoA ~180 deg). Fixed from − to +.
    const betaR = betaDeg * D2R;
    const weathercockYaw = WEATHERCOCK * betaR * clamp(v / 60, 0, 1);
    // Adverse yaw (BET §4): while rolling, the nose yaws AWAY from
    // the turn, scaled by the induced-drag share of total drag —
    // unless the pilot holds rudder into the turn. (rollRate > 0 =
    // rolling right; adverse yaw is nose left = positive yawRate.)
    const inducedShare =
      cdEff > 0.001
        ? (inducedK * cl * cl) / cdEff
        : 0;
    const rudderIntoTurn =
      Math.sign(yawIn) === -Math.sign(rollRate) && yawIn !== 0
        ? Math.abs(yawIn)
        : 0;
    const adverseYaw =
      Math.sign(rollRate) *
      ADV_YAW_K *
      Math.abs(rollRate) *
      inducedShare *
      (1 - rudderIntoTurn);
    // Single-engine-out yaw: toward the dead engine (v2 failEngineSide).
    const engineSide =
      fail.engineSide ?? clamp(dmg.engineL - dmg.engineR, -1, 1);
    const yawRate =
      (yawIn * YAW_RATE_DEG * D2R +
        engineSide * ENGINE_OUT_YAW_DEG * D2R) *
        qAuth +
      coordYaw +
      weathercockYaw +
      adverseYaw +
      spinStep.yawRate;
    // Wing damage: asymmetric roll (per-side in this port).
    const asymRoll = (dmg.wingR - dmg.wingL) * ASYM_ROLL * qAuth;

    // Hover auto-level (Issue 2): in a deep hover (vector >= 70 deg,
    // slow) the lift-fan FCS holds the jet level — a gentle PD on bank
    // and pitch, capped so the pilot can always overpower it (full
    // stick in hover is ~33 deg/s against a 17 deg/s max correction).
    // Without this a hovered jet drifts off level and the pilot has no
    // stable reference; with it the hover holds attitude hands-off.
    // Euler YXZ x > 0 = nose down, z > 0 = right bank; the correction
    // is the opposite sign on both axes. Switches off above 25 m/s,
    // so the transition to forward flight is the pilot's.
    if (hoverBlend > 0.5 && v < 25) {
      const pitch = new THREE.Euler().setFromQuaternion(quat, 'YXZ').x;
      rollRate += clamp(-bank * 0.8, -0.3, 0.3) * hoverBlend;
      pitchRate += clamp(-pitch * 0.8, -0.3, 0.3) * hoverBlend;
    }

    // --- Torque = I·rate / TAU (body frame: x pitch, y yaw, z roll) ---
    const inertia = inertias(m, span, length);
    const torque = new THREE.Vector3(
      (inertia.ix * pitchRate) / TORQUE_TAU,
      (inertia.iy * yawRate) / TORQUE_TAU,
      (inertia.iz * (rollRate + asymRoll + spinStep.rollRate)) / TORQUE_TAU,
    );

    return {
      lift: liftVec,
      drag: dragVec,
      thrust: thrustVec,
      torque,
      aoaDeg,
      cl,
      cd: cdEff,
      stalled: this.stalled,
      spin: this.spinModel.spin,
      spinArrow: this.spinModel.arrow,
      spinDir: this.spinModel.spinDir,
      betaDeg,
      gLoad,
    };
  }
}
