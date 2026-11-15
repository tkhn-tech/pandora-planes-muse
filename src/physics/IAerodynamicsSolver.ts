// IAerodynamicsSolver — the contract between the flight model and the
// physics controller. Ported from the v2 HTML template
// (pandora-template.html ~958–1075, Aircraft.step): the same CL/CD curve,
// stall + spin, ISA density, ground effect, VTOL vector and rotary branch,
// with the v1.1 body-frame AoA fix (atan2(-vBody.y, vBody.z) from the AIR
// velocity — never the old asin flight-path formula).
//
// OWNERSHIP LINE (Rapier vs solver): the solver owns FORCES. It is a pure
// function of the aero state (plus stall/spin hysteresis held on the
// solver instance, one instance per aircraft). It never moves anything.
// The controller (FlightPhysicsController) owns INTEGRATION: it turns the
// solver's forces into a velocity and hands that velocity to a Rapier
// dynamic rigid body, which owns POSITION and COLLISIONS. Rapier gravity
// is switched off on aircraft bodies (gravityScale 0) because gravity is
// already inside the solver's force sum — otherwise it would be counted
// twice.
//
// FlightControls — RECONCILIATION NOTE. Two shapes for this type already
// exist in the project and both must keep compiling:
//   * The input workers (PCFlightInput / MobileFlightInput /
//     IFlightInputProvider, written first) return the MERGED STICK shape:
//     { pitch, roll, yaw, throttle: number | null } where throttle null
//     means "no opinion this frame, keep the last target".
//   * The physics task brief specified the long shape: pitchIn / rollIn /
//     yawIn plus flaps / airbrake / vector / gear / hook / brake / firing.
//   * The AI controller returns { pitchIn, rollIn, yawIn, thrTarget }.
// All fields are therefore optional, and consumers must resolve through
// controlPitch / controlRoll / controlYaw / resolveThrottleTarget.
// The integrator (main.ts) is the place that should one day canonicalise
// this to a single shape; until then, producers may use any naming and
// consumers must use the resolve* helpers, never the raw fields.

import * as THREE from 'three';
import type { AircraftArchetype } from '../aircraft/AircraftSpec';

/** Per-zone structural damage, 0 = healthy .. 1 = destroyed. The v2
 *  template carried two scalars (wingDmg, engineDmg); the port splits
 *  them per side so asymmetric damage can roll/yaw the jet. */
export interface DamageZones {
  wingL: number;
  wingR: number;
  tail: number;
  engineL: number;
  engineR: number;
}

export const ZERO_DAMAGE: DamageZones = {
  wingL: 0,
  wingR: 0,
  tail: 0,
  engineL: 0,
  engineR: 0,
};

/** Pilot / AI control input for one fixed step.
 *
 *  ALL FIELDS ARE OPTIONAL — three producer shapes exist in the project
 *  and all three must keep compiling:
 *    * input workers: { pitch, roll, yaw, throttle: number | null }
 *      (throttle null = "no opinion, keep the last target");
 *    * the task-brief long shape: pitchIn / rollIn / yawIn plus flaps /
 *      airbrake / vector / gear / hook / brake / firing;
 *    * the AI controller: { pitchIn, rollIn, yawIn, thrTarget }
 *      (throttle target kept on the AI, returned each update).
 *  Consumers MUST resolve through controlPitch / controlRoll /
 *  controlYaw / resolveThrottleTarget below, never the raw fields.
 *  Missing stick fields default to 0 (no input); missing throttle
 *  fields mean "keep the current target". */
export interface FlightControls {
  /** Stick pitch, -1..1. NEGATIVE pulls up (v2 template convention:
   *  pitchIn < -0.25 is the take-off rotation trigger, and
   *  pitchRate = pitchIn * 55 deg/s). */
  pitch?: number;
  /** Stick roll, -1..1. */
  roll?: number;
  /** Stick yaw / rudder, -1..1. */
  yaw?: number;
  /** Throttle target 0..1, or null = "no opinion, keep the last target"
   *  (the input workers' convention). */
  throttle?: number | null;
  /** Alias for pitch. */
  pitchIn?: number;
  /** Alias for roll. */
  rollIn?: number;
  /** Alias for yaw. */
  yawIn?: number;
  /** Throttle target 0..1 (the AI controller's alias for `throttle`). */
  thrTarget?: number;
  /** Flaps 0..30 deg (0 = up). */
  flaps?: number;
  /** Airbrake 0..1. */
  airbrake?: number;
  /** Thrust-vector angle 0..90 deg (VTOL jets only; clamped to the spec). */
  vector?: number;
  /** Gear down. */
  gear?: boolean;
  /** Tailhook down. */
  hook?: boolean;
  /** Wheel brakes held. */
  brake?: boolean;
  /** Gun trigger held. */
  firing?: boolean;
}

/** Resolve the effective stick pitch from either naming (default 0). */
export function controlPitch(c: FlightControls): number {
  return c.pitchIn ?? c.pitch ?? 0;
}

/** Resolve the effective stick roll from either naming (default 0). */
export function controlRoll(c: FlightControls): number {
  return c.rollIn ?? c.roll ?? 0;
}

/** Resolve the effective stick yaw from either naming (default 0). */
export function controlYaw(c: FlightControls): number {
  return c.yawIn ?? c.yaw ?? 0;
}

/** Resolve the effective throttle target: `throttle` wins, then the
 *  AI's `thrTarget`, then null ("keep the current target"). */
export function resolveThrottleTarget(c: FlightControls): number | null {
  return c.throttle ?? c.thrTarget ?? null;
}

/** Failure flags the G7 failure system can set on an aircraft. The
 *  solver only reads them; it never sets them. */
export interface AeroFailures {
  /** Whole-engine failure (both engines at half thrust in v2). */
  engineOut?: boolean;
  /** Single-engine-out side: -1 = left engine dead, +1 = right dead,
   *  0 = none. Yaws toward the dead engine (v2 template). */
  engineSide?: -1 | 0 | 1;
  /** Hydraulic failure: pitch authority x0.4 (v2). */
  hydroDegraded?: boolean;
}

/** Control-authority multipliers, from the spec (turn / roll / pitch). */
export interface ControlAuthority {
  /** Turn multiplier (v2 spec.turn). */
  turn: number;
  /** Max roll rate, deg/s (v2 spec.roll). */
  rollDeg: number;
  /** Pitch multiplier (v2 spec.pitch). */
  pitch: number;
}

/** Everything the solver needs for one fixed step. */
export interface AeroState {
  /** Airspeed vector (ground velocity MINUS wind), in WORLD frame.
   *  The solver converts it to the body frame itself. */
  velocityAir: THREE.Vector3;
  /** Aircraft attitude. */
  quat: THREE.Quaternion;
  /** Flying mass, kg (spec.mass — the v2 operating mass). */
  mass: number;
  /** Wing area, m^2. */
  wingArea: number;
  /** Wingspan, m (also the ground-effect height threshold). */
  span: number;
  /** Mean chord, m. Defaults to wingArea / span. */
  chord?: number;
  /** Fuselage length, m (roll/pitch inertia model). Defaults to span*1.4. */
  length?: number;
  /** SPOOLED throttle 0..1 — the controller owns the 0.55/s spool
   *  (v2 template), the solver takes the result as-is. */
  throttle: number;
  /** Max thrust, N (spec.thrustMax). */
  thrustMax: number;
  /** Thrust-vector angle, deg (VTOL jets only). */
  vectorDeg: number;
  /** Flaps, deg (0..30). */
  flapsDeg: number;
  /** Airbrake 0..1. */
  airbrake: number;
  /** Gear down. */
  gearDown: boolean;
  /** Per-zone damage, 0..1. */
  damage: DamageZones;
  /** Altitude above sea level, m (ISA density). */
  altitude: number;
  /** Air density, kg/m^3. Pass 1.225 at sea level; <= 0 recomputes ISA. */
  density: number;
  /** Aircraft archetype — REQUIRED: the solver branches on rotary
   *  (rotor-disc lift, collective, autorotation) vs fixed/vtol. */
  archetype: AircraftArchetype;
  /** Pilot / AI input for this step. Needed for the control torques
   *  (rates). Omit for a pure aero query (no control torque). */
  controls?: FlightControls;
  /** Height above the ground / deck, m. Ground effect applies when this
   *  is below one wingspan and airspeed > 30 m/s. Omit to skip it. */
  heightAboveGround?: number;
  /** Control authority (spec.turn / roll / pitch). Defaults to 1 / 150 / 1. */
  controlAuthority?: ControlAuthority;
  /** Active failure flags (G7). */
  failures?: AeroFailures;
  /** Fixed-step length, s. Default 1/60. Used only for the stall
   *  nose-drop rate and spin build/decay (v2 applies them per step). */
  dt?: number;
}

/** Solver output for one fixed step. lift / drag / thrust are WORLD-frame
 *  force vectors (N); torque is a BODY-frame control torque (N·m). */
export interface AeroResult {
  /** Lift force, along the body up axis (N). For a rotary archetype this
   *  is rotor-disc lift, not wing lift. */
  lift: THREE.Vector3;
  /** Drag force, opposing the airflow (N). Also carries the stall-spin
   *  lateral drift force (v2 adds it straight to velocity; here it is a
   *  force so the total stays lift + drag + thrust). */
  drag: THREE.Vector3;
  /** Thrust force, along the (possibly vectored) thrust axis (N). At
   *  vector 90 deg on a VTOL jet this is vertical lift, not forward
   *  thrust — exactly as in v2. */
  thrust: THREE.Vector3;
  /** Control torque in the BODY frame (N·m), built from the v2
   *  rate-authority model: torque = I * rate / TAU with TAU = 0.25 s, so
   *  the controller recovers the template's pitch/roll/yaw rates via
   *  torqueToRates(). The v2 template has no aerodynamic moment model —
   *  only rate authority — so this torque is the rate command expressed
   *  as a torque, not a free moment. */
  torque: THREE.Vector3;
  /** Angle of attack, deg (body-frame, v1.1 fix). */
  aoaDeg: number;
  /** Lift coefficient (after ground effect, clamped -0.8..1.5). */
  cl: number;
  /** Drag coefficient. */
  cd: number;
  /** True while stalled (with the v2 3-deg hysteresis). */
  stalled: boolean;
  /** Spin amount 0..1 (builds while stalled and yawed, decays otherwise). */
  spin: number;
  /** G-load from wing/rotor lift (lift / weight), floored at 0.2 (v2). */
  gLoad: number;
}

/** The aerodynamic solver contract. One instance per aircraft: stall
 *  hysteresis and spin state live on the instance. */
export interface IAerodynamicsSolver {
  solve(state: AeroState): AeroResult;
}
