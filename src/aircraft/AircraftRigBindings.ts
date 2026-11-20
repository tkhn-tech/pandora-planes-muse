// AircraftRigBindings — control-surface / rotor / gear animation by
// naming convention. Ported from the v2 HTML template
// (pandora-template.html ~790–810 bindRig, ~1097–1122 updateRig).
//
// These Sketchfab models mostly lack those names, so binding is
// BEST-EFFORT: whatever exists is bound and animated, the rest is
// recorded as missing and reported to the ?debug panel via
// getDebugStatus(). No console.warn here — it would flood the warnings
// panel on every attach (v2 comment, kept).

import * as THREE from 'three';
import {
  controlPitch,
  controlRoll,
  controlYaw,
  type FlightControls,
} from '../physics/IAerodynamicsSolver';
import type { AircraftSpec } from './AircraftSpec';

const D2R = Math.PI / 180;

/** Physics gear posts (FlightPhysicsController: GEAR_HEIGHT = 2.2 m):
 *  the visual gear's wheel feet sit at the same height, so the wheels
 *  touch the ground exactly when the physics gear does. */
const GEAR_FOOT_Y = -2.2;
/** Gear transit time, s — a visible stow/deploy, not an instant pop. */
const GEAR_TRANSIT_S = 1.2;
/** How far the gear slides up into the belly when stowed, m. */
const GEAR_RETRACT_M = 1.6;
/** Main-gear track half-width, m (physics posts are x = ±1.6). */
const GEAR_MAIN_X = 1.6;

/** All bindable slots (v2's 21 patterns + propeller). */
export type RigSlot =
  | 'aileron_L'
  | 'aileron_R'
  | 'elevator_L'
  | 'elevator_R'
  | 'rudder'
  | 'flap_L'
  | 'flap_R'
  | 'airbrake'
  | 'tailhook'
  | 'gear_front'
  | 'gear_L'
  | 'gear_R'
  | 'propeller'
  | 'main_rotor'
  | 'tail_rotor'
  | 'nozzle_L'
  | 'nozzle_R'
  | 'wing_L'
  | 'wing_R'
  | 'tail_section'
  | 'engine_pod_L'
  | 'engine_pod_R';

export const ALL_RIG_SLOTS: RigSlot[] = [
  'aileron_L',
  'aileron_R',
  'elevator_L',
  'elevator_R',
  'rudder',
  'flap_L',
  'flap_R',
  'airbrake',
  'tailhook',
  'gear_front',
  'gear_L',
  'gear_R',
  'propeller',
  'main_rotor',
  'tail_rotor',
  'nozzle_L',
  'nozzle_R',
  'wing_L',
  'wing_R',
  'tail_section',
  'engine_pod_L',
  'engine_pod_R',
];

/** v2 RIG_PATTERNS verbatim, plus propeller. First match wins per slot. */
const RIG_PATTERNS: Array<[RigSlot, RegExp]> = [
  ['aileron_L', /aileron.*(l|left)/i],
  ['aileron_R', /aileron.*(r|right)/i],
  ['elevator_L', /elevator.*(l|left)/i],
  ['elevator_R', /elevator.*(r|right)/i],
  ['rudder', /rudder/i],
  ['flap_L', /flap.*(l|left)/i],
  ['flap_R', /flap.*(r|right)/i],
  ['airbrake', /airbrake|speed.?brake/i],
  ['tailhook', /tailhook|hook/i],
  ['gear_front', /gear.*(front|nose)/i],
  ['gear_L', /gear.*(l|left)/i],
  ['gear_R', /gear.*(r|right)/i],
  ['propeller', /propeller|prop/i],
  ['main_rotor', /main.?rotor/i],
  ['tail_rotor', /tail.?rotor/i],
  ['nozzle_L', /nozzle.*(l|left)/i],
  ['nozzle_R', /nozzle.*(r|right)/i],
  ['wing_L', /wing.*(l|left)/i],
  ['wing_R', /wing.*(r|right)/i],
  ['tail_section', /tail(?!.*rotor)/i],
  ['engine_pod_L', /engine.?pod.*(l|left)/i],
  ['engine_pod_R', /engine.?pod.*(r|right)/i],
];

/** Live state the rig animation reads (v2 updateRig inputs). */
export interface RigState {
  aoaDeg: number;
  gear: boolean;
  hook: boolean;
  vectorDeg: number;
  /** Rotor angular speed, rad/s (v2: 4 + throttle·24 — passed through
   *  as-is by the controller, NOT rpm despite the field name). */
  rotorRpm: number;
}

/** Lowest airframe surface above a gear station (x, z in model
 *  spec-metre units), in the same units. Raycasts straight up in
 *  world space — the model's world transform is honoured, so this
 *  stays correct even if the aircraft was already positioned — and
 *  converts the hit back into model units. Call BEFORE any
 *  procedural leg is attached: at that point every mesh under the
 *  group is airframe. Returns NaN when the ray hits nothing (a hole
 *  in the mesh); the caller falls back to bellyY + 0.25. */
function surfaceAbove(group: THREE.Group, x: number, z: number): number {
  group.updateWorldMatrix(true, true);
  const meshes: THREE.Mesh[] = [];
  group.traverse((o) => {
    if ((o as THREE.Mesh).isMesh) meshes.push(o as THREE.Mesh);
  });
  if (meshes.length === 0) return NaN;
  const s = group.scale.x !== 0 ? group.scale.x : 1;
  const origin = group.localToWorld(new THREE.Vector3(x / s, -10 / s, z / s));
  const dir = new THREE.Vector3(0, 1, 0).transformDirection(group.matrixWorld);
  const ray = new THREE.Raycaster(origin, dir, 0, 40);
  const hits = ray.intersectObjects(meshes, false);
  if (hits.length === 0) return NaN;
  const local = group.worldToLocal(hits[0]!.point.clone());
  return local.y * s;
}

/** One procedural gear leg: a strut from just inside the belly down to
 *  a wheel whose foot sits at GEAR_FOOT_Y (-2.2 m, the physics gear
 *  height). The leg is a Group at (x, 0, z) in the model frame; its
 *  children are positioned in model-frame Y so the rig can slide the
 *  whole leg up into the belly when the gear stows. */
function buildGearLeg(
  x: number,
  z: number,
  wheelR: number,
  strutTop: number,
): THREE.Group {
  const leg = new THREE.Group();
  leg.position.set(x, 0, z);
  const hubY = GEAR_FOOT_Y + wheelR;
  const strutLen = Math.max(strutTop - hubY, 0.2);
  const strutMat = new THREE.MeshStandardMaterial({
    color: 0x9a9a9a,
    metalness: 0.6,
    roughness: 0.4,
  });
  const strut = new THREE.Mesh(
    new THREE.CylinderGeometry(0.07, 0.09, strutLen, 10),
    strutMat,
  );
  strut.position.y = hubY + strutLen / 2;
  leg.add(strut);
  const wheelMat = new THREE.MeshStandardMaterial({
    color: 0x1a1a1a,
    metalness: 0.1,
    roughness: 0.9,
  });
  const wheel = new THREE.Mesh(
    new THREE.CylinderGeometry(wheelR, wheelR, 0.18, 18),
    wheelMat,
  );
  // Cylinder axis is Y; rotate so the axle is along X (rolls along Z).
  wheel.rotation.z = Math.PI / 2;
  wheel.position.y = hubY;
  leg.add(wheel);
  const hub = new THREE.Mesh(
    new THREE.CylinderGeometry(0.09, 0.09, 0.2, 10),
    strutMat,
  );
  hub.rotation.z = Math.PI / 2;
  hub.position.y = hubY;
  leg.add(hub);
  return leg;
}

export class AircraftRigBindings {
  readonly spec: AircraftSpec;
  /** Bound nodes by slot. */
  readonly bound: Partial<Record<RigSlot, THREE.Object3D>> = {};
  /** Slots with no matching node in this model. */
  readonly missing: RigSlot[] = [];
  /** Gear deploy factor 0..1 (1 = fully down, 0 = fully stowed) —
   *  animated toward the gear state over GEAR_TRANSIT_S so the gear
   *  visibly stows/deploys instead of popping. Read by probes. */
  gearDeploy = 1;
  /** Model belly height (bbox min.y) in the model frame, measured
   *  before the procedural gear is attached — normalizeAircraft aligns
   *  it to the physics belly (-0.8 m). Read by probes. */
  bellyY = 0;
  /** Base position.y of each bound gear node (retract slides it up
   *  from here by GEAR_RETRACT_M). */
  private gearBaseY = new Map<THREE.Object3D, number>();
  /** GEAR_RETRACT_M in the gear nodes' local units — the model group
   *  may carry a normalize scale, so the world-space 1.6 m retract
   *  is divided by it (player jets: scale ~1, no-op). */
  private gearRetractLocal = GEAR_RETRACT_M;

  constructor(
    readonly group: THREE.Group,
    spec: AircraftSpec,
  ) {
    this.spec = spec;
    for (const [slot, re] of RIG_PATTERNS) {
      let node: THREE.Object3D | null = null;
      group.traverse((o) => {
        if (!node && o.name && re.test(o.name)) node = o;
      });
      if (node) this.bound[slot] = node;
      else this.missing.push(slot);
    }
    // Procedural landing gear (pass 10): NO player GLB has gear
    // meshes — verified 2026-11-20 by loading the raw GLBs with no
    // rig involved (fa18: 4 named meshes, f22: 3, f35a: 2, f15e: 3 —
    // Weapon_pack/Canopy/ground/helmet002 etc., none below the belly
    // at -0.8 m and none unnamed; the F-22/F-15E "landingOff" meshes
    // are flat belly gear-door fairings, the gear-UP configuration,
    // and stay visible as the belly panel). The 9 "unnamed gear
    // meshes" in the pass-11 brief were the pass-10 procedural legs'
    // own children (strut 40 tris / wheel 72 tris / hub 40 tris),
    // measured by a diagnostic that ran AFTER the rig built them —
    // there is no included gear to bind, so the gear stays
    // procedural: strut + wheel x3 at the physics gear-post stations.
    // A GLB that did have named gear meshes keeps them instead (they
    // are bound above).
    //
    // Pass 11: the strut tops used to stop at bellyY + 0.25 (-0.55 m),
    // but the airframe surface at the posts sits HIGHER than the
    // belly plane — F-15E mains -0.08 m / nose +0.17 m, F-35A mains
    // +0.28/-0.10 m / nose +0.16 m, F-22 mains -0.26 m / nose
    // -0.51 m, F-18 mains +1.62 m (under the wing) / nose +0.84 m —
    // so the struts visibly floated below the jet ("gears
    // disconnected from the jet"). Each strut top is now raycast to
    // the lowest airframe surface above its own station (+0.10 m
    // overlap into the fuselage), per jet per station, so the gear
    // connects to the airframe it belongs to.
    const gearSlots: Array<[RigSlot, number, number, number]> = [
      // slot, x, z, wheel radius
      ['gear_front', 0, 0.35 * spec.length, 0.3],
      ['gear_L', -GEAR_MAIN_X, -0.15 * spec.length, 0.35],
      ['gear_R', GEAR_MAIN_X, -0.15 * spec.length, 0.35],
    ];
    // Belly (lowest fuselage point), measured before the gear is
    // attached. normalizeAircraft aligns it to the physics belly
    // (-0.8 m).
    this.bellyY = new THREE.Box3().setFromObject(group).min.y;
    // The model group may carry a normalize scale (uniform): a leg
    // built in spec-metre units and parented under it would be
    // scaled and displaced (MiG-29: x4.6 — its legs used to float
    // 19 m ahead of the jet). Counter-scale each leg and divide its
    // position by the model scale so it lands at the spec-metre
    // station unscaled. Player jets are ~1.0, so this is a no-op
    // for them.
    const modelScale = group.scale.x !== 0 ? group.scale.x : 1;
    this.gearRetractLocal = GEAR_RETRACT_M / modelScale;
    // Raycast every station BEFORE adding any leg — a leg added
    // earlier would be visible to the next station's ray.
    const strutTops = new Map<RigSlot, number>();
    for (const [slot, x, z] of gearSlots) {
      if (this.bound[slot]) continue;
      const surfaceY = surfaceAbove(group, x, z);
      strutTops.set(
        slot,
        Number.isNaN(surfaceY)
          ? this.bellyY + 0.25
          : Math.max(
              this.bellyY + 0.25,
              Math.min(surfaceY + 0.1, this.bellyY + 3.0),
            ),
      );
    }
    for (const [slot, x, z, wheelR] of gearSlots) {
      if (this.bound[slot]) continue;
      const gear = buildGearLeg(x, z, wheelR, strutTops.get(slot) ?? this.bellyY + 0.25);
      gear.name = 'gear_leg_' + slot;
      gear.scale.setScalar(1 / modelScale);
      gear.position.set(x / modelScale, 0, z / modelScale);
      group.add(gear);
      this.bound[slot] = gear;
      this.gearBaseY.set(gear, gear.position.y);
      const mi = this.missing.indexOf(slot);
      if (mi >= 0) this.missing.splice(mi, 1);
    }
    for (const slot of ['gear_front', 'gear_L', 'gear_R'] as const) {
      const node = this.bound[slot];
      if (node && !this.gearBaseY.has(node)) this.gearBaseY.set(node, node.position.y);
    }
  }

  /** 'bound: aileron_L, ... · missing: ...' — shown in the ?debug panel. */
  getDebugStatus(): string {
    const found = ALL_RIG_SLOTS.filter((s) => this.bound[s]);
    return (
      'bound: ' +
      (found.join(', ') || 'none') +
      ' · missing: ' +
      this.missing.join(', ')
    );
  }

  /** Animate whatever bindRig found (v2 updateRig verbatim; usually a
   *  no-op on Sketchfab meshes). Rotations are set ABSOLUTE each step,
   *  matching the template — no base-pose restore. */
  update(dt: number, controls: FlightControls, state: RigState): void {
    const r = this.bound;
    const rollIn = controlRoll(controls);
    const pitchIn = controlPitch(controls);
    const yawIn = controlYaw(controls);
    void state.aoaDeg; // accepted for future trim/buffet work; v2 ignores it.
    if (r.aileron_L) r.aileron_L.rotation.z = -rollIn * 0.5;
    if (r.aileron_R) r.aileron_R.rotation.z = rollIn * 0.5;
    if (r.elevator_L) r.elevator_L.rotation.x = pitchIn * 0.4;
    if (r.elevator_R) r.elevator_R.rotation.x = pitchIn * 0.4;
    if (r.rudder) r.rudder.rotation.y = yawIn * 0.5;
    if (r.flap_L) r.flap_L.rotation.x = -((controls.flaps ?? 0) / 30) * 0.5;
    if (r.flap_R) r.flap_R.rotation.x = -((controls.flaps ?? 0) / 30) * 0.5;
    if (r.airbrake) r.airbrake.rotation.x = -(controls.airbrake ?? 0) * 0.9;
    if (r.tailhook) r.tailhook.visible = state.hook;
    // Gear: animate gearDeploy toward the gear state over
    // GEAR_TRANSIT_S, then slide the bound gear legs up into the
    // belly as they stow (and hide them once fully stowed) — a
    // visible transition, not an instant pop. Applies to the
    // procedural legs and to any GLB gear meshes that were bound.
    const gearTarget = state.gear ? 1 : 0;
    const step = dt / GEAR_TRANSIT_S;
    this.gearDeploy += Math.max(-step, Math.min(step, gearTarget - this.gearDeploy));
    for (const slot of ['gear_front', 'gear_L', 'gear_R'] as const) {
      const node = r[slot];
      if (!node) continue;
      node.visible = this.gearDeploy > 0.02;
      const baseY = this.gearBaseY.get(node) ?? 0;
      node.position.y = baseY + (1 - this.gearDeploy) * this.gearRetractLocal;
    }
    if (r.nozzle_L) r.nozzle_L.rotation.x = -state.vectorDeg * D2R;
    if (r.nozzle_R) r.nozzle_R.rotation.x = -state.vectorDeg * D2R;
    if (r.propeller) r.propeller.rotation.z += dt * state.rotorRpm;
    if (r.main_rotor) r.main_rotor.rotation.y += dt * state.rotorRpm;
    if (r.tail_rotor) r.tail_rotor.rotation.x += dt * state.rotorRpm;
  }

  /** Detach a bound segment (fatal damage, v2 detachWing): removes the
   *  node from its parent and returns it so the caller can hand it to
   *  CrashVFXManager.spawnDebris() as a tumbling Rapier body. Returns
   *  null when the slot was never bound (caller falls back to a box). */
  detachSegment(name: RigSlot): THREE.Object3D | null {
    const node = this.bound[name];
    if (!node) return null;
    if (node.parent) node.parent.remove(node);
    delete this.bound[name];
    if (!this.missing.includes(name)) this.missing.push(name);
    return node;
  }
}
