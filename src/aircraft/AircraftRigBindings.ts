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
import type { AircraftSpec, IncludedGearStation } from './AircraftSpec';

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
  // Gear patterns are END-ANCHORED (pass 12): the loose v2
  // `gear.*(r|right)` matched "gear_front" — the 'r' in "front" — so
  // gear_R bound gear_front and the real gear_R mesh was never
  // animated: it stayed down in flight at y=0 while the probe read
  // rig.bound (gear_front, stowed) and passed. gear_L had the same
  // hazard via "gear_leg_..." (the 'l' in "leg"). The numeric suffix
  // allows Blender duplicates (gear_L.001); the other L/R patterns
  // (aileron, flap, nozzle...) cannot cross-match this way — their
  // disambiguator letter never occurs in the opposite slot's name.
  ['gear_front', /gear[_-]*(front|nose)([._-]\d+)?$/i],
  ['gear_L', /gear[_-]*(l|left)([._-]\d+)?$/i],
  ['gear_R', /gear[_-]*(r|right)([._-]\d+)?$/i],
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

/**
 * Separate a GLB's own (baked-in) landing gear from its airframe mesh
 * into named gear groups the rig can bind and animate.
 *
 * Why this exists (2026-11-20): Nicholas's F/A-18F GLB carries its
 * own gear — nose twin wheels + 2 main wheels — baked into the
 * "ground" airframe mesh (no separate gear nodes; the gear is short,
 * its wheel bottoms 1.36 m above the physics gear feet). Pass 11
 * claimed "no included gear exists", measured on the NORMALIZED
 * model where normalizeAircraft had already aligned the wheel
 * bottoms to -0.8 m, so of course nothing was below -0.8. The gear
 * is real (see the purple circles in Nicholas's screenshot) and must
 * be used instead of the procedural legs: the F-18 shows ONLY its
 * own 3 gear legs. The other player GLBs were re-verified the same
 * way (MeshoptDecoder, raw GLB, gear stations): F-22/F-15E carry
 * only a flat "landingOff" belly fairing, the F-35A's lowest point
 * is its belly — no included gear, procedural legs stay.
 *
 * How: the airframe mesh (largest by bbox volume) has its triangles
 * split by centroid in the aircraft frame (after normalize): a
 * triangle whose centroid is inside a station box and below its
 * capY belongs to that gear. The gear triangles become new meshes
 * named gear_front / gear_L / gear_R (matching RIG_PATTERNS, so the
 * rig binds them and animates them with the existing gearDeploy
 * factor); the remaining triangles stay on the airframe mesh. capY
 * is the fuselage / wing underside at the station minus a 0.05 m
 * safety margin, so no airframe triangle is ever cut into the gear —
 * the gear strut is cut 0.05 m short of the airframe, an invisible
 * gap in the fuselage's shadow.
 *
 * The gear geometry is baked into the group (aircraft) frame, so the
 * rig's position.y slide (gearRetractLocal) moves it in aircraft
 * metres. The airframe keeps its local-frame geometry and transform.
 * Only position / normal / uv are carried over — the F-18's airframe
 * has no other attributes. Returns per-slot triangle counts (0 for a
 * station that matched nothing — a wrong station box, caught by
 * probe-gear).
 */
export function separateIncludedGear(
  group: THREE.Group,
  stations: IncludedGearStation[],
): {
  separated: boolean;
  gearTris: Partial<Record<'gear_front' | 'gear_L' | 'gear_R', number>>;
  airframeTris: number;
} {
  group.updateWorldMatrix(true, true);
  // Airframe = the largest mesh by bbox volume.
  let airframe: THREE.Mesh | null = null;
  let bestV = -1;
  group.traverse((o) => {
    const m = o as THREE.Mesh;
    if (!m.isMesh) return;
    const bb = new THREE.Box3().setFromObject(m);
    const sz = new THREE.Vector3();
    bb.getSize(sz);
    const v = sz.x * sz.y * sz.z;
    if (v > bestV) {
      bestV = v;
      airframe = m;
    }
  });
  if (!airframe) return { separated: false, gearTris: {}, airframeTris: 0 };
  const geo = (airframe as THREE.Mesh).geometry as THREE.BufferGeometry;
  const posAttr = geo.attributes.position as THREE.BufferAttribute | undefined;
  if (!posAttr) return { separated: false, gearTris: {}, airframeTris: 0 };
  // Relative matrix: airframe local -> group (aircraft) frame.
  const rel = new THREE.Matrix4()
    .copy(group.matrixWorld)
    .invert()
    .multiply((airframe as THREE.Mesh).matrixWorld);
  const normalMat = new THREE.Matrix3().getNormalMatrix(rel);
  const index = geo.index;
  const triCount = index ? index.count / 3 : posAttr.count / 3;
  const normAttr = geo.attributes.normal as THREE.BufferAttribute | undefined;
  const uvAttr = geo.attributes.uv as THREE.BufferAttribute | undefined;
  interface Bucket {
    pos: number[];
    norm: number[];
    uv: number[];
    hasUv: boolean;
  }
  const airBucket: Bucket = { pos: [], norm: [], uv: [], hasUv: false };
  const gearBuckets = new Map<string, Bucket>();
  for (const st of stations) {
    gearBuckets.set(st.slot, { pos: [], norm: [], uv: [], hasUv: false });
  }
  const vLocal = new THREE.Vector3();
  const vGroup = new THREE.Vector3();
  const nLocal = new THREE.Vector3();
  const nGroup = new THREE.Vector3();
  const centroid = new THREE.Vector3();
  const pushVert = (bucket: Bucket, vi: number, toGroup: boolean): void => {
    vLocal.fromBufferAttribute(posAttr, vi);
    if (toGroup) {
      vGroup.copy(vLocal).applyMatrix4(rel);
      bucket.pos.push(vGroup.x, vGroup.y, vGroup.z);
    } else {
      bucket.pos.push(vLocal.x, vLocal.y, vLocal.z);
    }
    if (normAttr) {
      nLocal.fromBufferAttribute(normAttr, vi);
      if (toGroup) {
        nGroup.copy(nLocal).applyMatrix3(normalMat).normalize();
        bucket.norm.push(nGroup.x, nGroup.y, nGroup.z);
      } else {
        bucket.norm.push(nLocal.x, nLocal.y, nLocal.z);
      }
    }
    if (uvAttr) {
      bucket.uv.push(uvAttr.getX(vi), uvAttr.getY(vi));
      bucket.hasUv = true;
    }
  };
  let airTris = 0;
  const gearTris: Partial<Record<'gear_front' | 'gear_L' | 'gear_R', number>> = {};
  for (let t = 0; t < triCount; t++) {
    const vi = [0, 1, 2].map((k) => (index ? index.getX(t * 3 + k) : t * 3 + k));
    // Centroid and max vertex height in the group (aircraft) frame.
    // A triangle is gear only when its centroid is inside a station
    // box AND all three vertices are below the cap — a large
    // airframe triangle straddling the cap plane (its centroid below
    // the cap but a vertex above it) must stay with the airframe,
    // or the wing / fuselage gets a hole where the strut passes
    // through (a probe ray through the gear_R station read the wing
    // interior at +0.22 m instead of the underside at -1.20 m).
    centroid.set(0, 0, 0);
    let maxY = -Infinity;
    for (const v of vi) {
      vLocal.fromBufferAttribute(posAttr, v);
      vGroup.copy(vLocal).applyMatrix4(rel);
      centroid.add(vGroup);
      maxY = Math.max(maxY, vGroup.y);
    }
    centroid.multiplyScalar(1 / 3);
    let slot: IncludedGearStation | null = null;
    for (const st of stations) {
      if (
        centroid.x >= st.x0 &&
        centroid.x <= st.x1 &&
        centroid.z >= st.z0 &&
        centroid.z <= st.z1 &&
        maxY < st.capY
      ) {
        slot = st;
        break;
      }
    }
    if (slot) {
      const bucket = gearBuckets.get(slot.slot)!;
      for (const v of vi) pushVert(bucket, v, true);
      gearTris[slot.slot] = (gearTris[slot.slot] ?? 0) + 1;
    } else {
      for (const v of vi) pushVert(airBucket, v, false);
      airTris++;
    }
  }
  const buildGeo = (bucket: Bucket): THREE.BufferGeometry => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(bucket.pos, 3));
    if (bucket.norm.length) {
      g.setAttribute('normal', new THREE.Float32BufferAttribute(bucket.norm, 3));
    }
    if (bucket.hasUv && bucket.uv.length) {
      g.setAttribute('uv', new THREE.Float32BufferAttribute(bucket.uv, 2));
    }
    return g;
  };
  (airframe as THREE.Mesh).geometry = buildGeo(airBucket);
  for (const st of stations) {
    const bucket = gearBuckets.get(st.slot)!;
    if (!bucket.pos.length) continue;
    const mesh = new THREE.Mesh(buildGeo(bucket), (airframe as THREE.Mesh).material);
    mesh.name = st.slot;
    mesh.castShadow = false;
    group.add(mesh);
  }
  return { separated: true, gearTris, airframeTris: airTris };
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
    // Procedural landing gear (pass 10): for jets whose GLB has no
    // gear meshes, the gear is procedural (strut + wheel x3, built
    // here at the physics gear-post stations). The F-18 is the
    // exception — its GLB carries its own gear, separated into
    // gear_front / gear_L / gear_R by separateIncludedGear() in
    // attachModel, which the pattern bind above already bound, so
    // the loop below skips those slots (`if (this.bound[slot])
    // continue`) and the F-18 shows ONLY its own gear. The F-22 /
    // F-15E "landingOff" meshes are flat belly gear-door fairings,
    // the gear-UP configuration, and stay visible as the belly
    // panel; the F-35A's lowest point is its belly. A GLB that did
    // have named gear meshes keeps them instead (they are bound
    // above).
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
    // Procedural legs are built at the physics gear-post stations
    // (spec.gearPosts when the model sets them — pass 12, item 14 —
    // else the v2 formula), so the visual legs always sit on the
    // physics posts. The F-18 never reaches this loop (its GLB gear
    // is bound above).
    const postNoseZ = spec.gearPosts?.noseZ ?? 0.35 * spec.length;
    const postMainZ = spec.gearPosts?.mainZ ?? -0.15 * spec.length;
    const postMainX = spec.gearPosts?.mainX ?? GEAR_MAIN_X;
    const gearSlots: Array<[RigSlot, number, number, number]> = [
      // slot, x, z, wheel radius
      ['gear_front', 0, postNoseZ, 0.3],
      ['gear_L', -postMainX, postMainZ, 0.35],
      ['gear_R', postMainX, postMainZ, 0.35],
    ];
    // Belly (lowest fuselage point). normalizeAircraft aligns it to
    // the physics belly (-0.8 m) — or to spec.bellyAlign.targetY for
    // the F-18, whose airframe carries appendages below the fuselage
    // belly (a centreline ventral feature at -1.63 m, stores at
    // -1.45 m), so the bbox min is NOT the belly there. Excludes any
    // separated GLB gear groups (their wheels reach the physics
    // feet, -2.2 m) and any procedural legs, so the belly stays the
    // airframe's lowest point on the other jets.
    if (spec.bellyAlign) {
      this.bellyY = spec.bellyAlign.targetY;
    } else {
      group.updateWorldMatrix(true, true);
      const bb = new THREE.Box3();
      let any = false;
      group.traverse((o) => {
        const m = o as THREE.Mesh;
        if (!m.isMesh) return;
        let p: THREE.Object3D | null = o;
        while (p) {
          if (
            p.name === 'gear_front' ||
            p.name === 'gear_L' ||
            p.name === 'gear_R' ||
            p.name.startsWith('gear_leg_')
          )
            return;
          p = p.parent;
        }
        bb.expandByObject(m);
        any = true;
      });
      // Pass 13 (worker C): expandByObject measures in WORLD space,
      // but bellyY is consumed in MODEL units (strut tops, the
      // bellyY + 3.0 cap). The rig is built inside attachModel,
      // which startRound calls on a REUSED player whose group still
      // sits at the previous spawn's altitude (e.g. y 600) — so
      // bellyY came out ~599 and every procedural leg was built with
      // strutTop ~599.5: a ~600 m spike from the nose gear up
      // through the jet (visible whenever the gear is down, and it
      // crossed the F-35A's cockpit sightline at 0.53 m). Convert
      // the world bbox back into the model's own frame before
      // reading it.
      const inv = group.matrixWorld.clone().invert();
      bb.applyMatrix4(inv);
      this.bellyY = any
        ? bb.min.y
        : new THREE.Box3().setFromObject(group).applyMatrix4(inv).min.y;
    }
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
