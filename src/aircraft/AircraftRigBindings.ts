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

export class AircraftRigBindings {
  readonly spec: AircraftSpec;
  /** Bound nodes by slot. */
  readonly bound: Partial<Record<RigSlot, THREE.Object3D>> = {};
  /** Slots with no matching node in this model. */
  readonly missing: RigSlot[] = [];

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
    if (r.gear_front) r.gear_front.visible = state.gear;
    if (r.gear_L) r.gear_L.visible = state.gear;
    if (r.gear_R) r.gear_R.visible = state.gear;
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
