// FlightPhysicsController — Aircraft: one rigid body + one solver.
// Ported from the v2 HTML template (pandora-template.html ~889–1204
// Aircraft.step / groundCheck / damage / crash, ~749–890 model loading,
// ~1204–1330 carrier deck interaction).
//
// OWNERSHIP LINE (solver vs Rapier):
//   * CurveAerodynamicsSolver owns FORCES (lift / drag / thrust / control
//     torque) — a pure function of the aero state.
//   * This controller owns the per-step BOOKKEEPING the template did in
//     Aircraft.step: throttle spool (0.55/s), fuel burn, G-overstress,
//     ground-roll friction, nose-wheel steering, arresting wires, deck
//     velocity transfer, spawn protection, damage zones.
//   * Rapier owns INTEGRATION and COLLISIONS. Each fixed step the
//     controller computes the new velocity from the solver forces
//     (semi-implicit Euler, exactly like the template: vel += acc·dt)
//     and hands it to the body with setLinvel / setAngvel
//     (NOT applyImpulse — the template's rates are kinematic targets,
//     and impulses would fight the body's damping/inertia). Rapier then
//     integrates position and resolves contacts against the terrain
//     heightfield / carrier deck. The body runs with gravityScale 0:
//     gravity is already in the solver's force sum.
//   * Ground contact therefore comes from Rapier: the integrator feeds
//     collision events into notifyContact(), and `grounded` is set when
//     a contact was reported this step AND vertical speed is ~0. The
//     terrainHeightAt callback is used ONLY as a fallback contact
//     detector (and for the terrain-penetration safety net) — the
//     controller NEVER pins pos.y to the ground (no analytic ground
//     pinning); Rapier resolves penetration.
//
// TEMPLATE COMPATIBILITY (main.ts drives the player template-style, so
// the Aircraft also exposes the v2 template's mutable surface):
//   * pitchIn / rollIn / yawIn, thrTarget, flaps, airbrake, vector,
//     brake, gear, hook, gunCd, flareCd are public mutable fields.
//     The 3-argument step(dt, wind, turbAcc) overload reads them.
//   * failEngine / failEngineSide / failFire / failHydro / failGearJam /
//     failFuel are accessor pairs over `failures`.
//   * trapped / lastHitBy / vtolCounted are readable (lastHitBy is a
//     minimal { isPlayer } record — WeaponSystem passes only a boolean,
//     so the shooter object itself is not retained).
//   * resetAt(x, y, z, hdgDeg) is the template-named alias of respawn().
//   The canonical API is still step(dt, controls, wind,
//   terrainHeightAt, turbAcc?) with FlightControls; the template surface
//   exists so the integrator compiles while it migrates to it.
//
// CALL ORDER (integrator / main.ts): world.step() -> for each aircraft:
//   notifyContact() from the collision-event dispatch, then
//   aircraft.step(dt, controls, wind, terrainHeightAt). The controller
//   reads the body AFTER world.step, so pos/vel/quat are fresh.
// RAPIER must be initialised (await RAPIER.init()) before the world
// passed in here is created. Aircraft start DISABLED at the origin;
// respawn(runway) enables and places them.

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import {
  controlPitch,
  controlRoll,
  resolveThrottleTarget,
  type FlightControls,
  ZERO_DAMAGE,
  type DamageZones,
} from './IAerodynamicsSolver';
import {
  CurveAerodynamicsSolver,
  torqueToRates,
} from './CurveAerodynamicsSolver';
import { BladeElementSolver } from './BladeElementSolver';
import {
  NOSE_ROT_Y,
  type AircraftSpec,
  type ModelKey,
} from '../aircraft/AircraftSpec';
import { AircraftRigBindings } from '../aircraft/AircraftRigBindings';
import { DamageableEntity } from '../combat/DamageableEntity';

const D2R = Math.PI / 180;
const G = 9.81;

function clamp(v: number, a: number, b: number): number {
  return Math.max(a, Math.min(b, v));
}

/** v2 constants (template Aircraft.step / groundCheck / damage). */
const SPOOL_RATE = 0.55; // throttle units per second
const GEAR_HEIGHT = 2.2; // gear-down ride height above the surface
const BELLY_HEIGHT = 1.2; // gear-up ride height
const HARD_LANDING_VS = 9; // m/s vertical speed that breaks the jet
const HARD_LANDING_BANK = 0.45; // rad of bank that breaks the jet
const GROUND_FRICTION = 0.3; // m/s^2 rolling friction (research §6:
// real rolling resistance is ~0.02-0.03 g; 1.2 ate ~12% of takeoff accel)
const ROTATION_RATE_MAX = 12 * D2R; // rad/s nose-up while the gear is on
// a surface (BET report §3: rotation needs ~10-15 deg/s, not the 55 deg/s
// max authority — the full rate whips the nose up and tail-strikes)
const BRAKE_DECEL = 6; // m/s^2 with wheel brakes
const STEER_RATE = 0.9; // nose-wheel steering, rad/s at full stick
const TRAP_MAX_SPEED = 72; // m/s — faster snaps the wire
const TRAP_BASE_DECEL = 14; // m/s^2 initial wire decel
const TRAP_RAMP = 30; // m/s^2 extra decel per second of trap
const SPAWN_PROTECTION_S = 5; // v1.1: player spawn protection
const OVER_G_MARGIN = 1.5; // damage starts at maxG + 1.5
const OVER_G_DPS = 8; // hp per second over the limit
const OVER_G_MIN_SPEED = 60; // m/s
const WIRE_LOCAL_Z_MIN = -105; // arresting-wire trigger zone (carrier-local)
const WIRE_LOCAL_Z_MAX = -45;
const WIRE_LOCAL_X = 26;
const FUEL_BURN_BASE = 0.02; // per second, template units (fuel 0..100)
const FUEL_BURN_THR = 0.11; // per second at full throttle
const FUEL_LEAK_EXTRA = 0.25; // per second with a fuel leak
const WING_LOSS_DMG = 0.75; // wing damage that detaches the wing
const WING_DETACH_FATAL = 0.4; // wing damage that detaches on crash
const SMOKE_HP = 60; // smoke below this hp
const FIRE_HP = 35; // fire below this hp
const LOAD_TIMEOUT_MS = 9000; // v2 loadGLB timeout

// ---- Damage model (War Thunder report §3, §7) ----
// Fire: a fuel-tank fire ramps to full intensity in ~12 s (game pace;
// WT's 40-60 s scaled down), an engine fire ramps faster (~4 s) but
// stays localised. Fuel fire 2.0 hp/s at full intensity, engine fire
// 1.2 hp/s. A fuel fire spreads to one adjacent zone per ~8 s; a wing
// fire burning long enough destroys the spar (wing separation).
const FUEL_FIRE_RAMP_S = 12; // seconds to full intensity
const ENGINE_FIRE_RAMP_S = 4;
const FUEL_FIRE_DPS = 2.0; // hp/s at full intensity
const ENGINE_FIRE_DPS = 1.2;
const FIRE_SPREAD_S = 8; // fuel fire spreads to an adjacent zone
// Extinguish: a fuel fire's self-extinguish chance rises with airspeed
// (slipstream) — 2%/s below 150 m/s IAS, 8%/s above 300 m/s. An
// engine fire has a 15%/s chance to go out when that engine's throttle
// is cut to 0 (fuel starvation). Diving fast with the throttle cut is
// the correct player response.
const FUEL_FIRE_EXT_LO = 0.02; // per second, IAS < 150 m/s
const FUEL_FIRE_EXT_HI = 0.08; // per second, IAS > 300 m/s
const ENGINE_FIRE_EXT_THR0 = 0.15; // per second, throttle cut to 0

/** Component HP per zone (damage-model §7, tuned to gun 12/hit,
 *  missile 40, bomb ≤60). `destroyed` = fuselage 0, OR both engines 0,
 *  OR tail 0 (controls gone), OR total 0 — the WT severe-damage rule.
 *  Wing 0 = wing detaches → dying spin. Fuel 0 = fuel fire starts.
 *  F-35A is single-engine (one `engine` zone, not engineL/engineR);
 *  losing it is a severe-damage kill (all engines destroyed). */
function zoneHpFor(specKey: string): Record<string, number> {
  switch (specKey) {
    case 'f35a':
      return {
        fuselage: 40, wingL: 25, wingR: 25, tail: 20, fin: 15,
        engine: 25, fuel: 20,
      };
    case 'f22':
      return {
        fuselage: 45, wingL: 30, wingR: 30, tail: 22, fin: 15,
        engineL: 22, engineR: 22, fuel: 22,
      };
    case 'f15e':
      return {
        fuselage: 42, wingL: 28, wingR: 28, tail: 20, fin: 15,
        engineL: 22, engineR: 22, fuel: 22,
      };
    case 'mig29':
      return {
        fuselage: 38, wingL: 25, wingR: 25, tail: 18, fin: 15,
        engineL: 20, engineR: 20, fuel: 18,
      };
    case 'su34':
      return {
        fuselage: 45, wingL: 30, wingR: 30, tail: 22, fin: 15,
        engineL: 22, engineR: 22, fuel: 25,
      };
    case 'fa18':
    default:
      return {
        fuselage: 40, wingL: 25, wingR: 25, tail: 20, fin: 15,
        engineL: 20, engineR: 20, fuel: 20,
      };
  }
}

/** Damage zone names accepted by applyDamage(). 'wing' / 'engine' are
 *  the v2 template zones (WeaponSystem passes these); the per-side
 *  variants let callers hit one side only. 'fuel' is the fuel tank
 *  (damage-model §3 — a hit can start a fuel fire); 'fin' is the
 *  vertical fin (damage-model §7). */
export type DamageZone =
  | 'wing'
  | 'wingL'
  | 'wingR'
  | 'engine'
  | 'engineL'
  | 'engineR'
  | 'tail'
  | 'fin'
  | 'fuel'
  | 'fuselage';

/** Spawn point: runway centre, elevation and heading (deg). */
export interface RunwaySpawn {
  x: number;
  z: number;
  elev: number;
  hdg: number;
}

/** Failure flags set by the G7 failure system (main.ts). */
export interface AircraftFailures {
  engineOut?: boolean;
  engineSide?: -1 | 0 | 1;
  hydroDegraded?: boolean;
  fuelLeak?: boolean;
  fire?: boolean;
  gearJam?: boolean;
}

/** Minimal carrier-deck surface the controller needs. The carrier
 *  worker owns the carrier itself; this is only the interaction surface
 *  (template carrierDeckY / carrierRelative / carrier.vel / yawRate). */
export interface DeckProvider {
  alive: boolean;
  /** Deck height at world (x, z), or -Infinity when not over the deck. */
  deckHeightAt(x: number, z: number): number;
  /** Deck velocity, m/s (world frame). */
  velocity(): THREE.Vector3;
  /** Deck yaw rate, rad/s. */
  yawRate(): number;
  /** Carrier-local coords, or null when the provider cannot convert. */
  local(x: number, z: number): { x: number; z: number } | null;
}

/** Hooks the integrator wires to visuals / audio / score. The
 *  controller never imports CrashVFXManager, the HUD or sfx — main.ts
 *  connects these. */
export interface AircraftEvents {
  /** Aircraft went down (reason: 'shot down' | 'hard landing' |
   *  'terrain' | 'engine fire' | ...). Main plays the explosion. */
  onCrash?: (ac: Aircraft, reason: string) => void;
  /** Aircraft entered the dying fall (damage-model §6 step 1) — shot
   *  down but still falling, not yet dead. Main starts the spectate
   *  camera for the player and shows the "SHOT DOWN" banner. */
  onDying?: (ac: Aircraft, reason: string) => void;
  /** A wing/tail segment was detached by fatal damage. Main hands the
   *  mesh to CrashVFXManager.spawnDebris() as a tumbling Rapier body.
   *  `segment` is null when the model had no such node (box fallback). */
  onDetachSegment?: (
    ac: Aircraft,
    segment: THREE.Object3D | null,
    pos: THREE.Vector3,
    vel: THREE.Vector3,
  ) => void;
  /** Arresting wire caught (hook down, in the wires, under 72 m/s). */
  onTrap?: (ac: Aircraft) => void;
  /** Wire snapped (in the wires at 72 m/s or faster). */
  onWireSnap?: (ac: Aircraft) => void;
  /** Flares were fired (bookkeeping already done by fireFlares()).
   *  Main wires the VISUALS here (6 sprites) — do NOT call
   *  WeaponSystem.fireFlares() from this hook, it would decrement the
   *  flare count a second time. */
  onFlares?: (ac: Aircraft) => void;
}

export interface AircraftOptions {
  scene: THREE.Scene;
  world: RAPIER.World;
  spec: AircraftSpec;
  isPlayer: boolean;
  modelKey: ModelKey;
  /** Pre-built rig (dependency injection for tests). Normally omitted:
   *  attachModel() builds one from the loaded model. */
  rig?: AircraftRigBindings;
  /** Carrier deck surface (optional; no deck interaction without it). */
  deck?: DeckProvider;
  /** Use the blade-element solver (BET §5) instead of the curve
   *  solver. Default false: the curve solver carries all of the BET
   *  §0–§4 corrections (per-model CL curves, coordinated yaw, spin
   *  state, adverse yaw, thrust lapse, qAuth floor, MiG roll cap,
   *  F-22 TVC) and is the verified default. The BET solver is built
   *  and wired (BladeElementSolver, same IAerodynamicsSolver
   *  contract) but still needs its hover-FCS trim and elevator/lift
   *  calibration pass before it becomes the default — with it on, the
   *  F-35A tumbles in hover (the fan/nozzle thrust split has a large
   *  nose-down moment with no FCS to trim it) and jets over-rotate
   *  on takeoff (elevator authority vs pitch damping gives a ~92°/s
   *  equilibrium pitch rate). Enable per-aircraft for testing. */
  useBet?: boolean;
  events?: AircraftEvents;
}

/** GLB data-URI loader per model key. The modules are generated by
 *  scripts/embed_models.mjs and export a named `*_GLB` const. */
const MODEL_LOADERS: Record<
  Exclude<ModelKey, 'heli'>,
  () => Promise<string>
> = {
  fa18: async () => (await import('../assets/models/fa18')).FA18_GLB,
  f35a: async () => (await import('../assets/models/f35a')).F35A_GLB,
  f22: async () => (await import('../assets/models/f22')).F22_GLB,
  f15e: async () => (await import('../assets/models/f15e')).F15E_GLB,
  mig29: async () => (await import('../assets/models/mig29')).MIG29_GLB,
  su34: async () => (await import('../assets/models/su34')).SU34_GLB,
  ford: async () => (await import('../assets/models/ford')).FORD_GLB,
};

// One GLTFLoader for all aircraft; the MeshoptDecoder registration is
// the v1.1 fix (the F-18 GLB requires EXT_meshopt_compression and v1
// never registered a decoder, so it always fell back to the blocky jet).
const gltfLoader = new GLTFLoader();
gltfLoader.setMeshoptDecoder(MeshoptDecoder);

function countTris(root: THREE.Object3D): number {
  let t = 0;
  root.traverse((o) => {
    if ((o as THREE.Mesh).isMesh) {
      const g = (o as THREE.Mesh).geometry;
      if (g.index) t += g.index.count / 3;
      else if (g.attributes.position) t += g.attributes.position.count / 3;
    }
  });
  return Math.round(t);
}

/** Centre the loaded scene on its bbox centre and scale it to
 *  targetLen along Z (v2 normalizeAircraft verbatim). */
function normalizeAircraft(
  obj: THREE.Object3D,
  targetLen: number,
  noseRotY: number,
): { group: THREE.Group; scale: number; size: THREE.Vector3 } {
  const inner = new THREE.Group();
  inner.add(obj);
  inner.rotation.y = noseRotY;
  const wrap = new THREE.Group();
  wrap.add(inner);
  const box = new THREE.Box3().setFromObject(wrap);
  const size = new THREE.Vector3();
  box.getSize(size);
  const ctr = new THREE.Vector3();
  box.getCenter(ctr);
  inner.position.sub(ctr);
  const s = targetLen / Math.max(size.z, 0.01);
  wrap.scale.setScalar(s);
  return { group: wrap, scale: s, size: size.clone().multiplyScalar(s) };
}

function loadGLB(uri: string): Promise<THREE.Object3D> {
  return new Promise((res, rej) => {
    let done = false;
    const to = setTimeout(() => {
      if (!done) {
        done = true;
        rej(new Error('timeout'));
      }
    }, LOAD_TIMEOUT_MS);
    gltfLoader.load(
      uri,
      (g) => {
        if (!done) {
          done = true;
          clearTimeout(to);
          res(g.scene);
        }
      },
      undefined,
      (e) => {
        if (!done) {
          done = true;
          clearTimeout(to);
          rej(e);
        }
      },
    );
  });
}

/** Procedural low-poly jet (fallback if a GLB fails; v2 proceduralJet). */
function proceduralJet(tint: number, twin: boolean): THREE.Group {
  const g = new THREE.Group();
  const mat = new THREE.MeshLambertMaterial({ color: tint });
  const dark = new THREE.MeshLambertMaterial({ color: 0x2a2f36 });
  const fuse = new THREE.Mesh(new THREE.ConeGeometry(1.1, 7, 6), mat);
  fuse.rotation.x = Math.PI / 2;
  fuse.position.z = 4.5;
  g.add(fuse);
  const body = new THREE.Mesh(new THREE.CylinderGeometry(1.0, 1.3, 9, 6), mat);
  body.rotation.x = Math.PI / 2;
  body.position.z = -1.5;
  g.add(body);
  const wing = new THREE.Mesh(new THREE.BoxGeometry(13.5, 0.35, 4.2), mat);
  wing.position.set(0, -0.2, -1.5);
  g.add(wing);
  const tail = new THREE.Mesh(new THREE.BoxGeometry(5.5, 0.3, 2.6), mat);
  tail.position.set(0, 0.1, -5.6);
  g.add(tail);
  for (const s of [-1, 1]) {
    const fin = new THREE.Mesh(new THREE.BoxGeometry(0.3, 3.4, 2.6), mat);
    fin.position.set(s * 1.3, 1.6, -5.4);
    fin.rotation.z = -s * 0.35;
    g.add(fin);
    const noz = new THREE.Mesh(new THREE.CylinderGeometry(0.75, 0.9, 2.2, 6), dark);
    noz.rotation.x = Math.PI / 2;
    noz.position.set(s * (twin ? 1.1 : 0.7), 0, -6.2);
    g.add(noz);
  }
  const can = new THREE.Mesh(
    new THREE.SphereGeometry(1.0, 8, 6),
    new THREE.MeshLambertMaterial({ color: 0x1a2a3a }),
  );
  can.scale.set(0.9, 0.7, 1.8);
  can.position.set(0, 0.9, 3.2);
  g.add(can);
  return g;
}

/** Procedural low-poly helicopter (v2 proceduralHeli). Nodes are named
 *  so AircraftRigBindings finds the rotors. No heli GLB exists in Drive. */
function proceduralHeli(): THREE.Group {
  const g = new THREE.Group();
  const mat = new THREE.MeshLambertMaterial({ color: 0x4a5a3a });
  const dark = new THREE.MeshLambertMaterial({ color: 0x2a2f36 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(3.4, 3.0, 8.5), mat);
  body.position.z = 1;
  g.add(body);
  const nose = new THREE.Mesh(new THREE.SphereGeometry(1.7, 8, 6), mat);
  nose.scale.set(1, 0.9, 1.2);
  nose.position.set(0, -0.2, 5.2);
  g.add(nose);
  const boom = new THREE.Mesh(new THREE.CylinderGeometry(0.7, 1.1, 8, 6), mat);
  boom.rotation.x = Math.PI / 2;
  boom.position.set(0, 0.6, -6.5);
  g.add(boom);
  const fin = new THREE.Mesh(new THREE.BoxGeometry(0.3, 2.6, 2.2), mat);
  fin.position.set(0, 1.8, -10.2);
  g.add(fin);
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.35, 0.35, 1.6, 6), dark);
  mast.position.y = 2.2;
  g.add(mast);
  const rotor = new THREE.Group();
  rotor.name = 'main_rotor';
  for (let i = 0; i < 4; i++) {
    const b = new THREE.Mesh(new THREE.BoxGeometry(16.4, 0.12, 0.7), dark);
    b.rotation.y = (i * Math.PI) / 4;
    rotor.add(b);
  }
  rotor.position.y = 3.0;
  g.add(rotor);
  const tailRotor = new THREE.Group();
  tailRotor.name = 'tail_rotor';
  for (let i = 0; i < 2; i++) {
    const b = new THREE.Mesh(new THREE.BoxGeometry(3.2, 0.12, 0.5), dark);
    b.rotation.x = (i * Math.PI) / 2;
    tailRotor.add(b);
  }
  tailRotor.position.set(1.0, 1.2, -10.0);
  tailRotor.rotation.z = Math.PI / 2;
  g.add(tailRotor);
  for (const s of [-1, 1]) {
    const skid = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 6, 5), dark);
    skid.rotation.x = Math.PI / 2;
    skid.position.set(s * 1.5, -1.9, 1);
    g.add(skid);
  }
  return g;
}

export class Aircraft {
  readonly spec: AircraftSpec;
  readonly isPlayer: boolean;
  readonly modelKey: ModelKey;
  readonly key: string;
  /** Visual group (synced from the Rapier body every step). */
  readonly group: THREE.Group;
  /** Rapier dynamic body. Rapier owns position; read it via pos/vel/quat. */
  readonly body: RAPIER.RigidBody;
  /** All colliders on the body (fuselage + 3 gear posts). main.ts
   *  registers every handle in colliderOwners so contacts on the gear
   *  posts dispatch into notifyContact() like fuselage contacts. */
  private readonly colliders: RAPIER.Collider[] = [];
  /** Loaded model (null until attachModel resolves). */
  model: THREE.Group | null = null;
  modelReady = false;
  modelStatus: 'pending' | 'ok' | 'failed' = 'pending';
  modelError?: string;
  modelTris = 0;
  rig: AircraftRigBindings | null = null;

  /** Position / velocity / attitude, synced from the body at the start
   *  of every step() (after the integrator's world.step) and in
   *  respawn(). Read these — never the body directly. */
  readonly pos = new THREE.Vector3();
  readonly vel = new THREE.Vector3();
  readonly quat = new THREE.Quaternion();

  /** Template-style stick inputs (main.ts writes these, then calls the
   *  3-argument step overload). pitchIn NEGATIVE pulls up (v2). */
  pitchIn = 0;
  rollIn = 0;
  yawIn = 0;
  /** Throttle TARGET 0..1 (spooled into `throttle` at 0.55/s). */
  thrTarget = 0;
  /** VTOL pad-landing credit flag (main.ts sets / clears it). */
  vtolCounted = false;
  /** Terrain height callback used by the 3-argument step overload.
   *  Main MUST assign the real ground height here (e.g.
   *  `ac.terrainHeightAt = (x, z) => worldGen.groundHeight(x, z)`) —
   *  the default is flat 0, which is only correct on the runway. */
  terrainHeightAt: (x: number, z: number) => number = () => 0;

  alive = true;
  grounded = false;
  onDeck = false;
  /** True while the gear is in physical contact with a surface (set
   *  in stepInternal — NOT the `grounded` flag, whose 0.85×stall gate
   *  flips far too early). The rotation clamp and the ROTATE HUD cue
   *  read this. */
  onSurface = false;
  aoaDeg = 0;
  ias = 0;
  gload = 1;
  vsi = 0;
  stalled = false;
  /** Spin amount 0..1 (BET §2 — the solver's spin state). */
  spin = 0;
  /** Spin command-arrow direction for the HUD: −1 = stick left,
   *  +1 = stick right, 0 = no arrow. */
  spinArrow: -1 | 0 | 1 = 0;
  /** Spin direction: +1 = yawing right, −1 = yawing left. The AI's
   *  spin recovery feeds rudder opposite this. */
  spinDir: 1 | -1 = 1;
  /** Sideslip β, deg (BET §1 — the solver's weathercock input). */
  betaDeg = 0;
  /** Fuel 0..1 (fraction). */
  fuel = 1;
  /** Spooled throttle 0..1 (HUD / solver input). */
  throttle = 0;
  flaps = 0;
  airbrake = 0;
  vectorDeg = 0;
  gear = true;
  hook = false;
  brake = false;
  /** Sim seconds; spawn protection ends at spawnProtectedUntil. */
  spawnProtectedUntil = 0;
  lastHitByIsPlayer = false;
  /** Overall hit points 0..100 (v2 template hp). */
  hp = 100;
  /** Per-zone structural damage 0..1. */
  readonly damageZones: DamageZones = { ...ZERO_DAMAGE };
  /** Single source of truth for per-zone HP (damage-model §6 step 2).
   *  Built from the §7 table for this spec; applyDamage routes through
   *  it, and its severe-damage rule (fuselage / both engines / tail /
   *  total) is what starts the dying fall. */
  readonly damageEntity: DamageableEntity;
  /** Fire state machine (damage-model §3). `fireKind` null = no fire.
   *  `fireIntensity` 0..1 ramps up (fuel 12 s, engine 4 s) and drives
   *  the burn rate and the visuals. `fireZone` is the zone burning. */
  fireKind: 'fuel' | 'engine' | null = null;
  fireIntensity = 0;
  fireZone: DamageZone | null = null;
  /** `dying` (damage-model §6 step 1): the jet is shot down and
   *  falling, but not yet dead — alive stays true for physics, the
   *  solver steps degraded (throttle 0, controls zeroed), and the
   *  ground impact becomes finishCrash(). Untargetable while dying. */
  dying = false;
  dyingReason: string | null = null;
  /** True once finishCrash() has run (ground impact after a dying
   *  fall) — handleDeaths uses it to scale the explosion by the
   *  remaining fuel and scatter debris. */
  crashedFromDying = false;
  lockT = 0;
  lockTarget: Aircraft | null = null;
  /** Gun cooldown, s (WeaponSystem sets it; GUN_COOLDOWN 0.075). */
  gunCd = 0;
  /** Missiles / flares remaining. */
  msl = 6;
  flares = 30;
  flareCd = 0;
  bombs: number;
  failures: AircraftFailures = {};
  lastCrashReason: string | null = null;

  private readonly world: RAPIER.World;
  private readonly deck?: DeckProvider;
  private readonly events: AircraftEvents;
  /** Aerodynamics solver (BET §5): the curve solver is the default —
   *  it carries all of the BET §0–§4 corrections (per-model CL
   *  curves, coordinated yaw, spin state, adverse yaw, thrust lapse,
   *  qAuth floor, MiG roll cap, F-22 TVC). The blade-element solver
   *  (opt-in via AircraftOptions.useBet) is the §5 moment model —
   *  its rates are emergent from the station moments — kept for the
   *  calibration pass (hover-FCS trim, elevator authority). The
   *  rotary archetype always uses the curve solver (the report keeps
   *  the scalar rotor model until BET lands for helis). Both
   *  implement IAerodynamicsSolver, so the controller treats them
   *  identically. */
  private readonly solver: CurveAerodynamicsSolver | BladeElementSolver;
  /** Body-frame angular velocity (rad/s), read from the Rapier body
   *  in syncFromBody and fed to the BET solver as AeroState.angVel. */
  private readonly angVelBody = new THREE.Vector3();
  private simTime = 0;
  private _trapped = false;
  private trapTimer = 0;
  private wingDetached = false;
  private tailDetached = false;
  private fireSpreadTimer = 0;
  private lastHitByRecord: { isPlayer: boolean } | null = null;
  private pendingContact: { impactVy: number; isDeck: boolean } | null = null;
  private static readonly modelCache = new Map<
    ModelKey,
    { group: THREE.Group; tris: number; failed: boolean; error?: string }
  >();

  constructor(opts: AircraftOptions) {
    this.spec = opts.spec;
    this.isPlayer = opts.isPlayer;
    this.modelKey = opts.modelKey;
    this.key = opts.spec.key;
    this.world = opts.world;
    this.deck = opts.deck;
    this.events = opts.events ?? {};
    this.solver =
      opts.useBet && opts.spec.archetype !== 'rotary'
        ? new BladeElementSolver(opts.spec)
        : new CurveAerodynamicsSolver();
    this.rig = opts.rig ?? null;
    this.bombs = opts.spec.bombs ? 4 : 0;
    this.damageEntity = new DamageableEntity(zoneHpFor(opts.spec.key));

    this.group = new THREE.Group();
    opts.scene.add(this.group);

    // Four colliders, not one big box — the takeoff fix (issues 1 + 2).
    //
    // The old single cuboid spanned the whole fuselage (hy 1.6, centre 0),
    // so its bottom corners sat in the runway and Rapier's contact solver
    // cancelled any pitch rotation: the jet could not rotate at all, and
    // its 0.6 friction (0.75 combined with the 0.9 terrain, Rapier's
    // average combine rule) ate ~149 kN of the 196 kN thrust — the
    // observed ~1.3 m/s^2 instead of ~7-9 m/s^2.
    //
    // Now the fuselage rides CLEAR of the ground (belly -0.8 m, i.e. 1.4 m
    // above the surface when parked) and the jet stands on three gear
    // posts whose feet touch the surface (bottom -2.2 m = gear height).
    // Rotation pivots on the main gear: at 12 deg nose-up the tail belly
    // drops 0.30*length*sin(12 deg) ~ 1.17 m for the F-18 and still clears
    // by ~0.2 m, so pitch authority is no longer fought by the ground.
    // Friction is 0 on every collider with the Min combine rule — the
    // controller owns all ground physics (rolling resistance 0.3 m/s^2,
    // brakes, steering, traps, deck-velocity transfer) and Rapier
    // Coulomb friction must not eat thrust a second time.
    //
    // Density is set so the Rapier mass equals spec.mass — the solver
    // divides forces by the same mass, so the two agree.
    const s = opts.spec;
    const hx = Math.max(s.span * 0.28, 1.2);
    const hzFuse = s.length * 0.45;
    const vFuse = 8 * hx * 1.4 * hzFuse;
    // Gear "skis": each post is a 3 m long skid (hz 1.5) rather than a
    // 1 m stub. The posts are physics-only (invisible — the visual gear
    // is part of the GLB), and a long skid spreads the ground contact
    // over several heightfield triangles instead of balancing the whole
    // jet on three 0.7 x 1.0 m pads. That matters because Rapier's
    // heightfield contact solver produces a spurious up-and-back
    // impulse when a small pad straddles a cell boundary / diagonal:
    // measured in the node driver, one world.step changed the F-35A's
    // Rapier velocity from (4.89, -0.16) to (1.30, +1.12) m/s at
    // t=1.35 s / 4.9 m/s on flat ground (flaps 30), launching it into
    // a tumble (AoA -129 deg); the F-15E did the same at 21 m/s and
    // read as a 40 m/s lift-off. A 3 m skid keeps face contacts
    // dominant so a single edge contact cannot dominate the manifold.
    const vMain = 2 * (0.7 * 0.8 * 3.0); // two main skids
    const vNose = 0.6 * 0.8 * 3.0; // one nose skid
    const density = s.mass / (vFuse + vMain + vNose);
    this.body = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(0, 0, 0)
        .setLinearDamping(0)
        .setAngularDamping(0.2)
        .setCcdEnabled(true)
        .setGravityScale(0) // gravity is inside the solver's force sum
        .setEnabled(false), // respawn() enables + places the aircraft
    );
    const minRule = RAPIER.CoefficientCombineRule.Min;
    const mk = (
      hx2: number,
      hy2: number,
      hz2: number,
      x: number,
      y: number,
      z: number,
    ): void => {
      this.colliders.push(
        this.world.createCollider(
          RAPIER.ColliderDesc.cuboid(hx2, hy2, hz2)
            .setTranslation(x, y, z)
            .setDensity(density)
            .setFriction(0)
            .setFrictionCombineRule(minRule)
            .setRestitution(0)
            // Collision events feed the integrator's notifyContact() dispatch.
            .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
          this.body,
        ),
      );
    };
    // Fuselage: belly -0.8 m, top +2.0 m (hy 1.4, centre +0.6).
    mk(hx, 1.4, hzFuse, 0, 0.6, 0);
    // Main gear: two skids at z = -0.15*length, feet at -2.2 m.
    mk(0.35, 0.4, 1.5, -1.6, -1.8, -s.length * 0.15);
    mk(0.35, 0.4, 1.5, 1.6, -1.8, -s.length * 0.15);
    // Nose gear: z = +0.35*length, feet at -2.2 m.
    mk(0.3, 0.4, 1.5, 0, -1.8, s.length * 0.35);
  }

  /** Handles of every collider on this aircraft's body, for the
   *  integrator's colliderOwners registry (fuselage + gear posts). */
  colliderHandles(): number[] {
    return this.colliders.map((c) => c.handle);
  }

  /** Remove the Rapier body and the visual group (round restart). */
  dispose(): void {
    try {
      this.world.removeRigidBody(this.body);
    } catch {
      // already removed
    }
    this.group.removeFromParent();
  }

  /** Sim clock, seconds since construction. */
  get time(): number {
    return this.simTime;
  }

  /** ROTATE cue (BET §3): the HUD shows "ROTATE" when the gear is on
   *  a surface and the speed has reached the per-spec rotation speed.
   *  Gated on onSurface (physical contact), not `grounded` — the
   *  flag's 0.85×stall gate flips far too early (F-22: 46 m/s vs its
   *  72 m/s Vr). */
  get rotateCue(): boolean {
    return (
      this.alive &&
      this.onSurface &&
      this.spec.rotateSpeedKt > 0 &&
      this.ias >= this.spec.rotateSpeedKt * 0.514444
    );
  }

  /** v1.1 spawn protection, seconds REMAINING (template spawnProt). */
  get spawnProt(): number {
    return Math.max(0, this.spawnProtectedUntil - this.simTime);
  }

  /** v1.1 spawn protection: 5 s after (player) respawn. */
  isSpawnProtected(): boolean {
    return this.simTime < this.spawnProtectedUntil;
  }

  /** AI controller driving this aircraft, if any (template ac.ai).
   *  Set by the AI controller's constructor. crash() marks it dead. */
  ai: { state: string } | null = null;

  /** HUD alias: AoA in degrees (same value as aoaDeg). */
  get aoa(): number {
    return this.aoaDeg;
  }

  /** HUD alias: thrust-vector angle in degrees (same as vectorDeg).
   *  Settable — the HUD slider and main.ts assign to it. */
  get vector(): number {
    return this.vectorDeg;
  }

  set vector(v: number) {
    this.vectorDeg = clamp(v, 0, this.spec.maxVectorDeg);
  }

  get isTrapped(): boolean {
    return this._trapped;
  }

  /** Template field: true while an arresting wire holds the jet. */
  get trapped(): boolean {
    return this._trapped;
  }

  /** Template kill-credit record (main.ts: lastHitBy?.isPlayer).
   *  Minimal { isPlayer } — WeaponSystem passes only a boolean, so the
   *  shooter object itself is not retained. */
  get lastHitBy(): { isPlayer: boolean } | null {
    return this.lastHitByRecord;
  }

  /** Template failure flags (accessors over `failures`). */
  get failEngine(): boolean {
    return !!this.failures.engineOut;
  }

  set failEngine(v: boolean) {
    this.failures.engineOut = v;
  }

  get failEngineSide(): -1 | 0 | 1 {
    return this.failures.engineSide ?? 0;
  }

  set failEngineSide(v: -1 | 0 | 1) {
    this.failures.engineSide = v;
  }

  get failFire(): boolean {
    return !!this.failures.fire;
  }

  set failFire(v: boolean) {
    this.failures.fire = v;
  }

  get failHydro(): boolean {
    return !!this.failures.hydroDegraded;
  }

  set failHydro(v: boolean) {
    this.failures.hydroDegraded = v;
  }

  get failGearJam(): boolean {
    return !!this.failures.gearJam;
  }

  set failGearJam(v: boolean) {
    this.failures.gearJam = v;
  }

  get failFuel(): boolean {
    return !!this.failures.fuelLeak;
  }

  set failFuel(v: boolean) {
    this.failures.fuelLeak = v;
  }

  get isWingDetached(): boolean {
    return this.wingDetached;
  }

  fwd(): THREE.Vector3 {
    return new THREE.Vector3(0, 0, 1).applyQuaternion(this.quat);
  }

  up(): THREE.Vector3 {
    return new THREE.Vector3(0, 1, 0).applyQuaternion(this.quat);
  }

  right(): THREE.Vector3 {
    return new THREE.Vector3(1, 0, 0).applyQuaternion(this.quat);
  }

  /** Smoke / fire state for the damage visuals (v2: smoke < 60 hp,
   *  fire < 35 hp). The controller never spawns the plumes itself —
   *  main.ts reads this every frame and drives CrashVFXManager.
   *  The fire state machine (damage-model §3) overrides the hp
   *  thresholds: a burning jet shows fire at any hp, with the kind
   *  (fuel = orange flame + black smoke, engine = localised fire)
   *  and the ramped intensity for the visuals. */
  getDamageFx(): {
    smoke: boolean;
    fire: boolean;
    fireKind: 'fuel' | 'engine' | null;
    fireIntensity: number;
  } {
    return {
      smoke: this.alive && (this.hp < SMOKE_HP || this.fireKind !== null),
      fire: this.alive && (this.hp < FIRE_HP || this.fireKind !== null),
      fireKind: this.fireKind,
      fireIntensity: this.fireIntensity,
    };
  }

  /** Collision-event feed from the integrator: call once per Rapier
   *  contact involving this body (before step()). impactVy is the
   *  vertical velocity at contact (negative = descending). Sets the
   *  grounded test for the next step and the hard-landing check. */
  notifyContact(impactVy: number, isDeck: boolean): void {
    this.pendingContact = { impactVy, isDeck };
  }

  /** Load the GLB for this modelKey (MeshoptDecoder registered — the
   *  v1.1 fix) with a procedural fallback on failure. The template is
   *  cached per key and cloned per aircraft (materials cloned too, so
   *  per-jet tinting never mutates the template). Builds the rig. */
  async attachModel(): Promise<void> {
    let template:
      | { group: THREE.Group; tris: number; failed: boolean; error?: string }
      | undefined = Aircraft.modelCache.get(this.modelKey);
    if (!template) {
      try {
        let group: THREE.Group;
        if (this.modelKey === 'heli') {
          group = proceduralHeli();
        } else {
          const uri = await MODEL_LOADERS[this.modelKey]();
          const scene3 = await loadGLB(uri);
          scene3.traverse((o) => {
            if ((o as THREE.Mesh).isMesh) (o as THREE.Mesh).castShadow = false;
          });
          if (this.modelKey === 'fa18') {
            const pilot = scene3.getObjectByName('helmet.002');
            if (pilot) pilot.visible = false;
          }
          const targetLen =
            this.modelKey === 'ford' ? 333 : this.spec.length;
          const norm = normalizeAircraft(
            scene3,
            targetLen,
            NOSE_ROT_Y[this.modelKey],
          );
          group = norm.group;
        }
        template = { group, tris: countTris(group), failed: false };
        Aircraft.modelCache.set(this.modelKey, template);
      } catch (e) {
        const error = String((e as Error)?.message ?? e);
        const proc = proceduralJet(
          this.spec.tint,
          this.modelKey === 'mig29' || this.modelKey === 'su34',
        );
        if (this.modelKey === 'su34') proc.scale.setScalar(23.3 / 19);
        template = { group: proc, tris: countTris(proc), failed: true, error };
        Aircraft.modelCache.set(this.modelKey, template);
      }
    }
    this.modelStatus = template.failed ? 'failed' : 'ok';
    this.modelError = template.error;
    this.modelTris = template.tris;
    this.model = template.group.clone(true);
    this.model.traverse((o) => {
      if ((o as THREE.Mesh).isMesh && (o as THREE.Mesh).material) {
        (o as THREE.Mesh).material = (
          (o as THREE.Mesh).material as THREE.Material
        ).clone();
      }
    });
    this.group.add(this.model);
    this.modelReady = true;
    if (!this.rig) this.rig = new AircraftRigBindings(this.model, this.spec);
  }

  /** One fixed step — canonical form. Call after the integrator's
   *  world.step() (and after feeding that step's contacts via
   *  notifyContact()). turbAcc is an optional gust acceleration; wind
   *  is an AIRSPEED offset (v1.1 — the wind vector is subtracted from
   *  ground velocity, never added as an acceleration). */
  step(
    dt: number,
    controls: FlightControls,
    wind: THREE.Vector3,
    terrainHeightAt: (x: number, z: number) => number,
    turbAcc?: THREE.Vector3 | null,
  ): void;
  /** One fixed step — template form (main.ts player/AI loop). Inputs
   *  come from the aircraft's public fields (pitchIn / rollIn / yawIn /
   *  thrTarget / flaps / airbrake / vector / gear / hook / brake) and
   *  terrain height from `this.terrainHeightAt` — main MUST assign the
   *  real ground-height function there (default is flat 0). */
  step(dt: number, wind: THREE.Vector3, turbAcc?: THREE.Vector3 | null): void;
  step(
    dt: number,
    controlsOrWind: FlightControls | THREE.Vector3,
    windOrTurb?: THREE.Vector3,
    terrainHeightAt?: (x: number, z: number) => number,
    turbAcc?: THREE.Vector3 | null,
  ): void {
    if (controlsOrWind instanceof THREE.Vector3) {
      const controls: FlightControls = {
        pitch: this.pitchIn,
        roll: this.rollIn,
        yaw: this.yawIn,
        throttle: null, // null = keep thrTarget (main sets it directly)
      };
      this.stepInternal(
        dt,
        controls,
        controlsOrWind,
        this.terrainHeightAt,
        windOrTurb,
      );
      return;
    }
    this.stepInternal(
      dt,
      controlsOrWind,
      windOrTurb ?? new THREE.Vector3(),
      terrainHeightAt ?? this.terrainHeightAt,
      turbAcc,
    );
  }

  private stepInternal(
    dt: number,
    controls: FlightControls,
    wind: THREE.Vector3,
    terrainHeightAt: (x: number, z: number) => number,
    turbAcc?: THREE.Vector3 | null,
  ): void {
    if (!this.alive) return;
    this.simTime += dt;
    if (this.gunCd > 0) this.gunCd -= dt;
    if (this.flareCd > 0) this.flareCd -= dt;
    this.syncFromBody();

    // Dying: the jet is falling, not flying — all controls are zeroed
    // and the throttle is cut (damage-model §6 step 1). The solver
    // steps degraded (stall/spin + drag) so the tumble and the
    // terminal velocity emerge naturally. This override is
    // authoritative: even if main.ts or the AI keeps writing
    // pitchIn/rollIn/yawIn, the solver sees zero.
    if (this.dying) {
      controls = { pitch: 0, roll: 0, yaw: 0, throttle: 0 };
    }

    const pitchIn = controlPitch(controls);
    const rollIn = controlRoll(controls);
    const thr = resolveThrottleTarget(controls);
    if (thr !== null) this.thrTarget = clamp(thr, 0, 1);
    if (controls.flaps !== undefined) this.flaps = clamp(controls.flaps, 0, 30);
    if (controls.airbrake !== undefined) this.airbrake = clamp(controls.airbrake, 0, 1);
    if (controls.vector !== undefined) {
      this.vectorDeg = clamp(controls.vector, 0, this.spec.maxVectorDeg);
    }
    if (controls.gear !== undefined && !this.failures.gearJam) {
      this.gear = controls.gear;
    }
    if (controls.hook !== undefined) this.hook = controls.hook;
    this.brake = controls.brake ?? false;

    // Throttle spool (v2, 0.55/s).
    this.throttle += clamp(
      this.thrTarget - this.throttle,
      -SPOOL_RATE * dt,
      SPOOL_RATE * dt,
    );

    const S = this.spec;
    const m = S.mass;
    const vel = this.vel;
    const vAir = vel.clone().sub(wind);
    this.ias = vAir.length();

    const terrainH = terrainHeightAt(this.pos.x, this.pos.z);
    const deckY =
      this.deck && this.deck.alive
        ? this.deck.deckHeightAt(this.pos.x, this.pos.z)
        : -Infinity;
    const floorY = Math.max(terrainH, deckY);
    const isDeck = deckY > terrainH;

    const result = this.solver.solve({
      velocityAir: vAir,
      quat: this.quat,
      mass: m,
      wingArea: S.wingArea,
      wingAreaLift: S.wingAreaEffective ?? S.wingArea,
      span: S.span,
      chord: S.wingArea / S.span,
      length: S.length,
      throttle: this.throttle,
      thrustMax: S.thrustMax,
      vectorDeg: this.vectorDeg,
      flapsDeg: this.flaps,
      airbrake: this.airbrake,
      gearDown: this.gear,
      damage: this.damageZones,
      altitude: Math.max(this.pos.y, 0),
      density: 1.225 * Math.exp(-Math.max(this.pos.y, 0) / 8500),
      archetype: S.archetype,
      controls,
      heightAboveGround: this.pos.y - floorY,
      controlAuthority: { turn: S.turn, rollDeg: S.rollDeg, pitch: S.pitch },
      failures: {
        engineOut: this.failures.engineOut,
        engineSide: this.failures.engineSide,
        hydroDegraded: this.failures.hydroDegraded,
      },
      // BET per-model data (report §0/§2, supplement §1–§6).
      inducedK: S.inducedK,
      clCurve: S.clCurve,
      angVel: this.angVelBody,
      tvcDeg: S.tvcDeg,
      rollCapSchedule: S.rollCapSchedule,
      // Dying airframe (damage-model §6 step 1): the wing can no
      // longer hold 1 g — lift is cut to 30% so the jet freefalls
      // instead of gliding on with zero controls. Drag is untouched
      // (the solver keeps the full drag on the unscaled CL).
      liftScale: this.dying ? 0.3 : 1,
      dt,
    });
    this.aoaDeg = result.aoaDeg;
    this.gload = result.gLoad;
    this.stalled = result.stalled;
    this.spin = result.spin;
    this.spinArrow = result.spinArrow;
    this.spinDir = result.spinDir;
    this.betaDeg = result.betaDeg;

    // Forces -> acceleration -> new velocity (semi-implicit Euler, v2).
    const acc = result.lift
      .clone()
      .add(result.drag)
      .add(result.thrust)
      .divideScalar(m);
    acc.y -= G;
    if (turbAcc) acc.add(turbAcc);
    const newVel = vel.clone().addScaledVector(acc, dt);

    // --- Ground / deck contact (v2 groundCheck, Rapier-owned) ---
    const gearH = this.gear ? GEAR_HEIGHT : BELLY_HEIGHT;
    const contact = this.pendingContact;
    this.pendingContact = null;
    const deckVel =
      isDeck && this.deck && this.deck.alive
        ? this.deck.velocity()
        : new THREE.Vector3();
    let steerYawRate = 0;
    let deckYawRate = 0;
    // Physical ground contact this step — NOT the `grounded` flag. The
    // flag's 0.85xstall rotation gate flips far too early (F-22: 46 m/s
    // vs its 72 m/s Vr), which released the pitch clamp mid-roll and let
    // the full 55 deg/s authority whip the nose into a tail strike.
    let onSurface = false;
    // Ground-relative horizontal velocity (deck-relative on the carrier).
    let relH = new THREE.Vector3();
    // Tolerance 0.15 m (was 0.05): the runway box colliders sit 5 cm
    // above the analytic height (their top is elev + 0.05, so the gear
    // rests at floorY + gearH + 0.05), which put a resting jet EXACTLY
    // on the old 0.05 threshold — floating-point flicker read it as
    // airborne, grounded/onSurface went false, the rotation clamp
    // released, and the full pitch authority whipped every jet nose-up
    // at 30 kt. 0.15 m still means "on or within a hand's breadth of
    // the ground"; a jet 15 cm up is in ground effect, not flying.
    if (this.pos.y <= floorY + gearH + 0.15 && newVel.y <= 0.5) {
      // Touching a surface. Rapier resolves the penetration — this
      // block only reads the contact, applies roll physics and the
      // v2 runway/deck rules; it never pins pos.y.
      // A dying jet touching the ground is the impact — finishCrash
      // (fuel-scaled explosion + debris + scorch), not a landing.
      if (this.dying) {
        this.finishCrash();
        return;
      }
      onSurface = true;
      this.onSurface = true;
      this.grounded = true;
      this.onDeck = isDeck;
      const vs = contact
        ? Math.max(0, -contact.impactVy)
        : Math.max(0, -newVel.y);
      const bank = Math.abs(
        new THREE.Euler().setFromQuaternion(this.quat, 'YXZ').z,
      );
      if (vs > HARD_LANDING_VS || bank > HARD_LANDING_BANK) {
        this.crash('hard landing');
        return;
      }
      // Ground roll, relative to the deck when on it: friction +
      // brakes + nose-wheel steering (v2). Deck velocity passes
      // through untouched (subtracted before friction, added back).
      relH = new THREE.Vector3(newVel.x - deckVel.x, 0, newVel.z - deckVel.z);
      const sp = relH.length();
      const fr = this.brake ? BRAKE_DECEL : GROUND_FRICTION;
      if (sp > 0.5) {
        relH.multiplyScalar(Math.max(0, 1 - (fr * dt) / sp));
        newVel.x = relH.x + deckVel.x;
        newVel.z = relH.z + deckVel.z;
        if (Math.abs(rollIn) > 0.1) {
          steerYawRate = rollIn * STEER_RATE * clamp(sp / 20, 0, 1);
        }
      }
      // Arresting wires (v2: 3 zones across the stern deck, local
      // z −45..−105, |x| < 26; hook down + under 72 m/s = trap).
      if (this.onDeck && this.hook && !this._trapped && this.deck) {
        const rel = this.deck.local(this.pos.x, this.pos.z);
        const inWires =
          rel !== null &&
          rel.z < WIRE_LOCAL_Z_MAX &&
          rel.z > WIRE_LOCAL_Z_MIN &&
          Math.abs(rel.x) < WIRE_LOCAL_X;
        if (inWires && sp < TRAP_MAX_SPEED) {
          this._trapped = true;
          this.trapTimer = 0;
          this.events.onTrap?.(this);
        } else if (inWires && sp >= TRAP_MAX_SPEED) {
          this.events.onWireSnap?.(this);
          this.hook = false;
        }
      }
      if (this._trapped) {
        this.trapTimer += dt;
        const k = Math.max(
          0,
          1 - ((TRAP_BASE_DECEL + this.trapTimer * TRAP_RAMP) * dt) / Math.max(sp, 1),
        );
        newVel.x = deckVel.x + (newVel.x - deckVel.x) * k;
        newVel.z = deckVel.z + (newVel.z - deckVel.z) * k;
        if (sp < 2) {
          this._trapped = false;
          this.thrTarget = 0;
          this.throttle = 0;
        }
      }
      // Deck velocity transfer: a landed jet rides the moving deck.
      if (this.onDeck && this.deck && this.deck.alive) {
        deckYawRate = this.deck.yawRate();
      }
      // Take-off rotation gate (v2): enough speed + pull lifts off
      // naturally via the aero step; the flag just gates roll physics.
      if (sp > S.stallSpeed * 0.85 && pitchIn < -0.25) this.grounded = false;
      else if (sp < 5) this.grounded = true;
    } else {
      this.grounded = false;
      this.onDeck = false;
      onSurface = false;
      this.onSurface = false;
    }
    // Terrain collision safety net: only when actually penetrating.
    // A dying jet penetrating the terrain is the impact — finishCrash.
    if (this.pos.y < terrainH - 0.5) {
      if (this.dying) this.finishCrash();
      else this.crash('terrain');
      return;
    }

    // G-load on the ground is 1.0, not the solver's 0.2 floor (research
    // §6; the play-test saw 0.2 g on a parked jet). The solver's floor
    // stays — this override is the controller's ground truth.
    if (this.grounded) this.gload = 1;

    // G-load + structural limit (v2): over-G damages the airframe.
    if (this.gload > S.maxG + OVER_G_MARGIN && this.ias > OVER_G_MIN_SPEED) {
      this.applyDamage(OVER_G_DPS * dt, 'fuselage', false);
      if (!this.alive) return;
    }

    // Rotation: solver torque -> v2 rates -> Rapier angular velocity.
    // The BET solver returns `rates` directly (integrated from the
    // real moment model, ω̇ = I⁻¹(M − ω × Iω)); the curve solver
    // returns only a torque, recovered here with torqueToRates().
    const rates =
      result.rates ?? torqueToRates(result.torque, m, S.span, S.length);
    // Takeoff rotation (BET report §3): while the gear is on a surface,
    // the nose stays down until the per-spec rotation speed, and even
    // then the rotation rate is capped at ~12 deg/s — the full 55 deg/s
    // authority whips the nose up, the fuselage tail strikes the runway,
    // and Rapier resolves the penetration by launching the jet, which
    // read as a 40 m/s "lift-off" (F-18 flaps 30, F-15E flaps 30) or a
    // tumble (F-35A flaps 0, AoA -129 deg). Gated on physical contact
    // (onSurface), not the grounded flag — the flag's 0.85xstall gate
    // flips far too early for this (F-22: 46 m/s vs its 72 m/s Vr).
    // Pitch-down is left alone (the nose gear stops it). Speed is
    // deck-relative on the carrier.
    if (onSurface && rates.pitchRate < 0) {
      const spdH = Math.hypot(relH.x, relH.z);
      if (spdH < S.rotateSpeedKt * 0.514444) rates.pitchRate = 0;
      else rates.pitchRate = Math.max(rates.pitchRate, -ROTATION_RATE_MAX);
    }
    const angBody = new THREE.Vector3(
      rates.pitchRate,
      rates.yawRate + steerYawRate + deckYawRate,
      rates.rollRate,
    );
    const angWorld = angBody.applyQuaternion(this.quat);
    this.body.setAngvel({ x: angWorld.x, y: angWorld.y, z: angWorld.z }, true);
    this.body.setLinvel({ x: newVel.x, y: newVel.y, z: newVel.z }, true);
    this.vsi = newVel.y;

    // Fuel burn (v2 rates are per second on a 0..100 scale; fuel here
    // is 0..1, so divide by 100 — same ~13 min endurance at full
    // throttle). Empty tanks cut the throttle target.
    this.fuel = Math.max(
      0,
      this.fuel -
        ((FUEL_BURN_BASE +
          this.throttle * FUEL_BURN_THR +
          (this.failures.fuelLeak ? FUEL_LEAK_EXTRA : 0)) *
          dt) /
          100,
    );
    if (this.fuel <= 0 && this.throttle > 0) this.thrTarget = 0;

    // Fire state machine (damage-model §3). The failure-scenario
    // engine fire feeds it (failures.fire starts an engine fire if
    // none is burning); the damage-model fire (fuel/engine, from
    // applyDamage's ignition rolls) is the same machine. A fuel fire
    // ramps to full in ~12 s, an engine fire in ~4 s; fuel burns
    // 2.0 hp/s at full intensity, engine 1.2 hp/s. A fuel fire spreads
    // to one adjacent zone per ~8 s (a wing fire burning long enough
    // destroys the spar). Extinguish: a fuel fire's self-extinguish
    // chance rises with airspeed (2%/s below 150 m/s IAS, 8%/s above
    // 300 m/s); an engine fire has a 15%/s chance to go out when the
    // throttle is cut to 0. A fire that destroys the jet starts the
    // dying fall — it does not vanish in mid-air.
    if (this.failures.fire && !this.fireKind && this.alive && !this.dying) {
      this.startFire('engine', 'engineL');
    }
    if (this.fireKind && this.alive && !this.dying) {
      const ramp = this.fireKind === 'fuel' ? FUEL_FIRE_RAMP_S : ENGINE_FIRE_RAMP_S;
      this.fireIntensity = Math.min(1, this.fireIntensity + dt / ramp);
      const dps =
        this.fireKind === 'fuel' ? FUEL_FIRE_DPS : ENGINE_FIRE_DPS;
      const burn = dps * this.fireIntensity * dt;
      this.hp -= burn;
      // The fire eats the burning zone too (structural damage, so the
      // solver degrades and a wing fire can destroy the spar).
      const dz = this.damageZones;
      const zone = this.fireZone;
      if (zone === 'fuel') dz.fuel = clamp(dz.fuel + burn * 0.004, 0, 1);
      else if (zone === 'engineL' || zone === 'engine') dz.engineL = clamp(dz.engineL + burn * 0.005, 0, 1);
      else if (zone === 'engineR') dz.engineR = clamp(dz.engineR + burn * 0.005, 0, 1);
      // Spread (fuel fire only): one adjacent zone per ~8 s.
      if (this.fireKind === 'fuel') {
        this.fireSpreadTimer += dt;
        if (this.fireSpreadTimer >= FIRE_SPREAD_S) {
          this.fireSpreadTimer = 0;
          const adjacent: DamageZone[] = ['wingL', 'wingR', 'tail', 'engineL', 'engineR'];
          const spreadTo = adjacent[Math.floor(Math.random() * adjacent.length)]!;
          if (spreadTo === 'wingL') dz.wingL = clamp(dz.wingL + 0.15, 0, 1);
          else if (spreadTo === 'wingR') dz.wingR = clamp(dz.wingR + 0.15, 0, 1);
          else if (spreadTo === 'tail') dz.tail = clamp(dz.tail + 0.15, 0, 1);
          else if (spreadTo === 'engineL') dz.engineL = clamp(dz.engineL + 0.15, 0, 1);
          else dz.engineR = clamp(dz.engineR + 0.15, 0, 1);
        }
      }
      // A wing fire burning long enough destroys the spar.
      if ((dz.wingL > WING_LOSS_DMG || dz.wingR > WING_LOSS_DMG) && !this.wingDetached) {
        this.detachWing();
      }
      // Extinguish rolls (per-second probabilities, scaled by dt).
      let extP = 0;
      if (this.fireKind === 'fuel') {
        const t = clamp((this.ias - 150) / 150, 0, 1);
        extP = FUEL_FIRE_EXT_LO + (FUEL_FIRE_EXT_HI - FUEL_FIRE_EXT_LO) * t;
      } else if (this.throttle <= 0.05 && this.thrTarget <= 0.05) {
        extP = ENGINE_FIRE_EXT_THR0;
      }
      if (extP > 0 && Math.random() < extP * dt) {
        this.fireKind = null;
        this.fireIntensity = 0;
        this.fireZone = null;
      }
      if (this.hp <= 0) {
        this.startDying('fire');
        return;
      }
    }

    this.updateRig(dt, controls);
  }

  /** Decoy flares (template Aircraft.fireFlares): 6 sprites, 1.5 s
   *  cooldown, consumes one flare. The AI calls this directly; the
   *  visuals go through events.onFlares (main wires the sprites).
   *  Returns false when there is nothing to fire. NOTE: this is the
   *  bookkeeping path — the player's input path should use
   *  WeaponSystem.fireFlares() instead, never both on one press. */
  fireFlares(): boolean {
    if (!this.alive || this.flares <= 0 || this.flareCd > 0) return false;
    this.flares--;
    this.flareCd = 1.5;
    this.events.onFlares?.(this);
    return true;
  }

  /** Animate bound control surfaces / rotors (v2 updateRig via the rig). */
  updateRig(dt: number, controls: FlightControls): void {
    if (!this.rig || !this.modelReady) return;
    this.rig.update(dt, controls, {
      aoaDeg: this.aoaDeg,
      gear: this.gear,
      hook: this.hook,
      vectorDeg: this.vectorDeg,
      rotorRpm: 4 + this.throttle * 24,
    });
  }

  /** Start a fire (damage-model §3). Fuel fire ramps to full in ~12 s,
   *  engine fire in ~4 s. Called from applyDamage's ignition rolls
   *  and from the engine-fire failure. A fire already burning is not
   *  restarted (its intensity keeps ramping). */
  startFire(kind: 'fuel' | 'engine', zone: DamageZone): void {
    if (!this.alive || this.dying) return;
    if (this.fireKind) return;
    this.fireKind = kind;
    this.fireZone = zone;
    this.fireIntensity = 0;
    this.fireSpreadTimer = 0;
  }

  /** v2 damage(): zone multipliers, per-zone accumulators, wing loss,
   *  smoke/fire thresholds, kill credit. A grounded or spawn-protected
   *  PLAYER cannot be damaged (v1.1 — the AI should not be shooting at
   *  a parked jet); over-G calls with byPlayer=false while airborne
   *  are unaffected.
   *
   *  Damage model (§6 step 2): the hit routes through DamageableEntity
   *  (the single source of truth for per-zone HP, §7 table) AND the
   *  Aircraft's hp/damageZones (which feed the solver's degradation).
   *  A lost zone triggers its effect: fuel → ignition roll (fuel fire),
   *  engine → engine-fire roll, wing → detach, tail → controls dead
   *  (dying), fin → tail authority degraded. Severe damage (WT rule:
   *  fuselage / both engines / tail / total) or hp ≤ 0 starts the
   *  dying fall — the jet does not vanish in mid-air. */
  applyDamage(amount: number, zone: DamageZone, byPlayer: boolean): void {
    if (!this.alive || this.dying) return;
    this.lastHitByIsPlayer = byPlayer;
    this.lastHitByRecord = { isPlayer: byPlayer };
    if (this.isPlayer && (this.grounded || this.isSpawnProtected())) return;
    // Route through the entity (zone HP + severe-damage rule). Map the
    // v2 generic zones to per-side entity zones (a 'wing' hit damages
    // both wings; an 'engine' hit damages both engines).
    const entityZone =
      zone === 'wing' ? 'wingL' : zone === 'engine' ? 'engineL' : zone;
    const result = this.damageEntity.applyDamage(amount, entityZone, byPlayer);
    // A generic 'wing'/'engine' hit damages the other side too (the
    // entity only saw one side above) — check its zone loss as well.
    let lost2: string | null = null;
    if (zone === 'wing') {
      lost2 = this.damageEntity.applyDamage(amount, 'wingR', byPlayer).zoneLost;
    } else if (zone === 'engine') {
      lost2 = this.damageEntity.applyDamage(amount, 'engineR', byPlayer).zoneLost;
    }
    let mult = 1;
    if (zone === 'wing' || zone === 'wingL' || zone === 'wingR') mult = 1.2;
    else if (zone === 'engine' || zone === 'engineL' || zone === 'engineR') mult = 1.5;
    this.hp -= amount * mult;
    const dz = this.damageZones;
    if (zone === 'wing') {
      dz.wingL = clamp(dz.wingL + amount * 0.002, 0, 1);
      dz.wingR = clamp(dz.wingR + amount * 0.002, 0, 1);
    } else if (zone === 'wingL') dz.wingL = clamp(dz.wingL + amount * 0.004, 0, 1);
    else if (zone === 'wingR') dz.wingR = clamp(dz.wingR + amount * 0.004, 0, 1);
    else if (zone === 'engine') {
      dz.engineL = clamp(dz.engineL + amount * 0.0025, 0, 1);
      dz.engineR = clamp(dz.engineR + amount * 0.0025, 0, 1);
    } else if (zone === 'engineL') dz.engineL = clamp(dz.engineL + amount * 0.005, 0, 1);
    else if (zone === 'engineR') dz.engineR = clamp(dz.engineR + amount * 0.005, 0, 1);
    else if (zone === 'tail') dz.tail = clamp(dz.tail + amount * 0.004, 0, 1);
    else if (zone === 'fin') {
      dz.fin = clamp(dz.fin + amount * 0.004, 0, 1);
      // Losing the fin degrades directional stability; the solver has
      // no fin input, so it feeds the tail authority (handling
      // degrades without killing the jet outright, per §7).
      dz.tail = clamp(dz.tail + amount * 0.002, 0, 1);
    } else if (zone === 'fuel') {
      dz.fuel = clamp(dz.fuel + amount * 0.004, 0, 1);
      // A fuel hit leaks and raises the ignition chance; an incendiary
      // (gun API mix) hit ignites it ~5% of the time (BRL Memo 484:
      // AP 0-2%, API ~5%). The missile/bomb proximity path rolls its
      // own 25% fuel-explosion chance in WeaponSystem.
      if (Math.random() < 0.05) this.startFire('fuel', 'fuel');
    }
    // Zone-lost effects (first time a zone's HP hits zero).
    const lost = result.zoneLost ?? lost2;
    if (lost === 'fuel') this.startFire('fuel', 'fuel');
    else if (lost === 'engine' || lost === 'engineL' || lost === 'engineR') {
      // An engine destroyed by a hit has a chance to catch fire.
      if (Math.random() < 0.15) {
        this.startFire('engine', lost as DamageZone);
      }
    } else if (lost === 'tail') {
      // Tail (controls) gone — uncontrollable, start the dying fall.
      this.startDying('tail destroyed');
      return;
    } else if (lost === 'fin') {
      dz.tail = Math.max(dz.tail, 0.6);
    }
    if ((dz.wingL > WING_LOSS_DMG || dz.wingR > WING_LOSS_DMG) && !this.wingDetached) {
      this.detachWing();
    }
    // Severe damage (WT rule) or hp exhausted → the dying fall, not an
    // instant vanish. The kill is credited at this moment (see
    // DamageableEntity.isSeverelyDamaged), not when the wreck hits.
    if (this.damageEntity.isSeverelyDamaged() || this.hp <= 0) {
      this.startDying('shot down');
    }
  }

  /** Detach the worse-damaged wing as a tumbling segment (v2 detachWing:
   *  hide the matching mesh, spawn debris, toast on the player side).
   *  The mesh goes to onDetachSegment; with no wing node the event
   *  still fires with segment=null (CrashVFXManager box fallback). */
  detachWing(): void {
    if (this.wingDetached) return;
    this.wingDetached = true;
    const side =
      this.damageZones.wingL >= this.damageZones.wingR ? 'wing_L' : 'wing_R';
    const segment = this.rig
      ? this.rig.detachSegment(side as 'wing_L' | 'wing_R')
      : null;
    this.events.onDetachSegment?.(this, segment, this.pos.clone(), this.vel.clone());
  }

  /** Detach the tail as a tumbling segment (damage-model §6 step 4 —
   *  the rig already binds `tail_section` per its slot list). Called
   *  when the tail zone is destroyed (spar/fire rules), not only from
   *  a damage threshold. */
  detachTail(): void {
    if (this.tailDetached) return;
    this.tailDetached = true;
    const segment = this.rig ? this.rig.detachSegment('tail_section') : null;
    this.events.onDetachSegment?.(this, segment, this.pos.clone(), this.vel.clone());
  }

  /** Start the dying fall (damage-model §6 step 1). The jet is shot
   *  down: `alive` stays true for physics, but `dying` makes it
   *  untargetable (AI skips it, weapons skip it, lock breaks), zeroes
   *  the throttle, and records the death cause/killer. The aircraft
   *  keeps stepping with the solver running degraded — throttle 0,
   *  controls zeroed — so the stall/spin model, asymmetric roll and
   *  engine-out yaw produce the tumble naturally, and the controller's
   *  gravity + solver drag drive the fall to its terminal velocity.
   *  NOTE: Rapier gravity stays at scale 0 — the controller already
   *  adds `acc.y -= G`, so setting gravityScale = 1 would double-count
   *  gravity (the report's §6 step 1 assumed gravity lived in the
   *  solver's force sum; in this port it lives in the controller). */
  startDying(reason: string): void {
    if (!this.alive || this.dying) return;
    this.dying = true;
    this.dyingReason = reason;
    this.lastCrashReason = reason;
    this.thrTarget = 0;
    this.throttle = 0;
    this.pitchIn = 0;
    this.rollIn = 0;
    this.yawIn = 0;
    if (this.ai) this.ai.state = 'dead';
    this.events.onDying?.(this, reason);
  }

  /** Ground impact after a dying fall (damage-model §6 step 5). The
   *  explosion is scaled by the remaining fuel (a wreck with fuel
   *  left fireballs; without fuel it crumples), debris scatters, and
   *  only then does the aircraft become dead — handleDeaths (via
   *  onCrash) does the explosion/scorch at THIS (impact) position,
   *  not the position where the jet was shot down. */
  finishCrash(): void {
    if (!this.alive) return;
    this.crashedFromDying = true;
    this.alive = false;
    if (
      (this.damageZones.wingL > WING_DETACH_FATAL ||
        this.damageZones.wingR > WING_DETACH_FATAL) &&
      !this.wingDetached
    ) {
      this.detachWing();
    }
    this.group.visible = false;
    this.body.setEnabled(false);
    this.events.onCrash?.(this, this.dyingReason ?? 'crashed');
  }

  /** Instant vanish (v2 crash): hard landing / terrain impact when the
   *  jet was NOT already dying — it is already on the ground, so there
   *  is no fall to spectate. A dying jet never reaches here; its ground
   *  contact calls finishCrash() instead. */
  crash(reason: string): void {
    if (!this.alive) return;
    this.alive = false;
    this.lastCrashReason = reason;
    if (
      (this.damageZones.wingL > WING_DETACH_FATAL ||
        this.damageZones.wingR > WING_DETACH_FATAL) &&
      !this.wingDetached
    ) {
      this.detachWing();
    }
    this.group.visible = false;
    this.body.setEnabled(false);
    if (this.ai) this.ai.state = 'dead';
    this.events.onCrash?.(this, reason);
  }

  /** Template-named alias of respawn(): place at (x, y, z) with heading
   *  hdgDeg. y is the absolute height (respawn adds gear height to the
   *  runway elevation instead, so this subtracts it back). */
  resetAt(x: number, y: number, z: number, hdgDeg: number): void {
    this.respawn({ x, z, elev: y - GEAR_HEIGHT, hdg: hdgDeg });
  }

  /**
   * Give an air-spawned aircraft a flying speed along its nose. Enemies
   * and traffic spawn in mid-air — a jet there is flying, not hovering
   * at 0 kt. Without this the jet spawns below stall speed, the wing
   * cannot hold 1 g, and no steering law can turn it; the AI bleeds
   * altitude and circles while it accelerates from zero. Call after
   * resetAt (respawn zeroes the body velocity). Ground spawns (runway,
   * deck) do not call this.
   */
  setAirspeed(vMs: number): void {
    const f = this.fwd();
    this.body.setLinvel({ x: f.x * vMs, y: 0, z: f.z * vMs }, true);
    this.vel.set(f.x * vMs, 0, f.z * vMs);
    this.ias = vMs;
  }

  /** Reset to a runway (v2 resetAt): repaired, refuelled, re-armed,
   *  5 s spawn protection for the player. The respawn timer is owned
   *  by main.ts (v1.1: tracked, so Restart cancels a pending respawn). */
  respawn(runway: RunwaySpawn): void {
    // Never spawn inside the terrain: the analytic runway elevation can
    // sit below the sampled mesh/collider height at plateau edges, and a
    // spawn inside the heightfield is what the first-Take-Off crash looked
    // like (jet falling nose-high from 5 ft before any throttle input).
    // Clamp the spawn elevation to the real ground height.
    const ground = this.terrainHeightAt
      ? this.terrainHeightAt(runway.x, runway.z)
      : runway.elev;
    const elev = Math.max(runway.elev, ground);
    this.pos.set(runway.x, elev + GEAR_HEIGHT, runway.z);
    this.vel.set(0, 0, 0);
    this.quat.setFromEuler(new THREE.Euler(0, runway.hdg * D2R, 0, 'YXZ'));
    this.throttle = 0;
    this.thrTarget = 0;
    this.pitchIn = 0;
    this.rollIn = 0;
    this.yawIn = 0;
    this.vtolCounted = false;
    this.fuel = 1;
    this.hp = 100;
    this.damageEntity.repair();
    this.damageZones.wingL = 0;
    this.damageZones.wingR = 0;
    this.damageZones.tail = 0;
    this.damageZones.engineL = 0;
    this.damageZones.engineR = 0;
    this.damageZones.fuel = 0;
    this.damageZones.fin = 0;
    this.fireKind = null;
    this.fireIntensity = 0;
    this.fireZone = null;
    this.fireSpreadTimer = 0;
    this.dying = false;
    this.dyingReason = null;
    this.crashedFromDying = false;
    this.alive = true;
    this.msl = 6;
    this.flares = 30;
    this.gunCd = 0;
    this.flareCd = 0;
    this.grounded = true;
    this.onDeck = false;
    this.onSurface = false;
    this.stalled = false;
    this.spin = 0;
    this.spinArrow = 0;
    this.betaDeg = 0;
    this._trapped = false;
    this.trapTimer = 0;
    this.wingDetached = false;
    this.tailDetached = false;
    this.lastHitByIsPlayer = false;
    this.lastHitByRecord = null;
    this.lastCrashReason = null;
    this.spawnProtectedUntil =
      this.simTime + (this.isPlayer ? SPAWN_PROTECTION_S : 0);
    this.bombs = this.spec.bombs ? 4 : 0;
    this.failures = {};
    this.flaps = 0;
    this.airbrake = 0;
    this.vectorDeg = 0;
    this.gear = true;
    this.hook = false;
    this.brake = false;
    this.aoaDeg = 0;
    this.gload = 1;
    this.vsi = 0;
    this.ias = 0;
    this.lockT = 0;
    this.lockTarget = null;
    this.group.visible = true;
    this.body.setEnabled(true);
    this.body.setTranslation(
      { x: this.pos.x, y: this.pos.y, z: this.pos.z },
      true,
    );
    this.body.setRotation(
      { x: this.quat.x, y: this.quat.y, z: this.quat.z, w: this.quat.w },
      true,
    );
    this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.syncFromBody();
  }

  /** Read the Rapier body into pos / vel / quat and the visual group.
   *  Also reads the body's angular velocity (world frame) into
   *  angVelBody (body frame) for the BET solver's v_local = v_cg +
   *  ω × r — the solver needs the true rate, including anything
   *  Rapier's contact solver or angular damping did to it. */
  private syncFromBody(): void {
    const t = this.body.translation();
    this.pos.set(t.x, t.y, t.z);
    const v = this.body.linvel();
    this.vel.set(v.x, v.y, v.z);
    const r = this.body.rotation();
    this.quat.set(r.x, r.y, r.z, r.w);
    const av = this.body.angvel();
    this.angVelBody.set(av.x, av.y, av.z).applyQuaternion(this.quat.clone().invert());
    this.group.position.copy(this.pos);
    this.group.quaternion.copy(this.quat);
  }
}
