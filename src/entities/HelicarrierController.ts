// HelicarrierController.ts — G5 helicarrier (v2 template lines ~1204-1331).
//
// Real Gerald Ford hull (GLB, Meshopt) + 4 procedural rotor pods, cruising
// 40 m/s along waypoints, deck as a Rapier KINEMATIC body so deck velocity
// transfers to landed aircraft, arresting-wire traps and VTOL pads.
//
// INTEGRATION NOTE: the Ford GLB data URI lives in the scaffold's models
// modules (base64 .ts, text-only pushes). Pass it as options.fordGlbUri.
// With no URI (or a failed load) build() falls back to v2's procedural hull,
// so the carrier is never missing. The deck geometry, pods, wires and pads
// are identical either way.
//
// ORDERING: call update(dt) BEFORE world.step() each frame so the kinematic
// deck body's translation / linear velocity apply to that step's contacts.

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import type { FloatingOrigin } from '../world/FloatingOrigin';

// Measured from the GLB (template header): length 52.9 units, deck top Y =
// 1.28, bbox centre Y = 2.80, bow +Z. Scale 333/52.9 = 6.295 -> 333 m.
export const FORD_SCALE = 333 / 52.9;
export const FORD_DECK_LOCAL_Y = (1.28 - 2.8) * FORD_SCALE; // -9.57 m from group origin
export const CARRIER_SPEED = 40; // m/s
export const CARRIER_ALT = 350; // m
/** v2 startRound: ~900 m dead ahead of Alpha (was ~1.1 km at 38 deg left
 *  of the takeoff heading — outside a portrait phone's ~±17 deg FOV, so
 *  the carrier was never seen). Alpha is at (-1400, -900) heading east;
 *  the carrier sits at (-500, -900) heading east to its first waypoint,
 *  so it is on the nose at round start in every viewport. */
export const CARRIER_START_POS = new THREE.Vector3(-500, 350, -900);
/** v2: first leg east from the start position, with the takeoff direction. */
export const CARRIER_WAYPOINTS: THREE.Vector3[] = [
  new THREE.Vector3(1400, 350, -900),
  new THREE.Vector3(700, 350, 1212),
  new THREE.Vector3(-700, 350, 1212),
  new THREE.Vector3(-1400, 350, 0),
  new THREE.Vector3(-700, 350, -1212),
  new THREE.Vector3(700, 350, -1212),
];
/** Rotor pod local positions: [x, y, z] (v2 podPos). */
export const POD_POSITIONS: Array<[number, number, number]> = [
  [-40, -4, 70],
  [40, -4, 70],
  [-40, -4, -70],
  [40, -4, -70],
];
/** Arresting-wire local z positions across the stern deck (v2). */
export const WIRE_Z = [-60, -75, -90];
/** Marked VTOL pad centres, carrier-local (x, z); radius 16 m (v2). */
export const VTOL_PADS: Array<[number, number]> = [
  [0, 60],
  [0, 110],
];
/** Deck footprint half-extents (v2 carrierDeckY check). */
export const DECK_HALF_X = 32;
export const DECK_HALF_Z = 150;

/**
 * Structural contract for the aircraft these methods touch. The real
 * Aircraft (../physics/FlightPhysicsController, v2 Aircraft) satisfies it
 * structurally; swap the annotation to `Aircraft` once that module lands
 * (see the report's open interface notes). Kept as a separate interface
 * rather than redefining Aircraft.
 */
export interface CarrierAircraft {
  pos: THREE.Vector3;
  vel: THREE.Vector3;
  quat: THREE.Quaternion;
  hook: boolean;
  gear: boolean;
  grounded: boolean;
  onDeck: boolean;
  thrTarget: number;
  throttle: number;
  /** VTOL-capable spec flag (F-35A). Only read by tryVtolLand. */
  spec?: { vtol: boolean };
  /** Thrust-vector angle in degrees (F-35A). Only read by tryVtolLand. */
  vector?: number;
}

export interface HelicarrierOptions {
  /** Ford GLB data URI (base64 .ts model module). Omit for the procedural
   *  fallback hull. */
  fordGlbUri?: string;
  /** Original-frame ground height (PandoraWorldGenerator.heightAt). Needed
   *  for the terrain-crash check; without it only the lift<=0 crash applies. */
  heightAt?: (x: number, z: number) => number;
  onPodDestroyed?: (index: number, worldPos: THREE.Vector3) => void;
  onCarrierDown?: () => void;
  onTrap?: (aircraft: CarrierAircraft) => void;
  onWireSnap?: (aircraft: CarrierAircraft) => void;
  onVtolLand?: (aircraft: CarrierAircraft) => void;
}

export interface PodState {
  hp: number;
  alive: boolean;
}

interface Pod {
  hp: number;
  alive: boolean;
  group: THREE.Group;
  disc: THREE.Mesh;
}

const gltfLoader = new GLTFLoader();
// v1.1: the Ford GLB needs MeshoptDecoder registered (same as the jets).
gltfLoader.setMeshoptDecoder(MeshoptDecoder);

function loadGLB(uri: string): Promise<THREE.Object3D> {
  return new Promise((res, rej) => {
    let done = false;
    const to = setTimeout(() => {
      if (!done) {
        done = true;
        rej(new Error('timeout'));
      }
    }, 9000);
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

/** v2 normalizeAircraft: centre on the bbox centre, scale to targetLen
 *  along Z. noseRotY fixes the source nose convention (Ford bow is +Z, so 0). */
function normalizeModel(obj: THREE.Object3D, targetLen: number, noseRotY: number): { group: THREE.Group; scale: number; centerY: number } {
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
  return { group: wrap, scale: s, centerY: ctr.y };
}

export class HelicarrierController {
  readonly group = new THREE.Group();
  /** Scene position of the group origin (v2 carrier.pos). */
  readonly pos: THREE.Vector3 = CARRIER_START_POS.clone();
  /** Horizontal velocity, recomputed each update (v2 carrier.vel). */
  readonly vel = new THREE.Vector3();
  yaw = 0;
  yawRate = 0;
  readonly speed = CARRIER_SPEED;
  readonly alt = CARRIER_ALT;
  alive = true;
  /** Roll list from lost pods, radians (v2 carrier.list). */
  list = 0;
  /** Deck top in carrier-local Y (v2 deckLocalY). -9.57 m with the real GLB. */
  deckLocalY = FORD_DECK_LOCAL_Y;
  modelReady = false;

  private readonly scene: THREE.Scene;
  private readonly world: RAPIER.World;
  private readonly options: HelicarrierOptions;
  private readonly waypoints: THREE.Vector3[] = CARRIER_WAYPOINTS.map((w) => w.clone());
  private wpIndex = 0;
  private readonly podList: Pod[] = [];
  private deckBody: RAPIER.RigidBody | null = null;
  private deckCollider: RAPIER.Collider | null = null;
  /** Progressive trap deceleration state per aircraft (v2 keeps trapped /
   *  trapTimer on the Aircraft; a WeakMap keeps this module from adding
   *  fields to Aircraft). */
  private trapTimers = new WeakMap<CarrierAircraft, number>();
  /** v2 vtolCounted, per aircraft; cleared when the aircraft leaves the deck. */
  private vtolCounted = new WeakSet<CarrierAircraft>();

  constructor(scene: THREE.Scene, world: RAPIER.World, options: HelicarrierOptions = {}) {
    this.scene = scene;
    this.world = world;
    this.options = options;
    this.group.name = 'helicarrier';
    this.scene.add(this.group);
  }

  /**
   * Build the hull (GLB or procedural fallback), the 4 rotor pods, the 3
   * arresting wires and the 2 VTOL pads, then the kinematic deck body.
   * Leaves the carrier at its v2 start state (see reset()).
   */
  async build(): Promise<void> {
    let loaded = false;
    if (this.options.fordGlbUri) {
      try {
        const obj = await loadGLB(this.options.fordGlbUri);
        const norm = normalizeModel(obj, 333, 0); // bow +Z already
        this.group.add(norm.group);
        this.modelReady = true;
        // Deck top in group coords from the measured source value (1.28)
        // and this load's actual bbox centre — the template's -9.57 m is
        // (1.28 - 2.80) * scale for the shipped GLB.
        this.deckLocalY = (1.28 - norm.centerY) * norm.scale;
        loaded = true;
      } catch (e) {
        console.warn('Ford GLB failed, procedural hull', e);
      }
    }
    if (!loaded) this.buildFallbackHull();
    this.buildPods();
    this.buildWiresAndPads();
    this.buildDeckBody();
    this.reset();
  }

  /** v2 buildCarrierFallback: flat deck + island + bow taper. */
  private buildFallbackHull(): void {
    const hullMat = new THREE.MeshLambertMaterial({ color: 0x5a6a72 });
    const deckMat = new THREE.MeshLambertMaterial({ color: 0x2e3136 });
    const hull = new THREE.Mesh(new THREE.BoxGeometry(64, 22, 300), hullMat);
    hull.position.y = this.deckLocalY - 11;
    this.group.add(hull);
    const deck = new THREE.Mesh(new THREE.BoxGeometry(64, 1.5, 300), deckMat);
    deck.position.y = this.deckLocalY;
    this.group.add(deck);
    const island = new THREE.Mesh(new THREE.BoxGeometry(12, 26, 40), hullMat);
    island.position.set(22, this.deckLocalY + 13, -30);
    this.group.add(island);
    const bow = new THREE.Mesh(new THREE.ConeGeometry(32, 60, 4), hullMat);
    bow.rotation.x = Math.PI / 2;
    bow.rotation.y = Math.PI / 4;
    bow.scale.set(1, 1, 0.35);
    bow.position.set(0, this.deckLocalY - 8, 175);
    this.group.add(bow);
  }

  private buildPods(): void {
    // Mid grey-blue, not near-black: the old 0x3a4a55 Lambert pods
    // read as pure black barrels on the deck edge (v2 play-test) —
    // Lambert goes very dark in shadow/dusk, so the base colour
    // itself has to be lighter.
    const podMat = new THREE.MeshLambertMaterial({ color: 0x5a6a75 });
    const discMat = new THREE.MeshBasicMaterial({
      color: 0x9aa8b8,
      transparent: true,
      opacity: 0.35,
      side: THREE.DoubleSide,
    });
    for (let i = 0; i < 4; i++) {
      const pod = new THREE.Group();
      const cyl = new THREE.Mesh(new THREE.CylinderGeometry(6, 6, 16, 8), podMat);
      cyl.rotation.z = Math.PI / 2;
      pod.add(cyl);
      const disc = new THREE.Mesh(new THREE.CircleGeometry(15, 20), discMat);
      disc.rotation.x = -Math.PI / 2;
      disc.position.y = 4;
      pod.add(disc);
      pod.position.set(POD_POSITIONS[i][0], POD_POSITIONS[i][1], POD_POSITIONS[i][2]);
      this.group.add(pod);
      this.podList.push({ hp: 100, alive: true, group: pod, disc });
    }
  }

  private buildWiresAndPads(): void {
    const wireMat = new THREE.MeshBasicMaterial({ color: 0xf2f2f2 });
    for (const wz of WIRE_Z) {
      const wire = new THREE.Mesh(new THREE.BoxGeometry(52, 0.3, 1.4), wireMat);
      wire.position.set(0, this.deckLocalY + 0.6, wz);
      this.group.add(wire);
    }
    const padMat = new THREE.MeshBasicMaterial({ color: 0xffd166, transparent: true, opacity: 0.85, side: THREE.DoubleSide });
    // Mid-grey, not near-black: the old 0x2a2f36 pads read as pure
    // black discs floating in the sky (the v2 play-test's "black
    // helicarrier props").
    const padDiscMat = new THREE.MeshBasicMaterial({ color: 0x5a626a, side: THREE.DoubleSide });
    for (const [px, pz] of VTOL_PADS) {
      const disc = new THREE.Mesh(new THREE.CircleGeometry(13, 24), padDiscMat);
      disc.rotation.x = -Math.PI / 2;
      disc.position.set(px, this.deckLocalY + 0.55, pz);
      this.group.add(disc);
      const ring = new THREE.Mesh(new THREE.RingGeometry(13, 16, 24), padMat);
      ring.rotation.x = -Math.PI / 2;
      ring.position.set(px, this.deckLocalY + 0.6, pz);
      this.group.add(ring);
    }
  }

  /**
   * The deck is a Rapier KINEMATIC (velocity-based) body: the flight
   * physics steps the world, the contact solver reads this body's linear /
   * angular velocity, and a landed aircraft rides the moving deck — the v2
   * manual `pos += carrier.vel * dt` transfer, done by the solver instead.
   * Collider is a box covering the v2 deck footprint (|x| < 32, |z| < 150)
   * with its TOP exactly at deckLocalY.
   */
  private buildDeckBody(): void {
    this.deckBody = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicVelocityBased().setTranslation(this.pos.x, this.pos.y, this.pos.z),
    );
    const desc = RAPIER.ColliderDesc.cuboid(DECK_HALF_X, 0.75, DECK_HALF_Z)
      .setTranslation(0, this.deckLocalY - 0.75, 0)
      // Friction 0 (was 1.0): same takeoff-killer as the terrain — the
      // controller owns deck physics (traps, deck-velocity transfer), so
      // the deck collider must not eat thrust through Coulomb friction.
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0)
      // Collision events feed the integrator's notifyContact() dispatch.
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
    this.deckCollider = this.world.createCollider(desc, this.deckBody);
  }

  /** v2 startRound carrier reset. With no arguments the carrier returns
   *  to its v2 start position and heading; main.ts passes them explicitly
   *  (same values), so both call patterns work. */
  reset(pos?: THREE.Vector3, yaw?: number): void {
    this.alive = true;
    this.pos.copy(pos ?? CARRIER_START_POS);
    this.yaw =
      yaw ??
      Math.atan2(
        CARRIER_WAYPOINTS[0]!.x - this.pos.x,
        CARRIER_WAYPOINTS[0]!.z - this.pos.z,
      );
    this.wpIndex = 0;
    this.list = 0;
    this.yawRate = 0;
    this.vel.set(0, 0, 0);
    for (const pod of this.podList) {
      pod.hp = 100;
      pod.alive = true;
      pod.group.visible = true;
    }
    this.trapTimers = new WeakMap();
    this.vtolCounted = new WeakSet();
    this.group.position.copy(this.pos);
    this.group.rotation.set(0, this.yaw, 0);
    this.syncDeckBody();
  }

  /**
   * v2 carrierUpdate, verbatim behaviour. Call BEFORE world.step() each
   * frame. Waypoint steering (advance within 120 m, yaw rate clamped to
   * +/-0.12 rad/s), 40 m/s cruise, pod lift (each lost pod -25% lift, +7 deg
   * list target), slow sink below 60% lift, terrain / total-lift crash.
   */
  update(dt: number): void {
    if (!this.alive) return;
    const wp = this.waypoints[this.wpIndex];
    const dx = wp.x - this.pos.x;
    const dz = wp.z - this.pos.z;
    const dist = Math.hypot(dx, dz);
    if (dist < 120) this.wpIndex = (this.wpIndex + 1) % this.waypoints.length;
    const targetYaw = Math.atan2(dx, dz);
    let dy = targetYaw - this.yaw;
    while (dy > Math.PI) dy -= Math.PI * 2;
    while (dy < -Math.PI) dy += Math.PI * 2;
    this.yawRate = clamp(dy * 0.5, -0.12, 0.12);
    this.yaw += this.yawRate * dt;
    this.vel.set(Math.sin(this.yaw) * this.speed, 0, Math.cos(this.yaw) * this.speed);
    this.pos.addScaledVector(this.vel, dt);
    // Lift from pods: each lost pod -25% lift, +7 deg list.
    const lift = this.podList.filter((p) => p.alive).length / 4;
    this.list = lerp(this.list, (1 - lift) * 0.12, dt * 0.5);
    if (lift < 0.6) this.pos.y -= (0.6 - lift) * 6 * dt; // sinks slowly
    const groundY = this.options.heightAt ? this.options.heightAt(this.pos.x, this.pos.z) : -Infinity;
    if (lift <= 0.01 || this.pos.y < groundY + 10) {
      this.alive = false;
      this.options.onCarrierDown?.();
      // Deck body stays where it is; the flight physics treats a dead
      // carrier as no deck (deckHeightAt returns -Infinity).
      this.group.position.copy(this.pos);
      this.group.rotation.set(0, this.yaw, this.list);
      return;
    }
    this.group.position.copy(this.pos);
    this.group.rotation.set(0, this.yaw, this.list);
    for (const pod of this.podList) pod.disc.rotation.z += dt * 22;
    this.syncDeckBody();
  }

  private syncDeckBody(): void {
    if (!this.deckBody) return;
    this.deckBody.setTranslation({ x: this.pos.x, y: this.pos.y, z: this.pos.z }, true);
    const q = this.group.quaternion;
    this.deckBody.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
    this.deckBody.setLinvel({ x: this.vel.x, y: this.vel.y, z: this.vel.z }, true);
    this.deckBody.setAngvel({ x: 0, y: this.yawRate, z: 0 }, true);
  }

  /** v2 carrierLocal: world point -> carrier-local (x, z). */
  carrierLocal(p: THREE.Vector3): { x: number; z: number } {
    const dx = p.x - this.pos.x;
    const dz = p.z - this.pos.z;
    const c = Math.cos(this.yaw);
    const s = Math.sin(this.yaw);
    return { x: dx * c - dz * s, z: dx * s + dz * c };
  }

  /** World Y of the deck surface at (x, z), or -Infinity when not over the
   *  deck / carrier dead (v2 carrierDeckY). */
  deckHeightAt(x: number, z: number): number {
    if (!this.alive) return -Infinity;
    const l = this.carrierLocal(new THREE.Vector3(x, 0, z));
    if (Math.abs(l.x) < DECK_HALF_X && Math.abs(l.z) < DECK_HALF_Z) return this.pos.y + this.deckLocalY;
    return -Infinity;
  }

  /** Alias of deckHeightAt (main.ts call pattern). */
  deckHeight(x: number, z: number): number {
    return this.deckHeightAt(x, z);
  }

  /**
   * Carrier-local position of a world point, as a Vector3 (x, 0, z) in
   * carrier-local coords — v2 carrierRelative(). Returns null when the
   * carrier is dead or the point is outside the deck footprint, so callers
   * doing pad/wire checks (which only make sense on the deck) can null-
   * check instead of testing bounds themselves.
   */
  localPosition(p: THREE.Vector3): THREE.Vector3 | null {
    if (!this.alive) return null;
    const l = this.carrierLocal(p);
    if (Math.abs(l.x) >= DECK_HALF_X || Math.abs(l.z) >= DECK_HALF_Z) return null;
    return new THREE.Vector3(l.x, 0, l.z);
  }

  /** Live deck origin (v2 carrier.pos). */
  get deckPos(): THREE.Vector3 {
    return this.pos;
  }

  /** Live deck velocity (v2 carrier.vel). */
  get deckVel(): THREE.Vector3 {
    return this.vel;
  }

  /** World Y of the deck surface. */
  get deckTopY(): number {
    return this.pos.y + this.deckLocalY;
  }

  /** Pod health snapshot: { hp, alive } per pod (v2 podHp). */
  get podsState(): PodState[] {
    return this.podList.map((p) => ({ hp: p.hp, alive: p.alive }));
  }

  /** Alias matching the task's `pods` field name. */
  get pods(): PodState[] {
    return this.podsState;
  }

  /**
   * The four rotor pod groups as THREE.Object3D (main.ts's
   * carrierPodPositions maps pods to world positions for the weapons
   * module). Use this — NOT `pods` (which is {hp, alive} state) — when a
   * world position is needed.
   */
  get podObjects(): THREE.Object3D[] {
    return this.podList.map((p) => p.group);
  }

  /** Kinematic deck body — register it with the FloatingOrigin. */
  get deckBodyRef(): RAPIER.RigidBody | null {
    return this.deckBody;
  }

  get deckColliderRef(): RAPIER.Collider | null {
    return this.deckCollider;
  }

  /**
   * Register the carrier with the floating origin so a shift moves the
   * group, the logical position, the waypoints and the deck body
   * together (main.ts calls this once, after build()).
   */
  registerWithFloatingOrigin(origin: FloatingOrigin): void {
    origin.registerObject(this.group);
    origin.registerVector(this.pos);
    for (const w of this.waypoints) origin.registerVector(w);
    if (this.deckBody) origin.registerBody(this.deckBody);
  }

  /**
   * Arresting-wire trap (v2 groundCheck wire block). Call every frame while
   * an aircraft is touching the deck with its hook down. Returns true while
   * a trap is engaged this frame (and applies the progressive hydraulic
   * deceleration to aircraft.vel); false otherwise. Hook down + touchdown
   * in a wire zone (local z -45..-105, |x| < 26) under 72 m/s catches the
   * wire; over 72 m/s the wire snaps (hook lowered, onWireSnap fired).
   * When horizontal speed drops under 2 m/s the trap releases and the
   * throttles are cut, as in v2. Fixed 1/60 s step — use tryTrapDt when the
   * caller has the real frame dt.
   */
  tryTrap(aircraft: CarrierAircraft): boolean {
    return this.trapStep(aircraft, 1 / 60);
  }

  /** Same as tryTrap, with the caller's real frame dt. */
  tryTrapDt(aircraft: CarrierAircraft, dt: number): boolean {
    return this.trapStep(aircraft, dt);
  }

  private trapStep(aircraft: CarrierAircraft, dt: number): boolean {
    if (!this.alive) return false;
    const deckY = this.deckHeightAt(aircraft.pos.x, aircraft.pos.z);
    if (deckY === -Infinity) {
      this.trapTimers.delete(aircraft);
      return false;
    }
    const gearH = aircraft.gear ? 2.2 : 1.2;
    const onDeck =
      Math.abs(aircraft.pos.y - (deckY + gearH)) < 1.5 && aircraft.vel.y <= 0.5;
    if (!onDeck) {
      this.trapTimers.delete(aircraft);
      return false;
    }
    const sp = Math.hypot(aircraft.vel.x, aircraft.vel.z);
    if (!this.trapTimers.has(aircraft)) {
      if (!aircraft.hook) return false;
      const rel = this.carrierLocal(aircraft.pos);
      const inWires = rel.z < -45 && rel.z > -105 && Math.abs(rel.x) < 26;
      if (!inWires) return false;
      if (sp < 72) {
        this.trapTimers.set(aircraft, 0);
        this.options.onTrap?.(aircraft);
        return true;
      }
      aircraft.hook = false;
      this.options.onWireSnap?.(aircraft);
      return false;
    }
    // Already trapped: progressive hydraulic decel (v2).
    const timer = (this.trapTimers.get(aircraft) ?? 0) + dt;
    this.trapTimers.set(aircraft, timer);
    aircraft.vel.multiplyScalar(Math.max(0, 1 - ((14 + timer * 30) * dt) / Math.max(sp, 1)));
    if (sp < 2) {
      this.trapTimers.delete(aircraft);
      aircraft.thrTarget = 0;
      aircraft.throttle = 0;
      return false;
    }
    return true;
  }

  /**
   * VTOL pad landing (v2 VTOL pad credit, main-loop block): F-35A with
   * vector >= 70 deg, touching down on a MARKED pad (|local dx| < 18 and
   * |local dz| < 18 of a pad centre), with velocity matched to the deck
   * (relative speed < 6 m/s) and a gentle descent (vel.y > -4 m/s).
   * While descending over a pad, horizontal velocity is blended toward the
   * deck velocity ("airspeed-matched"). Returns true on a qualifying
   * touchdown (once per landing — re-arms after the aircraft leaves the
   * deck). The onVtolLand callback is where the game manager scores it
   * (v2: +75).
   */
  tryVtolLand(aircraft: CarrierAircraft, dt = 1 / 60): boolean {
    if (!this.alive) return false;
    const deckY = this.deckHeightAt(aircraft.pos.x, aircraft.pos.z);
    if (deckY === -Infinity || aircraft.pos.y > deckY + 12) {
      this.vtolCounted.delete(aircraft);
      return false;
    }
    const rel = this.carrierLocal(aircraft.pos);
    const onPad = VTOL_PADS.some(([px, pz]) => Math.abs(rel.x - px) < 18 && Math.abs(rel.z - pz) < 18);
    if (!onPad) return false;
    if (aircraft.spec?.vtol && (aircraft.vector ?? 0) >= 70) {
      const k = Math.min(1, dt * 2);
      aircraft.vel.x += (this.vel.x - aircraft.vel.x) * k;
      aircraft.vel.z += (this.vel.z - aircraft.vel.z) * k;
    }
    const gearH = aircraft.gear ? 2.2 : 1.2;
    const touchedDown = aircraft.pos.y <= deckY + gearH + 0.5 && aircraft.vel.y <= 0.5;
    if (!touchedDown) return false;
    if (!aircraft.spec?.vtol || (aircraft.vector ?? 0) < 70) return false;
    const relV = aircraft.vel.clone().sub(this.vel).length();
    if (relV < 6 && aircraft.vel.y > -4) {
      if (!this.vtolCounted.has(aircraft)) {
        this.vtolCounted.add(aircraft);
        this.options.onVtolLand?.(aircraft);
      }
      return true;
    }
    return false;
  }

  /**
   * v2 carrierPodDamage: destroyed pod reduces lift (see update(): -25%
   * lift, +7 deg list per lost pod), hides the pod, fires onPodDestroyed
   * (the game manager spawns the smoke/fire FX — v2 makeSmokePlume lives
   * in the G6 FX module, not here). Altitude loss and the eventual terrain
   * crash happen in update().
   */
  damagePod(index: number, amount: number): void {
    if (!this.alive) return;
    const pod = this.podList[index];
    if (!pod || !pod.alive) return;
    pod.hp -= amount;
    if (pod.hp <= 0) {
      pod.hp = 0;
      pod.alive = false;
      pod.group.visible = false;
      const worldPos = pod.group.getWorldPosition(new THREE.Vector3());
      this.options.onPodDestroyed?.(index, worldPos);
    }
  }
}

function clamp(v: number, a: number, b: number): number {
  return Math.max(a, Math.min(b, v));
}

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}
