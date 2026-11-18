// HelicarrierController.ts — G5 helicarrier (v2 template lines ~1204-1331).
//
// SHIELD helicarrier hull (converted Printables roboticide model, CC
// BY-NC 4.0 — see README attribution) + its 4 rotor pods with spinning
// discs, cruising 40 m/s along waypoints, deck as a Rapier KINEMATIC
// body so deck velocity transfers to landed aircraft, arresting-wire
// traps and VTOL pads. The Gerald Ford GLB is retired as the carrier
// model (ford.ts stays in the repo, unused).
//
// INTEGRATION NOTE: the SHIELD GLB data URI lives in the scaffold's
// models modules (base64 .ts, text-only pushes). Pass it as
// options.modelGlbUri. With no URI (or a failed load) build() falls
// back to v2's procedural hull, so the carrier is never missing. The
// deck geometry, pods, wires and pads are derived from the loaded
// model where possible (deck_top / duct_ring / rotor_disc meshes) and
// fall back to the Ford measurements otherwise.
//
// ORDERING: call update(dt) BEFORE world.step() each frame so the kinematic
// deck body's translation / linear velocity apply to that step's contacts.

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import type { FloatingOrigin } from '../world/FloatingOrigin';

// SHIELD helicarrier, measured from the converted GLB (see
// src/assets/models/helicarrier.ts): 333 m long, bow +Z, deck top Y =
// 29.78 in source coords, bbox centre Y = 35.06, scale 1.0. The deck
// is two sections: a wide stern deck (71.7 m) and a narrower bow deck
// (48.1 m) — the collider and the bounds checks use both rectangles.
export const SHIELD_DECK_TOP_Y = 29.78;
export const SHIELD_CENTER_Y = 35.06;
/** Ford deck-local Y, used only by the procedural fallback hull
 *  (measured from the Ford GLB: (1.28 - 2.80) * (333/52.9)). */
export const FALLBACK_DECK_LOCAL_Y = (1.28 - 2.8) * (333 / 52.9);
/** Deck footprint half-extents for the procedural fallback hull (the
 *  Ford's 64 x 300 m deck, v2 carrierDeckY check). */
export const DECK_HALF_X = 32;
export const DECK_HALF_Z = 150;
export const CARRIER_SPEED = 40; // m/s
export const CARRIER_ALT = 350; // m
/** v4 startRound: ~900 m dead ahead of Alpha (was ~1.1 km at 38 deg left
 *  of the takeoff heading — outside a portrait phone's ~±17 deg FOV, so
 *  the carrier was never seen). Alpha is at (-2800, -1800) heading east;
 *  the carrier sits at (-1900, -1800) heading east to its first waypoint,
 *  so it is on the nose at round start in every viewport. */
export const CARRIER_START_POS = new THREE.Vector3(-1900, 350, -1800);
/** v4: first leg east from the start position, with the takeoff
 *  direction. Hexagon of ~3000 m radius across the 12 km basin — the
 *  legs stay inside the range bands (terrain < ~130 m on the loop),
 *  so the carrier cruises at its 350 m altitude with ~200 m of
 *  clearance; the terrain-following altitude (issue 8) is the safety
 *  net for the range crossings, not the norm. */
export const CARRIER_WAYPOINTS: THREE.Vector3[] = [
  new THREE.Vector3(3000, 350, -1800),
  new THREE.Vector3(1500, 350, 2600),
  new THREE.Vector3(-1500, 350, 2600),
  new THREE.Vector3(-3000, 350, 0),
  new THREE.Vector3(-1500, 350, -2600),
  new THREE.Vector3(1500, 350, -2600),
];

/** Per-carrier loop for the helicarrier count option (issue 5): the
 *  base hexagon scaled about the origin, with a start position scaled
 *  the same way (so it stays 900 m ahead of Alpha's latitude on the
 *  leg into waypoint 0). Scale 1.0 is the primary carrier; 0.75 and
 *  1.25 are the second and third, on concentric loops that never
 *  overlap or collide. `startIndex` spreads the carriers around
 *  their circuits (0 / 2 / 4) instead of stacking them. */
export function carrierLoop(
  scale: number,
): { waypoints: THREE.Vector3[]; startPos: THREE.Vector3 } {
  return {
    waypoints: CARRIER_WAYPOINTS.map(
      (w) => new THREE.Vector3(w.x * scale, w.y, w.z * scale),
    ),
    startPos: new THREE.Vector3(
      CARRIER_START_POS.x * scale,
      CARRIER_START_POS.y,
      CARRIER_START_POS.z * scale,
    ),
  };
}
/** Rotor pod local positions for the PROCEDURAL fallback hull: [x, y, z]
 *  (v2 podPos). The SHIELD model derives its pod positions from its own
 *  duct_ring meshes at build time (see buildShieldPods). */
export const POD_POSITIONS: Array<[number, number, number]> = [
  [-40, -4, 70],
  [40, -4, 70],
  [-40, -4, -70],
  [40, -4, -70],
];
/** Arresting-wire local z positions across the stern deck (v2). They sit
 *  in the SHIELD deck's wide stern section too (local z -128.9..+2.0), so
 *  the same positions serve both models. */
export const WIRE_Z = [-60, -75, -90];
/** Marked VTOL pad centres, carrier-local (x, z); radius 16 m (v2). They
 *  sit in the SHIELD deck's narrow bow section (local z +2..+152.5,
 *  half-width 24 m), forward of the island (which ends at local z +20),
 *  so the same positions serve both models. */
export const VTOL_PADS: Array<[number, number]> = [
  [0, 60],
  [0, 110],
];

/** One deck rectangle, carrier-local coords: centre (cx, cz) and
 *  half-extents (hx, hz). The deck is the union of these rectangles
 *  (the SHIELD deck is two: wide stern + narrow bow). */
export interface DeckRect {
  cx: number;
  cz: number;
  hx: number;
  hz: number;
}

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
  /** Carrier model GLB data URI (base64 .ts model module). Omit for the
   *  procedural fallback hull. (Was `fordGlbUri` — renamed when the
   *  SHIELD model replaced the Ford.) */
  modelGlbUri?: string;
  /** Deck top Y in the source model's coordinates, before
   *  normalisation. 29.78 for the SHIELD model; the Ford's 1.28 is the
   *  default for a model that does not carry deck_top meshes. */
  deckTopSourceY?: number;
  /** Original-frame ground height (PandoraWorldGenerator.heightAt). Needed
   *  for the terrain-crash check; without it only the lift<=0 crash applies. */
  heightAt?: (x: number, z: number) => number;
  /** Per-carrier loop waypoints (issue 5 — helicarrier count 0-3).
   *  Defaults to CARRIER_WAYPOINTS. Each carrier gets its own loop
   *  (a scaled hexagon, so the loops are concentric and never overlap)
   *  and its own start index, so the carriers are spread around their
   *  circuits instead of stacked on top of each other. */
  waypoints?: THREE.Vector3[];
  /** Index into `waypoints` the carrier starts steering toward
   *  (default 0). Combined with a scaled loop, this spreads the
   *  carriers around the map. */
  startIndex?: number;
  onPodDestroyed?: (index: number, worldPos: THREE.Vector3) => void;
  onCarrierDown?: () => void;
  /** Fired when a dying carrier hits the ground (issue 9): the game
   *  manager spawns the large impact explosion + debris + scorch.
   *  The carrier is left as a tilted wreck (alive stays true while
   *  it smokes); onCarrierDown fires ~10 s later when the wreck is
   *  finally written off. */
  onCarrierImpact?: (worldPos: THREE.Vector3) => void;
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
  /** Representative object for world-position queries (the pod group for
   *  procedural pods, the rotor disc for SHIELD pods). */
  group: THREE.Object3D;
  /** The spinning rotor (procedural disc, or the GLB's rotor_disc mesh
   *  re-pivoted about its duct-ring centre). */
  disc: THREE.Mesh;
  /** Every mesh in the pod (disc + hub + duct ring + housing for SHIELD;
   *  the single group for procedural) — hidden/shown together on
   *  damage / reset. */
  meshes: THREE.Object3D[];
  /** Spin axis for the disc: 'z' for the procedural disc (a flat circle
   *  rotated x=-90 deg, so z spins it in its plane), 'y' for the GLB
   *  rotor (re-pivoted about its vertical mast). */
  spinAxis: 'y' | 'z';
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
 *  along Z. noseRotY fixes the source nose convention (SHIELD bow is +Z,
 *  so 0). Returns the full bbox centre (needed to map model-space
 *  measurements — deck rects, pod centres — into local coords). */
function normalizeModel(obj: THREE.Object3D, targetLen: number, noseRotY: number): { group: THREE.Group; scale: number; center: THREE.Vector3 } {
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
  return { group: wrap, scale: s, center: ctr };
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
  /** Dying (issue 9): all pods dead (lift <= 0.01). The carrier
   *  stops waypoint-following, lists heavily, and sinks at an
   *  increasing rate — never a silent vanish or an intact fall.
   *  On ground contact onCarrierImpact fires (large explosion +
   *  debris + scorch) and the group is left as a tilted wreck;
   *  ~10 s later alive=false and onCarrierDown fires. While dying
   *  the terrain-following climb is overridden (the carrier must
   *  sink even though the climb would try to lift it). */
  dying = false;
  /** Seconds since dying started. */
  dyingT = 0;
  /** True once the wreck has hit the ground (impactDone). The deck
   *  reads as absent from then on (deckHeightAt -> -Infinity) even
   *  though alive stays true while the wreck smokes. */
  impactDone = false;
  /** Seconds since impact (wreck smoking time). */
  wreckT = 0;
  /** Active in this round (issue 5 — helicarrier count). A carrier
   *  beyond the chosen count is deactivated: it is not updated, its
   *  deck reads as absent, and its group + deck body are hidden /
   *  disabled, so it is fully out of the game. reset() re-activates. */
  active = true;
  /** Roll list from lost pods, radians (v2 carrier.list). */
  list = 0;
  /** Deck top in carrier-local Y. Set at build() from the loaded model
   *  (SHIELD: (29.78 - 35.06) * 1.0 = -5.28 m); the procedural fallback
   *  uses the Ford value. */
  deckLocalY = FALLBACK_DECK_LOCAL_Y;
  /** Deck footprint as carrier-local rectangles (the union is the
   *  deck). Set at build() from the loaded model (SHIELD: wide stern
   *  + narrow bow, from the deck_top meshes) or the fallback rect. */
  deckRects: DeckRect[] = [{ cx: 0, cz: 0, hx: DECK_HALF_X, hz: DECK_HALF_Z }];
  modelReady = false;

  private readonly scene: THREE.Scene;
  private readonly world: RAPIER.World;
  private readonly options: HelicarrierOptions;
  private readonly waypoints: THREE.Vector3[];
  private wpIndex: number;
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
    this.waypoints =
      options.waypoints?.map((w) => w.clone()) ??
      CARRIER_WAYPOINTS.map((w) => w.clone());
    this.wpIndex = options.startIndex ?? 0;
    this.group.name = 'helicarrier';
    this.scene.add(this.group);
  }

  /**
   * Build the hull (SHIELD GLB or procedural fallback), the 4 rotor
   * pods, the 3 arresting wires and the 2 VTOL pads, then the kinematic
   * deck body. Leaves the carrier at its v2 start state (see reset()).
   *
   * SHIELD model: the deck top, deck rectangles and pod positions are
   * derived from the GLB's own meshes (deck_top_* for the footprint,
   * duct_ring_* for the pod centres, rotor_disc_* for the spinning
   * rotors) so the collider and the damage zones match the visuals
   * exactly. The procedural pods are NOT built for the SHIELD model —
   * the GLB already has its own pods, and doubling up would put a
   * second set of barrels on the deck edge. If the GLB has no
   * recognisable pod/deck meshes, buildPods() and the Ford deck rect
   * are used instead.
   */
  async build(): Promise<void> {
    let loaded = false;
    if (this.options.modelGlbUri) {
      try {
        const obj = await loadGLB(this.options.modelGlbUri);
        const norm = normalizeModel(obj, 333, 0); // bow +Z already
        this.group.add(norm.group);
        this.modelReady = true;
        // Deck top in group coords from the measured source value
        // (29.78 for the SHIELD model) and this load's actual bbox
        // centre — (29.78 - 35.06) * 1.0 = -5.28 m.
        const deckTopY = this.options.deckTopSourceY ?? SHIELD_DECK_TOP_Y;
        this.deckLocalY = (deckTopY - norm.center.y) * norm.scale;
        // Deck rectangles from the GLB's deck_top meshes (model space
        // -> local: subtract the bbox centre, scale).
        const rects = this.deckRectsFromModel(obj, norm.center, norm.scale);
        if (rects.length > 0) this.deckRects = rects;
        // Rotor pods from the GLB's own meshes; fall back to the
        // procedural pods if the model has none recognisable.
        if (!this.buildShieldPods(obj, norm.center, norm.scale)) {
          this.buildPods();
        }
        loaded = true;
      } catch (e) {
        console.warn('Carrier GLB failed, procedural hull', e);
      }
    }
    if (!loaded) {
      this.deckLocalY = FALLBACK_DECK_LOCAL_Y;
      this.deckRects = [{ cx: 0, cz: 0, hx: DECK_HALF_X, hz: DECK_HALF_Z }];
      this.buildFallbackHull();
      this.buildPods();
    }
    this.buildWiresAndPads();
    this.buildDeckBody();
    this.reset();
  }

  /**
   * Deck rectangles from the GLB's deck_top_* meshes: each mesh's bbox
   * (model space) becomes one rectangle, mapped into carrier-local
   * coords (subtract the normalisation centre, scale). Returns [] when
   * the model has no deck_top meshes.
   */
  private deckRectsFromModel(
    obj: THREE.Object3D,
    center: THREE.Vector3,
    scale: number,
  ): DeckRect[] {
    const rects: DeckRect[] = [];
    obj.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (!mesh.isMesh || !/deck_top/i.test(mesh.name)) return;
      const g = mesh.geometry;
      if (!g.boundingBox) g.computeBoundingBox();
      const bb = g.boundingBox;
      if (!bb) return;
      const minX = (bb.min.x - center.x) * scale;
      const maxX = (bb.max.x - center.x) * scale;
      const minZ = (bb.min.z - center.z) * scale;
      const maxZ = (bb.max.z - center.z) * scale;
      rects.push({
        cx: (minX + maxX) / 2,
        cz: (minZ + maxZ) / 2,
        hx: (maxX - minX) / 2,
        hz: (maxZ - minZ) / 2,
      });
    });
    return rects;
  }

  /**
   * Rotor pods from the SHIELD GLB's own meshes. The four duct_ring_*
   * meshes define the pod centres; each pod's rotor_disc_*, rotor_hub_*
   * and pod_housing_* meshes are grouped to their nearest ring centre.
   * The disc (and hub) geometry is re-pivoted about its ring centre so
   * `disc.rotation.y` spins the rotor in place — the GLB's meshes are
   * positioned in model space, so rotating them about the model origin
   * would swing them around the carrier instead. Returns false when the
   * model has no duct rings (caller falls back to buildPods()).
   */
  private buildShieldPods(
    obj: THREE.Object3D,
    center: THREE.Vector3,
    scale: number,
  ): boolean {
    const rings: THREE.Mesh[] = [];
    const discs: THREE.Mesh[] = [];
    const hubs: THREE.Mesh[] = [];
    const housings: THREE.Mesh[] = [];
    obj.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (!mesh.isMesh) return;
      if (/^duct_ring/i.test(mesh.name)) rings.push(mesh);
      else if (/^rotor_disc/i.test(mesh.name)) discs.push(mesh);
      else if (/^rotor_hub/i.test(mesh.name)) hubs.push(mesh);
      else if (/^pod_housing/i.test(mesh.name)) housings.push(mesh);
    });
    if (rings.length === 0 || discs.length === 0) return false;
    // Ring centres in model space (the pivot each rotor spins about).
    const ringCentres = rings.map((r) => {
      const g = r.geometry;
      if (!g.boundingBox) g.computeBoundingBox();
      const bb = g.boundingBox!;
      return new THREE.Vector3(
        (bb.min.x + bb.max.x) / 2,
        (bb.min.y + bb.max.y) / 2,
        (bb.min.z + bb.max.z) / 2,
      );
    });
    const nearest = (p: THREE.Vector3): number => {
      let best = 0;
      let bestD = Infinity;
      ringCentres.forEach((c, i) => {
        const d = (p.x - c.x) ** 2 + (p.z - c.z) ** 2;
        if (d < bestD) {
          bestD = d;
          best = i;
        }
      });
      return best;
    };
    const meshCentre = (m: THREE.Mesh): THREE.Vector3 => {
      const g = m.geometry;
      if (!g.boundingBox) g.computeBoundingBox();
      const bb = g.boundingBox!;
      return new THREE.Vector3(
        m.position.x + (bb.min.x + bb.max.x) / 2,
        m.position.y + (bb.min.y + bb.max.y) / 2,
        m.position.z + (bb.min.z + bb.max.z) / 2,
      );
    };
    // Re-pivot a rotor mesh about its ring centre: translate the
    // geometry so the pivot is at the mesh origin, then put the mesh
    // at the pivot (model space, inside `inner`). After this,
    // rotation.y spins the rotor in place.
    const repivot = (m: THREE.Mesh, pivot: THREE.Vector3): void => {
      const local = new THREE.Vector3().copy(pivot).sub(m.position);
      const g = m.geometry;
      g.translate(-local.x, -local.y, -local.z);
      if (g.boundingBox) {
        g.boundingBox.min.sub(local);
        g.boundingBox.max.sub(local);
      }
      m.position.set(pivot.x, pivot.y, pivot.z);
    };
    const pods: Pod[] = ringCentres.map((c) => ({
      hp: 100,
      alive: true,
      group: null as unknown as THREE.Object3D,
      disc: null as unknown as THREE.Mesh,
      meshes: [] as THREE.Object3D[],
      spinAxis: 'y' as const,
    }));
    rings.forEach((r, i) => pods[i]!.meshes.push(r));
    discs.forEach((d) => {
      const i = nearest(meshCentre(d));
      repivot(d, ringCentres[i]!);
      pods[i]!.meshes.push(d);
      pods[i]!.disc = d;
      pods[i]!.group = d;
    });
    hubs.forEach((h) => {
      const i = nearest(meshCentre(h));
      repivot(h, ringCentres[i]!);
      pods[i]!.meshes.push(h);
    });
    housings.forEach((h) => {
      const i = nearest(meshCentre(h));
      pods[i]!.meshes.push(h);
    });
    // A pod without a disc cannot spin — it is not a rotor pod.
    const good = pods.filter((p) => p.disc);
    if (good.length === 0) return false;
    // Pods are ordered fwd-starboard, fwd-port, aft-starboard,
    // aft-port so index 0..3 is stable across loads (weapons / damage
    // credit use the index). The converted model's ring centres are
    // not perfectly symmetric (the fwd pair's z differs by ~0.5 m),
    // so the row is compared with a 10 m tolerance — otherwise the
    // 0.5 m z difference decides the order before the x tiebreak and
    // the fwd pair swaps.
    good.sort((a, b) => {
      const ca = (a.disc as THREE.Mesh).position;
      const cb = (b.disc as THREE.Mesh).position;
      const rowA = Math.round(ca.z / 10);
      const rowB = Math.round(cb.z / 10);
      if (rowB !== rowA) return rowB - rowA;
      return cb.x - ca.x;
    });
    this.podList.push(...good);
    return true;
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
      this.podList.push({ hp: 100, alive: true, group: pod, disc, meshes: [pod], spinAxis: 'z' });
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
   * One box collider per deck rectangle (the SHIELD deck is two: wide
   * stern + narrow bow), each with its TOP exactly at deckLocalY.
   */
  private buildDeckBody(): void {
    // Kinematic POSITION-based (issue 8): the deck is moved by setting
    // its translation each update, and Rapier derives the contact
    // velocity from the displacement — so the body sits EXACTLY at
    // the carrier's logical position. The old velocity-based body got
    // BOTH a translation teleport AND a velocity integration each
    // step (a double-move: up to vel*dt = 0.75 m vertically at a
    // 15 m/s climb), which put the physical deck 0.75 m above the
    // logical deck top the controller's onSurface check reads — the
    // jet on the deck read as 0.75 m airborne, onSurface flipped
    // false, the vertical coupling stopped, and the jet fell off.
    this.deckBody = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(this.pos.x, this.pos.y, this.pos.z),
    );
    for (const r of this.deckRects) {
      // Half-height 5 m (was 0.75 m): the box top stays at the deck
      // surface (deckLocalY) but the box now extends 10 m down into
      // the hull. The jet's collider reaches ~2 m below its centre,
      // so with a 1.5 m-thick box a jet that sinks a metre into the
      // deck during a fast climb (15 m/s terrain-following) had its
      // collider bottom below the box — Rapier then resolved the
      // penetration out the BOTTOM and pushed the jet down through
      // the deck (issue 8). A 10 m box keeps the jet inside it for
      // any plausible sink, so the contact always pushes it up.
      const desc = RAPIER.ColliderDesc.cuboid(r.hx, 5, r.hz)
        .setTranslation(r.cx, this.deckLocalY - 5, r.cz)
        // Friction 0 (was 1.0): same takeoff-killer as the terrain — the
        // controller owns deck physics (traps, deck-velocity transfer), so
        // the deck collider must not eat thrust through Coulomb friction.
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        // Collision events feed the integrator's notifyContact() dispatch.
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
      this.world.createCollider(desc, this.deckBody);
    }
    // Keep a handle to the first collider for the floating-origin /
    // integrator registration (the body carries all of them).
    this.deckCollider = this.deckBody.collider(0);
  }

  /** Carrier-local (x, z) inside any deck rectangle? */
  private deckContains(lx: number, lz: number): boolean {
    return this.deckRects.some(
      (r) => Math.abs(lx - r.cx) < r.hx && Math.abs(lz - r.cz) < r.hz,
    );
  }

  /** v2 startRound carrier reset. With no arguments the carrier returns
   *  to its v2 start position and heading; main.ts passes them explicitly
   *  (same values), so both call patterns work. Re-activates the carrier
   *  (issue 5): a deactivated carrier comes back visible, with its deck
   *  body enabled, when its count slot is chosen again. */
  reset(pos?: THREE.Vector3, yaw?: number): void {
    this.alive = true;
    this.active = true;
    this.group.visible = true;
    if (this.deckBody) this.deckBody.setEnabled(true);
    this.pos.copy(pos ?? CARRIER_START_POS);
    // wpIndex first: the default yaw faces the waypoint this carrier
    // is steering toward (its own loop's startIndex, not waypoint 0).
    this.wpIndex = this.options.startIndex ?? 0;
    this.yaw =
      yaw ??
      Math.atan2(
        this.waypoints[this.wpIndex]!.x - this.pos.x,
        this.waypoints[this.wpIndex]!.z - this.pos.z,
      );
    this.list = 0;
    this.yawRate = 0;
    this.dying = false;
    this.dyingT = 0;
    this.impactDone = false;
    this.wreckT = 0;
    // Cruise velocity along the reset heading — NOT zero. The deck
    // spawn (and velocityAt) runs between reset() and the first
    // update(), and update() recomputes vel from the yaw anyway, so
    // the carrier is at cruise speed from the first frame. Zeroing it
    // here made the deck spawn inherit a 0 m/s velocity while the deck
    // moved off at 40 m/s (issue 3: the jet spawned stationary on a
    // moving deck and fell off the stern).
    this.vel.set(
      Math.sin(this.yaw) * this.speed,
      0,
      Math.cos(this.yaw) * this.speed,
    );
    for (const pod of this.podList) {
      pod.hp = 100;
      pod.alive = true;
      for (const m of pod.meshes) m.visible = true;
    }
    this.trapTimers = new WeakMap();
    this.vtolCounted = new WeakSet();
    this.group.position.copy(this.pos);
    this.group.rotation.set(0, this.yaw, 0);
    this.syncDeckBody();
  }

  /** Take this carrier fully out of the game (issue 5 — its count
   *  slot is not chosen this round): not updated, deck reads as
   *  absent, group hidden and deck body disabled so no jet can land
   *  on an invisible deck. reset() re-activates it. */
  deactivate(): void {
    this.active = false;
    this.group.visible = false;
    if (this.deckBody) this.deckBody.setEnabled(false);
  }

  /**
   * v2 carrierUpdate, verbatim behaviour. Call BEFORE world.step() each
   * frame. Waypoint steering (advance within 120 m, yaw rate clamped to
   * +/-0.12 rad/s), 40 m/s cruise, pod lift (each lost pod -25% lift, +7 deg
   * list target), slow sink below 60% lift, terrain / total-lift crash.
   */
  /** Max terrain height ahead along the current heading, sampled
   *  every 200 m out to `dist` (issue 8 — terrain following). The
   *  heightAt source is the exact Rapier heightfield surface in the
   *  live game (main.ts), so this is the surface the carrier would
   *  actually hit, not the analytic approximation. */
  private groundAhead(dist: number): number {
    if (!this.options.heightAt) return -Infinity;
    const hx = Math.sin(this.yaw);
    const hz = Math.cos(this.yaw);
    let max = -Infinity;
    for (let d = 200; d <= dist; d += 200) {
      const h = this.options.heightAt(this.pos.x + hx * d, this.pos.z + hz * d);
      if (h > max) max = h;
    }
    return max;
  }

  update(dt: number): void {
    if (!this.active || !this.alive) return;
    // Dying (issue 9) runs FIRST, before waypoint steering and
    // terrain-following: a dying carrier must sink, not steer or
    // climb. (Terrain-following above the dying branch lifted the
    // wreck back up to cruise altitude after impact.)
    const lift0 = this.podList.filter((p) => p.alive).length / 4;
    if (lift0 <= 0.01 && !this.dying) {
      this.dying = true;
      this.dyingT = 0;
    }
    if (this.dying) {
      this.updateDying(dt, lift0);
      return;
    }
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
    // Terrain following (issue 8): the loop crosses the Himalaya-style
    // ranges (peaks 750 m) but the carrier cruises at y=350 — on the
    // 6 km map the old code flew straight into a range and died
    // (pos.y < groundY + 10, groundY analytic, up to 750 m). Look
    // 2,500 m ahead (62 s at 40 m/s) and target max(350, maxAhead +
    // 150). Climb is rate-limited to 15 m/s up, 10 m/s down: a
    // 25 m/s climb cleared the range but the fast vertical motion
    // made deck-riding fragile (a jet on the deck overshot the
    // surface and the onSurface gate flipped), so the climb is
    // gentle and the long look-ahead gives it time — 62 s at 15 m/s
    // is 930 m of climb, more than the 550 m the range needs. The
    // 150 m clearance is the margin: even if the climb lags, the
    // death check (ground + 10) is far below.
    const ahead = this.groundAhead(2500);
    const targetY = Math.max(CARRIER_ALT, ahead + 150);
    const dyAlt = clamp(targetY - this.pos.y, -10 * dt, 15 * dt);
    this.pos.y += dyAlt;
    // Record the climb rate in vel.y (issue 8 regression fix): the
    // deck body is kinematic, and syncDeckBody sets its linvel from
    // vel — with vel.y = 0 the deck teleported up through a jet
    // standing on it (the contact solver never pushed the jet up, so
    // it was left 29 m below the deck, fell, and crashed "hard
    // landing"). With vel.y = the climb rate, Rapier pushes the jet
    // up with the deck, and velocityAt / the VTOL and trap relative-
    // velocity checks see the matching vertical speed.
    this.vel.y = dt > 0 ? dyAlt / dt : 0;
    // Lift from pods: each lost pod -25% lift, +7 deg list.
    // (The dying check lives at the top of update() — a dying carrier
    // never reaches here.)
    const lift = this.podList.filter((p) => p.alive).length / 4;
    this.list = lerp(this.list, (1 - lift) * 0.12, dt * 0.5);
    if (lift < 0.6) this.pos.y -= (0.6 - lift) * 6 * dt; // sinks slowly
    const groundY = this.options.heightAt ? this.options.heightAt(this.pos.x, this.pos.z) : -Infinity;
    // Terrain death (issue-8 safety net — terrain-following above
    // should prevent it). An instant death is kept here: a carrier
    // that somehow hits terrain was never shot down, so there is no
    // dying sequence to play.
    if (this.pos.y < groundY + 10) {
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
    for (const pod of this.podList) pod.disc.rotation[pod.spinAxis] += dt * 22;
    this.syncDeckBody();
  }

  /**
   * Dying sequence (issue 9): all pods dead. The carrier stops
   * waypoint-following (drifts straight ahead, yaw rate decays to
   * 0), lists heavily toward ~28 deg, and sinks at an increasing
   * rate (2 m/s growing to 25 m/s). Terrain-following is overridden
   * — the carrier must sink even though the climb would try to lift
   * it. On ground contact onCarrierImpact fires and the group is
   * left as a tilted wreck; ~10 s of wreck-smoking later alive=false
   * and onCarrierDown fires. Never a silent vanish or an intact fall.
   */
  private updateDying(dt: number, _lift: number): void {
    this.dyingT += dt;
    if (this.impactDone) {
      // Wreck on the ground: hold the tilted wreck in place for ~10 s
      // of smoking, then write the carrier off (alive=false,
      // onCarrierDown). The group stays visible as wreckage — update()
      // returns early once !alive, so the group freezes at the wreck
      // pose. No more sinking or drifting once it has hit the ground.
      this.wreckT += dt;
      if (this.wreckT >= 10) {
        this.alive = false;
        this.options.onCarrierDown?.();
      }
      this.group.position.copy(this.pos);
      this.group.rotation.set(0, this.yaw, this.list);
      this.syncDeckBody();
      return;
    }
    // Stop turning: yaw rate decays to 0, the carrier drifts
    // straight ahead on its last heading.
    this.yawRate = lerp(this.yawRate, 0, dt * 0.5);
    this.yaw += this.yawRate * dt;
    this.vel.set(Math.sin(this.yaw) * this.speed, 0, Math.cos(this.yaw) * this.speed);
    this.pos.addScaledVector(this.vel, dt);
    // Heavy list toward ~28 deg (0.5 rad).
    this.list = lerp(this.list, 0.5, dt * 0.4);
    // Sink at an increasing rate: 2 m/s + 2 m/s per second of dying,
    // capped at 25 m/s. The carrier falls, it does not glide down.
    const sinkRate = Math.min(2 + this.dyingT * 2, 25);
    this.pos.y -= sinkRate * dt;
    this.vel.y = -sinkRate;
    const groundY = this.options.heightAt
      ? this.options.heightAt(this.pos.x, this.pos.z)
      : -Infinity;
    if (this.pos.y <= groundY + 20) {
      // Ground impact: rest the wreck on the ground (hull sunk 5 m
      // into the terrain so it reads as beached, not floating), heavy
      // list, stop the sink. The impact explosion / debris / scorch
      // are spawned by the game manager via onCarrierImpact.
      this.impactDone = true;
      this.wreckT = 0;
      this.pos.y = groundY + 5;
      this.list = 0.6;
      this.vel.y = 0;
      this.options.onCarrierImpact?.(this.pos.clone());
    }
    this.group.position.copy(this.pos);
    this.group.rotation.set(0, this.yaw, this.list);
    for (const pod of this.podList) pod.disc.rotation[pod.spinAxis] += dt * 22;
    this.syncDeckBody();
  }

  private syncDeckBody(): void {
    if (!this.deckBody) return;
    // Position-based: translation + rotation only. Rapier derives the
    // body's velocity from the displacement for contact resolution,
    // so a jet standing on the deck is carried up/down/around with
    // it. Do NOT set linvel/angvel here — for a position-based body
    // they are ignored, and setting them alongside the translation
    // was the double-move that put the physical deck ahead of the
    // logical position.
    this.deckBody.setTranslation({ x: this.pos.x, y: this.pos.y, z: this.pos.z }, true);
    const q = this.group.quaternion;
    this.deckBody.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
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
    if (!this.active || !this.alive) return -Infinity;
    // After impact the wreck deck is not landable (issue 9), even
    // though alive stays true while the wreck smokes.
    if (this.impactDone) return -Infinity;
    const l = this.carrierLocal(new THREE.Vector3(x, 0, z));
    if (this.deckContains(l.x, l.z)) return this.pos.y + this.deckLocalY;
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
    if (!this.deckContains(l.x, l.z)) return null;
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
      for (const m of pod.meshes) m.visible = false;
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
