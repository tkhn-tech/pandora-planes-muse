// TerrainStreamer.ts — pass 11: INFINITE procedurally-generating terrain.
//
// The old world is ONE 12x12 km heightfield (PandoraWorldGenerator):
// past ±6000 m there is no mesh, no Rapier collider, and the ground
// query (heightfieldHeightAt) freezes at the edge column — measured by
// the pass-11 diagnosis: a Rapier raycast down returns NO hit past the
// edge, the frozen query sits up to 151 m off the analytic continuation,
// and the rim mountain wall (peaks to 750 m) blocks straight low
// flight, while a flight that clears the rim exits into a terrain-less
// void. The same diagnosis measured the floating-origin shift itself as
// invisible for the jet and the terrain (original-frame position jump
// 0.000 m at a shift, max per-step delta exactly v*dt) — the visible
// "glitch" at each shift is the CHASE CAMERA, which is never
// origin-registered and lerps back over ~0.6 s (see the wiring notes at
// the bottom of this file); the streamer does not own the camera and
// cannot fix that, it only has to not add a second glitch.
//
// This module streams the world in chunks around the player so there is
// no edge to hit and no visible snap:
//
//  - Chunk grid: CHUNK_SIZE 3000 m, 50 segments at the SAME 60 m grid
//    spacing as the old 201x201 map (51x51 samples, edges shared with
//    neighbours). Chunks are addressed directly in original-frame
//    coordinates — chunk (cx, cz) spans [cx*3000, (cx+1)*3000] — so
//    every (cx, cz) in Z^2 is valid and the map is unbounded.
//    Generation is deterministic (heightAt is a pure function of x, z),
//    so a retired chunk regenerates identically when re-entered.
//  - Coexistence with the generator (no edits to it or to main.ts were
//    in scope): the central 4x4 chunk block (cx, cz in [-2, 1]) is the
//    old 12 km map and stays owned by PandoraWorldGenerator — its mesh,
//    heightfield collider and runway boxes are untouched. The streamer
//    creates geometry + colliders ONLY for chunks outside that block,
//    and chunk boundaries sit exactly on the old map edge (±6000 =
//    2*3000), so streamer chunks tile up to the old mesh with no
//    overlap (no double-contact) and no gap (shared edge nodes sample
//    the same heightAt at the same 60 m nodes).
//  - Height queries: heightAt(sceneX, sceneZ) answers from the
//    streamer's own chunk heights EVERYWHERE, including the central
//    block (same nodes + same triangle-diagonal rule as
//    heightfieldHeightAt, so it agrees with the generator's collider
//    there — verified by probe-infinite-flight to < 0.05 m). The
//    integrator swaps ONE function (heightfieldGroundHeight) to this
//    and every consumer (controller, carrier, weapons, VFX, camera)
//    follows.
//  - Origin shifts: every chunk mesh is registered with the
//    FloatingOrigin as an object and every chunk body as a body, exactly
//    like the generator's terrain mesh/body, so a shift moves them with
//    the world in the same frame. heightAt() reads the origin's
//    cumulativeShift live (no second accumulator — the pass-9
//    double-count bug class), so queries stay correct across shifts.
//  - Streaming: update(playerScenePos) once per frame keeps a
//    (2*radius+1)^2 window of chunks around the player's chunk
//    (radius 2 = 15x15 km, 25 chunks, ~125k tris). The player's own
//    chunk is created synchronously; the rest fill at CREATE_BUDGET
//    chunks per frame (a 3000 m chunk takes ~15 s to cross at 200 m/s,
//    so the window always stays ahead). Chunks leaving the window lose
//    mesh + collider immediately; their heights stay cached until they
//    are farther than radius+3 chunks away, then the record is dropped
//    (regeneration is deterministic).
//
// MAIN.TS WIRING (parent — this module does not touch main.ts):
//   1. Construct once, next to worldGen (after `await worldGen.build()`):
//        const terrainStreamer = new TerrainStreamer(scene, rapierWorld, floatingOrigin);
//        // register chunk colliders as terrain for contact dispatch:
//        // EITHER poll each frame: for (const h of terrainStreamer.terrainColliderHandles()) colliderOwners.set(h, 'terrain');
//        // OR pass callbacks in the constructor opts:
//        //   new TerrainStreamer(scene, rapierWorld, floatingOrigin, {
//        //     onColliderAdded: (h) => colliderOwners.set(h, 'terrain'),
//        //     onColliderRemoved: (h) => colliderOwners.delete(h),
//        //   });
//   2. Per frame in animate(), immediately AFTER worldGen.shiftOriginIfNeeded(...)
//      (same place, same frame — the shift and the streaming must see the
//      same player position and cumulativeShift):
//        terrainStreamer.update(player?.pos ?? new THREE.Vector3());
//   3. Replace the ground-height function (the ONE swap — every consumer
//      reads through it):
//        const heightfieldGroundHeight = (x: number, z: number): number =>
//          terrainStreamer.heightAt(x, z);
//      (was: worldGen.heightfieldHeightAt(x, z).) Keep worldGen built —
//      it still owns the central-block mesh/collider, the runway boxes,
//      islands, flora and clouds. Do NOT delete the generator.
//   4. THE CAMERA GLITCH (diagnosis finding A — NOT fixed by this
//      module): after the shift fires in animate(), the chase camera is
//      still ~2000 m from the jet for ~0.6 s because updateCamera lerps
//      and the camera is never origin-shifted. Fix in main.ts, one line,
//      right after the shiftOriginIfNeeded call:
//        const shift = worldGen.shiftOriginIfNeeded(player?.pos ?? new THREE.Vector3());
//        if (shift) { camera.position.x -= shift.x; camera.position.z -= shift.z; }
//      (equivalently: floatingOrigin.registerObject(camera) once at boot.
//      Cockpit / free-orbit modes position the camera absolutely, so
//      they never glitched — only chase (and spectate, same lerp) did.)
//   5. KNOWN REMAINING GAP (not this module's file): WeaponSystem
//      missiles/bombs are not registered with the FloatingOrigin either
//      (no origin references in WeaponSystem.ts) — in-flight ordnance
//      jumps ~2 km relative to the world at a shift. Register their
//      bodies/meshes the same way, or accept the transient jump.

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import {
  WORLD_SIZE,
  GRID_STEP,
  generateChunkHeights,
  chunkHeightfieldHeightAt,
  fbm,
  plateauWeight,
  runwayBlend,
} from './PandoraTerrainWorker';
import type { FloatingOrigin } from './FloatingOrigin';

/** Chunk edge length (m). 3000 divides the old 12000 m map exactly, so
 *  chunk boundaries land on the old map edge and on its 60 m grid. */
export const STREAM_CHUNK_SIZE = 3000;
/** Segments per chunk side — 3000/50 = 60 m spacing, the SAME grid as
 *  the old 201x201 map, so chunk nodes coincide with old-grid nodes. */
export const STREAM_CHUNK_SEGMENTS = 50;
/** Samples per chunk side (segments + 1, edges shared with neighbours). */
export const STREAM_CHUNK_RESOLUTION = STREAM_CHUNK_SEGMENTS + 1;
/** Chebyshev radius (chunks) of the loaded window around the player's
 *  chunk: 5x5 = 25 chunks = 15x15 km of terrain, edge ~7.5 km away
 *  (~66% fogged at the clear-weather fog far of 11 km). */
export const STREAM_RADIUS = 2;
/** Max chunks given mesh + collider per update() call (the player's own
 *  chunk is always created synchronously on top of this). At 200 m/s a
 *  chunk takes 15 s to cross, so the window fills far ahead of the jet. */
export const STREAM_CREATE_BUDGET = 4;
/** Chebyshev distance beyond which a chunk's cached heights are dropped
 *  entirely (geometry is already retired past STREAM_RADIUS). */
const EVICT_RADIUS = STREAM_RADIUS + 3;

/** Central-block chunk range: the old 12 km map is chunks cx, cz in
 *  [-2, 1] (chunk -2 spans [-6000,-3000], chunk 1 spans [3000,6000]).
 *  The generator owns those chunks' mesh + collider; the streamer never
 *  creates geometry for them (but still answers heightAt there from its
 *  own identically-sampled heights). Derived from WORLD_SIZE so it
 *  cannot drift from the map size. */
const CENTRAL_MIN = -WORLD_SIZE / 2 / STREAM_CHUNK_SIZE;
const CENTRAL_MAX = WORLD_SIZE / 2 / STREAM_CHUNK_SIZE - 1;

export interface TerrainStreamerOptions {
  /** Window radius in chunks (default STREAM_RADIUS). */
  radius?: number;
  /** Chunks given mesh + collider per update() (default STREAM_CREATE_BUDGET). */
  createBudget?: number;
  /** Called with a chunk collider handle when its collider is created
   *  (main.ts: colliderOwners.set(handle, 'terrain')). */
  onColliderAdded?: (handle: number) => void;
  /** Called with a chunk collider handle when its collider is removed
   *  (main.ts: colliderOwners.delete(handle)). */
  onColliderRemoved?: (handle: number) => void;
}

interface StreamChunk {
  cx: number;
  cz: number;
  /** Row-major heights (resolution^2), x fastest — generated once,
   *  cached until eviction; deterministic, so regeneration is identical. */
  heights: Float32Array;
  mesh: THREE.Mesh | null;
  body: RAPIER.RigidBody | null;
  collider: RAPIER.Collider | null;
}

function clamp01(v: number): number {
  return Math.max(0, Math.min(1, v));
}

export class TerrainStreamer {
  readonly chunkSize = STREAM_CHUNK_SIZE;
  readonly radius: number;
  private readonly createBudget: number;
  private readonly scene: THREE.Scene;
  private readonly world: RAPIER.World;
  private readonly origin: FloatingOrigin | null;
  private readonly onColliderAdded: ((handle: number) => void) | null;
  private readonly onColliderRemoved: ((handle: number) => void) | null;
  private readonly chunks = new Map<string, StreamChunk>();
  /** ONE shared material for every chunk mesh (vertex colours carry the
   *  terrain look, exactly like the generator's single terrain mesh). */
  private readonly material = new THREE.MeshLambertMaterial({ vertexColors: true });
  /** Chunks given mesh + collider so far (lifetime counter, includes
   *  re-creations after retirement — the probe asserts it grows when a
   *  re-entered chunk is rebuilt). */
  private createdCount = 0;
  /** Chunks whose mesh + collider were retired so far. */
  private retiredCount = 0;

  constructor(
    scene: THREE.Scene,
    world: RAPIER.World,
    origin?: FloatingOrigin | null,
    opts: TerrainStreamerOptions = {},
  ) {
    this.scene = scene;
    this.world = world;
    this.origin = origin ?? null;
    this.radius = opts.radius ?? STREAM_RADIUS;
    this.createBudget = opts.createBudget ?? STREAM_CREATE_BUDGET;
    this.onColliderAdded = opts.onColliderAdded ?? null;
    this.onColliderRemoved = opts.onColliderRemoved ?? null;
  }

  // ---- queries ------------------------------------------------------------

  /** Chunk coords containing original-frame (x, z). Math.floor so
   *  negative coords land in the correct chunk (chunk -1 is [-3000, 0)). */
  chunkCoordsAt(origX: number, origZ: number): { cx: number; cz: number } {
    return {
      cx: Math.floor(origX / STREAM_CHUNK_SIZE),
      cz: Math.floor(origZ / STREAM_CHUNK_SIZE),
    };
  }

  /** Exact Rapier-heightfield surface height at SCENE (x, z) — the
   *  surface the colliders actually are, triangle-interpolated with
   *  the same diagonal as heightfieldHeightAt. Scene coords +
   *  cumulativeShift = original frame (read live from the origin, so
   *  no second accumulator and no double-count). Works everywhere,
   *  including inside the central block (same nodes, same rule as the
   *  generator) and arbitrarily far outside it (heights generate on
   *  demand and cache). Never creates mesh/collider — pure query. */
  heightAt(sceneX: number, sceneZ: number): number {
    const shift = this.origin?.cumulativeShift;
    const ox = sceneX + (shift?.x ?? 0);
    const oz = sceneZ + (shift?.z ?? 0);
    const { cx, cz } = this.chunkCoordsAt(ox, oz);
    const rec = this.ensureHeights(cx, cz);
    return chunkHeightfieldHeightAt(
      rec.heights,
      STREAM_CHUNK_RESOLUTION,
      cx * STREAM_CHUNK_SIZE,
      cz * STREAM_CHUNK_SIZE,
      GRID_STEP,
      ox,
      oz,
    );
  }

  /** Cached heights array for the chunk containing original-frame
   *  (x, z), or null if the chunk was never generated (or was evicted
   *  after the player flew away). Does NOT generate — use heightAt()
   *  for a query that must always answer. Probe / debug accessor: lets
   *  a test compare a re-created chunk against a snapshot to prove
   *  deterministic regeneration. */
  heightsAt(origX: number, origZ: number): Float32Array | null {
    const { cx, cz } = this.chunkCoordsAt(origX, origZ);
    return this.chunks.get(key(cx, cz))?.heights ?? null;
  }

  /** True when the chunk containing original-frame (x, z) currently has
   *  a mesh + collider (false for central-block chunks — the generator
   *  owns those — and for chunks outside the loaded window). */
  hasGeometryAt(origX: number, origZ: number): boolean {
    const { cx, cz } = this.chunkCoordsAt(origX, origZ);
    const rec = this.chunks.get(key(cx, cz));
    return rec !== undefined && rec.mesh !== null && rec.body !== null;
  }

  /** Live chunk collider handles (for the integrator's contact
   *  registry — every handle here is terrain). */
  terrainColliderHandles(): number[] {
    const out: number[] = [];
    for (const rec of this.chunks.values()) {
      if (rec.collider) out.push(rec.collider.handle);
    }
    return out;
  }

  /** Number of chunks currently carrying mesh + collider. */
  get loadedGeometryCount(): number {
    let n = 0;
    for (const rec of this.chunks.values()) if (rec.mesh) n++;
    return n;
  }

  /** Number of chunk records cached (with or without geometry). */
  get cachedChunkCount(): number {
    return this.chunks.size;
  }

  get createdChunks(): number {
    return this.createdCount;
  }

  get retiredChunks(): number {
    return this.retiredCount;
  }

  // ---- streaming ----------------------------------------------------------

  /**
   * Per-frame streaming — call once per frame with the player's SCENE
   * position, immediately after the floating-origin shift check (same
   * place main.ts calls worldGen.shiftOriginIfNeeded, so the shift and
   * the streaming see the same position and cumulativeShift). Keeps the
   * (2*radius+1)^2 window around the player's chunk loaded: the
   * player's own chunk is created synchronously if missing, further
   * chunks fill at createBudget per frame (nearest first), chunks past
   * the window lose mesh + collider at once, and records past
   * EVICT_RADIUS are dropped (regeneration is deterministic).
   */
  update(playerScenePos: THREE.Vector3): void {
    const shift = this.origin?.cumulativeShift;
    const ox = playerScenePos.x + (shift?.x ?? 0);
    const oz = playerScenePos.z + (shift?.z ?? 0);
    const { cx: pcx, cz: pcz } = this.chunkCoordsAt(ox, oz);

    // Desired window (Chebyshev radius), minus the central block the
    // generator owns.
    const desired = new Set<string>();
    for (let cz = pcz - this.radius; cz <= pcz + this.radius; cz++) {
      for (let cx = pcx - this.radius; cx <= pcx + this.radius; cx++) {
        if (isCentral(cx, cz)) continue;
        desired.add(key(cx, cz));
      }
    }

    // Player's own chunk first, synchronously — heightAt and the ground
    // under the jet must never have a hole, even on a teleport.
    const playerKey = key(pcx, pcz);
    if (desired.has(playerKey)) this.ensureGeometry(pcx, pcz);

    // Fill the rest of the window, nearest first, within budget.
    const missing: Array<{ cx: number; cz: number; d: number }> = [];
    for (const k of desired) {
      const rec = this.chunks.get(k);
      if (rec && rec.mesh) continue;
      const [cx, cz] = parseKey(k);
      missing.push({ cx, cz, d: Math.max(Math.abs(cx - pcx), Math.abs(cz - pcz)) });
    }
    missing.sort((a, b) => a.d - b.d);
    for (let i = 0; i < Math.min(this.createBudget, missing.length); i++) {
      const m = missing[i]!;
      this.ensureGeometry(m.cx, m.cz);
    }

    // Retire geometry past the window, and drop records past eviction.
    for (const [k, rec] of this.chunks) {
      const d = Math.max(Math.abs(rec.cx - pcx), Math.abs(rec.cz - pcz));
      if (d > this.radius && rec.mesh) this.retireGeometry(rec);
      if (d > EVICT_RADIUS) {
        if (rec.mesh) this.retireGeometry(rec);
        this.chunks.delete(k);
      }
    }
  }

  /** Release everything: all chunk bodies/meshes and the shared material. */
  dispose(): void {
    for (const rec of this.chunks.values()) this.retireGeometry(rec);
    this.chunks.clear();
    this.material.dispose();
  }

  // ---- internals ----------------------------------------------------------

  private ensureHeights(cx: number, cz: number): StreamChunk {
    const k = key(cx, cz);
    let rec = this.chunks.get(k);
    if (!rec) {
      rec = {
        cx,
        cz,
        heights: generateChunkHeights(cx, cz, STREAM_CHUNK_SIZE, STREAM_CHUNK_RESOLUTION),
        mesh: null,
        body: null,
        collider: null,
      };
      this.chunks.set(k, rec);
    }
    return rec;
  }

  /** Give a chunk its mesh + collider (no-op if it already has them, or
   *  if it is a central-block chunk owned by the generator). */
  private ensureGeometry(cx: number, cz: number): void {
    if (isCentral(cx, cz)) return;
    const rec = this.ensureHeights(cx, cz);
    if (rec.mesh && rec.body) return;
    const shift = this.origin?.cumulativeShift;
    const originX = cx * STREAM_CHUNK_SIZE;
    const originZ = cz * STREAM_CHUNK_SIZE;
    // Chunk centre in CURRENT scene coords (origin shift moves it later
    // via the FloatingOrigin registrations below).
    const centreX = originX + STREAM_CHUNK_SIZE / 2 - (shift?.x ?? 0);
    const centreZ = originZ + STREAM_CHUNK_SIZE / 2 - (shift?.z ?? 0);

    const mesh = this.buildMesh(rec, originX, originZ);
    mesh.position.set(centreX, 0, centreZ);
    mesh.name = `terrain-chunk-${cx},${cz}`;
    this.scene.add(mesh);
    this.origin?.registerObject(mesh);

    const body = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed().setTranslation(centreX, 0, centreZ),
    );
    this.origin?.registerBody(body);
    // Column-major copy (index = ix*res + iz) — the same transposed
    // layout PandoraWorldGenerator uses for its heightfield.
    const res = STREAM_CHUNK_RESOLUTION;
    const hfHeights = new Float32Array(rec.heights.length);
    for (let iz = 0; iz < res; iz++) {
      for (let ix = 0; ix < res; ix++) {
        hfHeights[ix * res + iz] = rec.heights[iz * res + ix]!;
      }
    }
    const desc = RAPIER.ColliderDesc.heightfield(
      STREAM_CHUNK_SEGMENTS,
      STREAM_CHUNK_SEGMENTS,
      hfHeights,
      new RAPIER.Vector3(STREAM_CHUNK_SIZE, 1, STREAM_CHUNK_SIZE),
    )
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
    const collider = this.world.createCollider(desc, body);

    rec.mesh = mesh;
    rec.body = body;
    rec.collider = collider;
    this.createdCount++;
    this.onColliderAdded?.(collider.handle);
  }

  /** Remove a chunk's mesh + collider (heights stay cached for queries
   *  until eviction). Unregisters from the FloatingOrigin first so a
   *  later shift cannot move a removed body/mesh. */
  private retireGeometry(rec: StreamChunk): void {
    // No-op (and uncounted) when the chunk has no geometry — e.g. the
    // eviction branch calling retire on an already-retired chunk.
    if (!rec.mesh && !rec.body) return;
    if (rec.collider) {
      this.onColliderRemoved?.(rec.collider.handle);
    }
    if (rec.body) {
      this.origin?.unregisterBody(rec.body);
      // Removes the body's colliders with it.
      this.world.removeRigidBody(rec.body);
      rec.body = null;
      rec.collider = null;
    }
    if (rec.mesh) {
      this.origin?.unregisterObject(rec.mesh);
      this.scene.remove(rec.mesh);
      rec.mesh.geometry.dispose();
      rec.mesh = null;
    }
    this.retiredCount++;
  }

  /** Visual mesh for a chunk: PlaneGeometry with the chunk heights and
   *  the generator's vertex-colour recipe (fbm green/brown, rock and
   *  snow above the plateaus, dark green low ground, dark runway
   *  strips) evaluated at ORIGINAL-frame (x, z), so chunk colours agree
   *  with the old mesh where they meet. Vertex order after
   *  rotateX(-PI/2) is row-major, x fastest, z increasing — exactly the
   *  layout generateChunkHeights produces, so heights[i] IS vertex i. */
  private buildMesh(rec: StreamChunk, originX: number, originZ: number): THREE.Mesh {
    const SEG = STREAM_CHUNK_SEGMENTS;
    const geo = new THREE.PlaneGeometry(STREAM_CHUNK_SIZE, STREAM_CHUNK_SIZE, SEG, SEG);
    geo.rotateX(-Math.PI / 2);
    const pos = geo.attributes.position as THREE.BufferAttribute;
    const col = new Float32Array(pos.count * 3);
    const cA = new THREE.Color(0x3d7a44);
    const cB = new THREE.Color(0x8a7a4a);
    const cC = new THREE.Color(0x5a6a72);
    const cD = new THREE.Color(0x2a4a34);
    const cSnow = new THREE.Color(0xe8eef2);
    const tmp = new THREE.Color();
    for (let i = 0; i < pos.count; i++) {
      // Local vertex coords are chunk-relative (-1500..1500); the colour
      // recipe needs ORIGINAL-frame coords.
      const x = originX + (pos.getX(i) + STREAM_CHUNK_SIZE / 2);
      const z = originZ + (pos.getZ(i) + STREAM_CHUNK_SIZE / 2);
      const h = rec.heights[i]!;
      pos.setY(i, h);
      const n = fbm(x / 240 + 7, z / 240 - 2);
      tmp.copy(cA).lerp(cB, clamp01(n * 1.4 - 0.3));
      if (plateauWeight(x, z) <= 0.5) {
        if (h > 120) tmp.lerp(cC, clamp01((h - 120) / 120));
        if (h > 550) tmp.lerp(cSnow, clamp01((h - 550) / 150));
      }
      if (h < 5) tmp.lerp(cD, 0.6);
      if (runwayBlend(x, z) > 0.5) tmp.set(0x3a3a3a);
      col[i * 3] = tmp.r;
      col[i * 3 + 1] = tmp.g;
      col[i * 3 + 2] = tmp.b;
    }
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    geo.computeVertexNormals();
    return new THREE.Mesh(geo, this.material);
  }
}

function key(cx: number, cz: number): string {
  return `${cx},${cz}`;
}

function parseKey(k: string): [number, number] {
  const [a, b] = k.split(',');
  return [Number(a), Number(b)];
}

/** True for chunks of the old 12 km map (owned by PandoraWorldGenerator). */
function isCentral(cx: number, cz: number): boolean {
  return cx >= CENTRAL_MIN && cx <= CENTRAL_MAX && cz >= CENTRAL_MIN && cz <= CENTRAL_MAX;
}
