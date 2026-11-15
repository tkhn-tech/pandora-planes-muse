// PandoraWorldGenerator.ts — G4 world: terrain, floating islands, flora,
// clouds, sky. Ports pandora-template.html lines ~498-749 (Three setup,
// seeded noise, runways, terrain mesh, runway strips, floating islands,
// flora, clouds/sky, floating origin) and the night-glow half of updateSky
// (lines ~2036-2056).
//
// The pure height/noise math lives in PandoraTerrainWorker.ts (see its header
// for why) and is re-exported here so the integrator can import heightAt
// from either module. The visual terrain mesh and the Rapier heightfield
// collider are built from the SAME Float32Array, so ground contact is real.
//
// Rapier heightfield layout (verified numerically — see the task report):
// the collider takes a COLUMN-major copy of the heights (index =
// ix*(nsubdivsZ+1)+iz); the visual mesh uses the same values row-major.
// Mesh and collider therefore agree exactly; both approximate the analytic
// heightAt between grid nodes (piecewise-linear over the 40 m grid).

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import {
  WORLD_SIZE,
  WORLD_SEED,
  TERRAIN_SEGMENTS,
  heightAt,
  runwayBlend,
  fbm,
  createV2Rand,
  generateHeights,
  type TerrainChunkRequest,
} from './PandoraTerrainWorker';
import { RUNWAYS, type Runway } from './RunwayPlacer';
import type { FloatingOrigin } from './FloatingOrigin';

export { heightAt, runwayBlend, fbm, WORLD_SIZE, WORLD_SEED } from './PandoraTerrainWorker';
export { RUNWAYS, type Runway } from './RunwayPlacer';
/** Integrator alias: main.ts imports RunwayDef from this module. */
export type RunwayDef = Runway;

const ISLAND_COUNT = 8; // v2
const FLORA_STEMS = 420; // v2 instanced stems + caps
const FLORA_SPIRES = 90; // v2 instanced spires
const CLOUD_COUNT = 26; // v2
const WATERFALL_PARTICLES = 70; // v2

const SKY_DAY = new THREE.Color(0x87b5d9);
const SKY_DUSK = new THREE.Color(0xe8905a);
const SKY_NIGHT = new THREE.Color(0x0a1226);

interface Waterfall {
  geometry: THREE.BufferGeometry;
  vel: Float32Array;
  n: number;
}

export class PandoraWorldGenerator {
  private readonly scene: THREE.Scene;
  private readonly world: RAPIER.World;
  private origin: FloatingOrigin | null = null;

  /** Scene-frame offset accumulator: scene coords + originShift =
   *  original-frame coords (v2 groundH). Kept in sync with the
   *  FloatingOrigin's cumulativeShift via registerWithFloatingOrigin(). */
  private readonly _originShift = new THREE.Vector3();

  private terrainMesh: THREE.Mesh | null = null;
  private terrainBody: RAPIER.RigidBody | null = null;
  private terrainCollider: RAPIER.Collider | null = null;
  private heights: Float32Array | null = null;
  /** Samples per side of the terrain grid (151 = 150 segments + 1). */
  readonly resolution = TERRAIN_SEGMENTS + 1;

  private islands: THREE.Group[] = [];
  private islandBase: THREE.Vector3[] = [];
  private clouds: THREE.Sprite[] = [];
  private cloudBase: THREE.Vector3[] = [];
  private floraMeshes: THREE.InstancedMesh[] = [];
  private _floraMaterials: THREE.MeshLambertMaterial[] = [];
  private falls: Waterfall[] = [];

  private nightFactor = 0;
  private readonly skyTmp = new THREE.Color();
  private built = false;

  constructor(scene: THREE.Scene, world: RAPIER.World, origin?: FloatingOrigin) {
    this.scene = scene;
    this.world = world;
    if (origin) this.registerWithFloatingOrigin(origin);
  }

  /**
   * Build the whole world. Spawns the terrain worker to generate the 6x6 km
   * heights (seed defaults to the v2 seed 1337); if the worker fails or
   * times out after 10 s, heights are generated synchronously on the main
   * thread from the same pure function, so build() never fails on the
   * worker. Scatter (islands/flora/clouds) consumes ONE Park-Miller
   * sequence in v2 order (islands -> flora -> clouds), reproducing the v2
   * positions.
   */
  async build(seed: number = WORLD_SEED): Promise<void> {
    this.heights = await this.loadHeights(seed);
    this.buildTerrainMesh();
    this.buildHeightfieldCollider();
    // ONE Park-Miller sequence for all scatter, consumed in v2 order
    // (islands -> flora -> clouds) — this is what puts the islands, flora
    // and clouds at the v2 positions.
    const rand = createV2Rand(seed);
    this.buildIslands(rand);
    this.buildFlora(rand);
    this.buildClouds(rand);
    this.scene.background = new THREE.Color(0x87b5d9);
    this.scene.fog = new THREE.Fog(0x9db8cc, 900, 5200);
    this.built = true;
  }

  // ---- heights ------------------------------------------------------------

  private loadHeights(seed: number): Promise<Float32Array> {
    const req: TerrainChunkRequest = {
      seed,
      chunkX: 0,
      chunkZ: 0,
      size: WORLD_SIZE,
      resolution: this.resolution,
    };
    return new Promise((resolve) => {
      let done = false;
      const finish = (h: Float32Array) => {
        if (!done) {
          done = true;
          resolve(h);
        }
      };
      try {
        const worker = new Worker(new URL('./PandoraTerrainWorker.ts', import.meta.url), {
          type: 'module',
        });
        const timer = setTimeout(() => {
          worker.terminate();
          finish(generateHeights(req));
        }, 10000);
        worker.onmessage = (event: MessageEvent<{ heights: Float32Array }>) => {
          clearTimeout(timer);
          worker.terminate();
          finish(event.data.heights);
        };
        worker.onerror = () => {
          clearTimeout(timer);
          worker.terminate();
          finish(generateHeights(req));
        };
        worker.postMessage(req);
      } catch {
        finish(generateHeights(req));
      }
    });
  }

  // ---- terrain mesh + heightfield (SAME heights array) --------------------

  private buildTerrainMesh(): void {
    const heights = this.heights;
    if (!heights) throw new Error('PandoraWorldGenerator: heights not loaded');
    const SEG = TERRAIN_SEGMENTS;
    const geo = new THREE.PlaneGeometry(WORLD_SIZE, WORLD_SIZE, SEG, SEG);
    geo.rotateX(-Math.PI / 2);
    const pos = geo.attributes.position as THREE.BufferAttribute;
    // PlaneGeometry vertex order after rotateX(-PI/2) is row-major with x
    // fastest and z running -3000 -> +3000 down the rows — exactly the
    // layout generateHeights() produces, so heights[i] IS vertex i's height.
    const col = new Float32Array(pos.count * 3);
    const cA = new THREE.Color(0x3d7a44);
    const cB = new THREE.Color(0x8a7a4a);
    const cC = new THREE.Color(0x5a6a72);
    const cD = new THREE.Color(0x2a4a34);
    const tmp = new THREE.Color();
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i);
      const z = pos.getZ(i);
      const h = heights[i];
      pos.setY(i, h);
      const n = fbm(x / 240 + 7, z / 240 - 2);
      tmp.copy(cA).lerp(cB, clamp01(n * 1.4 - 0.3));
      if (h > 120) tmp.lerp(cC, clamp01((h - 120) / 120));
      if (h < 5) tmp.lerp(cD, 0.6);
      if (runwayBlend(x, z) > 0.5) tmp.set(0x3a3a3a);
      col[i * 3] = tmp.r;
      col[i * 3 + 1] = tmp.g;
      col[i * 3 + 2] = tmp.b;
    }
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    geo.computeVertexNormals(); // v2: smooth vertex normals, no flatShading flag
    this.terrainMesh = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ vertexColors: true }));
    this.terrainMesh.name = 'terrain';
    this.scene.add(this.terrainMesh);
  }

  private buildHeightfieldCollider(): void {
    const heights = this.heights;
    if (!heights) throw new Error('PandoraWorldGenerator: heights not loaded');
    // Fixed body at the origin; the collider is centred on it, exactly like
    // the visual mesh. scale.y = 1 so the heights array values ARE world Y.
    this.terrainBody = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
    );
    // LAYOUT (verified numerically — see the task report): Rapier's
    // heightfield heights are COLUMN-major (ColliderDesc.heightfield docs),
    // index = ix * (nsubdivsZ + 1) + iz, where ix is the x-index and iz the
    // z-index; sample (ix, iz) sits at x = -3000 + ix*40, z = -3000 + iz*40.
    // The visual mesh array is ROW-major (index = iz*151 + ix), so the
    // collider gets a transposed copy of the SAME values. Row-major passed
    // straight in is wrong by up to ~190 m; transposed, raycasts at grid
    // nodes agree with heightAt to ~3e-4 m (float32). Between nodes the
    // heightfield is piecewise-linear over the 40 m grid, so it deviates
    // from the analytic heightAt by up to ~13 m at the worst spots
    // (plateau edges) — exactly like the visual mesh, which shares the
    // same grid, so mesh and collider always agree with each other.
    const res = this.resolution;
    const hfHeights = new Float32Array(heights.length);
    for (let iz = 0; iz < res; iz++) {
      for (let ix = 0; ix < res; ix++) {
        hfHeights[ix * res + iz] = heights[iz * res + ix];
      }
    }
    const desc = RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS,
      TERRAIN_SEGMENTS,
      hfHeights,
      new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    )
      .setFriction(0.9)
      .setRestitution(0)
      // Collision events feed the integrator's notifyContact() dispatch.
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
    this.terrainCollider = this.world.createCollider(desc, this.terrainBody);
  }

  // ---- floating islands (v2 positions) ------------------------------------

  private buildIslands(rand: () => number): void {
    const rockMat = new THREE.MeshLambertMaterial({ color: 0x6a5a4a });
    const topMat = new THREE.MeshLambertMaterial({ color: 0x3d7a44 });
    const rootMat = new THREE.MeshLambertMaterial({ color: 0x4a3a2a });
    for (let i = 0; i < ISLAND_COUNT; i++) {
      const x = (rand() - 0.5) * 4800;
      const z = (rand() - 0.5) * 4800;
      const y = 260 + rand() * 260;
      const R = 60 + rand() * 90;
      const g = new THREE.Group();
      g.name = `island-${i}`;
      const cone = new THREE.Mesh(new THREE.ConeGeometry(R, R * 1.7, 7), rockMat);
      cone.rotation.x = Math.PI;
      cone.position.y = -R * 0.85;
      g.add(cone);
      const top = new THREE.Mesh(new THREE.CylinderGeometry(R, R * 0.85, 14, 8), topMat);
      top.position.y = 4;
      g.add(top);
      for (let k = 0; k < 4; k++) {
        const root = new THREE.Mesh(new THREE.ConeGeometry(3 + rand() * 4, 30 + rand() * 40, 5), rootMat);
        root.rotation.x = Math.PI;
        root.rotation.z = (rand() - 0.5) * 0.5;
        root.position.set((rand() - 0.5) * R, -R * 0.9, (rand() - 0.5) * R);
        g.add(root);
      }
      // Waterfall: Points stream falling from one edge, recycled.
      // v2 quirk, copied exactly: the local points already include edge.x /
      // edge.z AND the Points object is positioned at `edge`, so the stream
      // sits ~1.4R out from the island centre.
      const N = WATERFALL_PARTICLES;
      const p = new Float32Array(N * 3);
      const v = new Float32Array(N);
      const edge = new THREE.Vector3(R * 0.7, 0, 0);
      for (let k = 0; k < N; k++) {
        p[k * 3] = edge.x + (rand() - 0.5) * 6;
        p[k * 3 + 1] = -rand() * 160;
        p[k * 3 + 2] = edge.z + (rand() - 0.5) * 6;
        v[k] = 40 + rand() * 30;
      }
      const fg = new THREE.BufferGeometry();
      fg.setAttribute('position', new THREE.BufferAttribute(p, 3));
      const fall = new THREE.Points(
        fg,
        new THREE.PointsMaterial({ color: 0x9fd4ff, size: 3.2, transparent: true, opacity: 0.75 }),
      );
      fall.position.copy(edge);
      g.add(fall);
      this.falls.push({ geometry: fg, vel: v, n: N });

      g.position.set(x, y, z);
      this.islandBase.push(new THREE.Vector3(x, y, z));
      this.scene.add(g);
      this.islands.push(g);
    }
  }

  // ---- bioluminescent flora ----------------------------------------------

  private buildFlora(rand: () => number): void {
    const stemMat = new THREE.MeshLambertMaterial({ color: 0x8a7a9a, emissive: 0x2a1a4a, emissiveIntensity: 0.2 });
    const capMat = new THREE.MeshLambertMaterial({ color: 0x6a4a8a, emissive: 0x7a3aff, emissiveIntensity: 0.25 });
    const spireMat = new THREE.MeshLambertMaterial({ color: 0x2a6a5a, emissive: 0x2affc8, emissiveIntensity: 0.25 });
    this._floraMaterials.push(stemMat, capMat, spireMat);

    const stemGeo = new THREE.CylinderGeometry(0.7, 1.1, 7, 6);
    const capGeo = new THREE.SphereGeometry(3.2, 8, 6, 0, Math.PI * 2, 0, Math.PI / 2);
    const spireGeo = new THREE.ConeGeometry(1.6, 16, 6);
    const stems = new THREE.InstancedMesh(stemGeo, stemMat, FLORA_STEMS);
    const caps = new THREE.InstancedMesh(capGeo, capMat, FLORA_STEMS);
    const spires = new THREE.InstancedMesh(spireGeo, spireMat, FLORA_SPIRES);
    const m4 = new THREE.Matrix4();
    const q = new THREE.Quaternion();
    const sc = new THREE.Vector3();
    const pv = new THREE.Vector3();
    let si = 0;
    let pi = 0;
    for (let k = 0; k < 900 && si < FLORA_STEMS; k++) {
      const x = (rand() - 0.5) * 5600;
      const z = (rand() - 0.5) * 5600;
      if (runwayBlend(x, z) > 0.1) continue;
      const h = heightAt(x, z);
      if (h > 150) continue;
      const s = 0.7 + rand() * 1.6;
      q.setFromAxisAngle(pv.set(0, 1, 0), rand() * Math.PI * 2);
      m4.compose(new THREE.Vector3(x, h + 3.5 * s, z), q, sc.set(s, s, s));
      stems.setMatrixAt(si, m4);
      caps.setMatrixAt(si, m4);
      si++;
    }
    for (let k = 0; k < 200 && pi < FLORA_SPIRES; k++) {
      const x = (rand() - 0.5) * 5600;
      const z = (rand() - 0.5) * 5600;
      if (runwayBlend(x, z) > 0.1) continue;
      const h = heightAt(x, z);
      if (h > 150) continue;
      const s = 0.8 + rand() * 1.8;
      q.setFromAxisAngle(pv.set(0, 1, 0), rand() * Math.PI * 2);
      m4.compose(new THREE.Vector3(x, h + 8 * s, z), q, sc.set(s, s, s));
      spires.setMatrixAt(pi, m4);
      pi++;
    }
    stems.count = si;
    caps.count = si;
    spires.count = pi;
    this.floraMeshes.push(stems, caps, spires);
    this.scene.add(stems, caps, spires);
  }

  // ---- clouds + sky ---------------------------------------------------------

  private buildClouds(rand: () => number): void {
    const tex = makeCloudTexture();
    for (let i = 0; i < CLOUD_COUNT; i++) {
      const s = new THREE.Sprite(
        new THREE.SpriteMaterial({ map: tex, transparent: true, opacity: 0.5 + rand() * 0.3, depthWrite: false }),
      );
      const sc = 260 + rand() * 420;
      s.scale.set(sc, sc * 0.36, 1);
      s.position.set((rand() - 0.5) * 5600, 620 + rand() * 320, (rand() - 0.5) * 5600);
      this.cloudBase.push(s.position.clone());
      this.scene.add(s);
      this.clouds.push(s);
    }
  }

  // ---- queries --------------------------------------------------------------

  /** Pure original-frame ground height — same math as the worker. */
  heightAt(x: number, z: number): number {
    return heightAt(x, z);
  }

  /**
   * Scene-frame ground height (what aircraft / AI must query): scene coords
   * + the cumulative floating-origin shift = original-frame coords (v2
   * groundH). Only correct if this generator is registered with the
   * FloatingOrigin (constructor arg or registerWithFloatingOrigin) — that
   * keeps originShift in sync.
   */
  groundHeight(x: number, z: number): number {
    return heightAt(x + this._originShift.x, z + this._originShift.z);
  }

  /**
   * Night factor 0 (noon) .. 1 (midnight). Drives the flora emissive glow
   * exactly as v2 updateSky does (0.25 + nightF * 2.4) and tints the sky
   * dome / fog through the same day -> dusk -> night ramp. The integrator
   * owns the time-of-day clock; call this once per frame with its nightF.
   */
  setNightFactor(f: number): void {
    this.nightFactor = clamp01(f);
    const nf = this.nightFactor;
    for (const m of this._floraMaterials) m.emissiveIntensity = 0.25 + nf * 2.4;
    const day = 1 - nf;
    if (day > 0.55) this.skyTmp.copy(SKY_DAY);
    else if (day > 0.05) this.skyTmp.copy(SKY_DUSK).lerp(SKY_DAY, (day - 0.05) / 0.5);
    else this.skyTmp.copy(SKY_NIGHT).lerp(SKY_DUSK, day / 0.05);
    if (this.scene.background instanceof THREE.Color) this.scene.background.copy(this.skyTmp);
    else this.scene.background = this.skyTmp.clone();
    if (this.scene.fog) this.scene.fog.color.copy(this.skyTmp);
  }

  getNightFactor(): number {
    return this.nightFactor;
  }

  /** The three flora materials (stem, cap, spire) — for the integrator to
   *  inspect or drive directly. setNightFactor() is the normal path. */
  getFloraMaterial(): THREE.Material[] {
    return [...this._floraMaterials];
  }

  /** v2 waterfall update: points fall at their velocity, recycle at -170 m. */
  updateWaterfalls(dt: number): void {
    for (const f of this.falls) {
      const a = f.geometry.attributes.position as THREE.BufferAttribute;
      for (let i = 0; i < f.n; i++) {
        let y = a.getY(i) - f.vel[i] * dt;
        if (y < -170) y = 0;
        a.setY(i, y);
      }
      a.needsUpdate = true;
    }
  }

  /** v2 cloud drift: +windX * dt * 0.4, wrapping past x = 3000 to -3000. */
  updateClouds(dt: number, windX: number): void {
    for (const c of this.clouds) {
      c.position.x += windX * dt * 0.4;
      if (c.position.x > 3000) c.position.x = -3000;
    }
  }

  /**
   * Per-frame world animation (v2 main loop): waterfalls fall + recycle,
   * clouds drift with the wind. `wind` is the wind vector (its x component
   * drives the drift) or a bare wind-x speed; omit it for still air.
   * The integrator calls this once per frame.
   */
  update(dt: number, wind?: THREE.Vector3 | number): void {
    this.updateWaterfalls(dt);
    const windX = typeof wind === 'number' ? wind : (wind?.x ?? 0);
    this.updateClouds(dt, windX);
  }

  /**
   * Floating-origin check for the main loop (v2 shiftOrigin): when the
   * focus position is past the 2000 m threshold, shift the world back
   * toward the origin and return the shift vector; otherwise return null.
   * Delegates to the registered FloatingOrigin when one exists, so the
   * shift is applied exactly once.
   */
  shiftOriginIfNeeded(playerPos: THREE.Vector3): THREE.Vector3 | null {
    if (this.origin) {
      const shift = this.origin.shiftIfNeeded(playerPos);
      if (shift) this.origin.applyShift(shift);
      return shift;
    }
    if (Math.hypot(playerPos.x, playerPos.z) <= 2000) return null;
    const shift = new THREE.Vector3(playerPos.x, 0, playerPos.z);
    this.applyOriginShift(shift);
    return shift;
  }

  /**
   * Restore the original frame (v2 resetOrigin): every world mesh back to
   * its base position and the origin shift back to zero. Delegates to the
   * registered FloatingOrigin when one exists.
   */
  resetOrigin(): void {
    if (this.origin) {
      this.origin.reset();
      return;
    }
    if (this._originShift.x === 0 && this._originShift.z === 0) return;
    const shift = new THREE.Vector3(this._originShift.x, 0, this._originShift.z);
    this.applyOriginShift(shift);
    // applyOriginShift moved everything by -shift and bumped the
    // accumulator by +shift; undo the bump — the manual path has no
    // FloatingOrigin to keep it in sync, so zero it directly.
    this._originShift.set(0, 0, 0);
    if (this.terrainMesh) this.terrainMesh.position.set(0, 0, 0);
    this.islands.forEach((g, i) => g.position.copy(this.islandBase[i]));
    this.clouds.forEach((s, i) => s.position.copy(this.cloudBase[i]));
    for (const m of this.floraMeshes) m.position.set(0, 0, 0);
    if (this.terrainBody) this.terrainBody.setTranslation({ x: 0, y: 0, z: 0 }, true);
  }

  // ---- floating origin wiring ----------------------------------------------

  /**
   * Register the world with a FloatingOrigin: the terrain mesh, islands,
   * clouds and flora meshes move with the world, the terrain Rapier body
   * moves with it, and this generator's originShift accumulator stays in
   * sync so groundHeight() keeps working. Do NOT also call
   * applyOriginShift() for the same shift — that would double-move.
   */
  registerWithFloatingOrigin(origin: FloatingOrigin): void {
    this.origin = origin;
    if (this.terrainMesh) origin.registerObject(this.terrainMesh);
    for (const g of this.islands) origin.registerObject(g);
    for (const s of this.clouds) origin.registerObject(s);
    for (const m of this.floraMeshes) origin.registerObject(m);
    if (this.terrainBody) origin.registerBody(this.terrainBody);
    origin.registerCallback((shift) => {
      this._originShift.x += shift.x;
      this._originShift.z += shift.z;
    });
  }

  /**
   * Manual origin shift (no FloatingOrigin wired): moves the world meshes
   * and the terrain body by -shift and bumps the originShift accumulator.
   * Only for tests / standalone use — when a FloatingOrigin is registered,
   * use its applyShift() instead.
   */
  applyOriginShift(shift: THREE.Vector3): void {
    if (shift.x === 0 && shift.z === 0) return;
    const move = (o: THREE.Object3D) => {
      o.position.x -= shift.x;
      o.position.z -= shift.z;
    };
    if (this.terrainMesh) move(this.terrainMesh);
    for (const g of this.islands) move(g);
    for (const s of this.clouds) move(s);
    for (const m of this.floraMeshes) move(m);
    if (this.terrainBody) {
      const t = this.terrainBody.translation();
      this.terrainBody.setTranslation({ x: t.x - shift.x, y: t.y, z: t.z - shift.z }, true);
    }
    this._originShift.x += shift.x;
    this._originShift.z += shift.z;
  }

  // ---- accessors ------------------------------------------------------------

  get isBuilt(): boolean {
    return this.built;
  }

  get terrain(): THREE.Mesh | null {
    return this.terrainMesh;
  }

  /** The runway table (original-frame coords). */
  get runways(): Runway[] {
    return RUNWAYS;
  }

  /** Cumulative floating-origin shift (scene coords + this = original
   *  frame). Read-only view of the internal accumulator. */
  get originShift(): THREE.Vector3 {
    return this._originShift;
  }

  /** The three flora materials (stem, cap, spire). main.ts drives their
   *  emissiveIntensity directly from the time-of-day code; setNightFactor()
   *  is the equivalent call on this class. */
  get floraMaterials(): THREE.MeshLambertMaterial[] {
    return this._floraMaterials;
  }

  /** Fixed Rapier body carrying the heightfield collider — register it with
   *  the FloatingOrigin (done automatically by registerWithFloatingOrigin). */
  /** Heightfield collider handle for the integrator's contact registry. */
  get terrainColliderHandle(): number | null {
    return this.terrainCollider ? this.terrainCollider.handle : null;
  }
  get terrainBodyRef(): RAPIER.RigidBody | null {
    return this.terrainBody;
  }

  get terrainColliderRef(): RAPIER.Collider | null {
    return this.terrainCollider;
  }

  /** The exact heights array the mesh and the collider share. */
  get terrainHeights(): Float32Array | null {
    return this.heights;
  }

  get islandGroups(): THREE.Group[] {
    return this.islands;
  }

  get islandBasePositions(): THREE.Vector3[] {
    return this.islandBase;
  }

  get cloudSprites(): THREE.Sprite[] {
    return this.clouds;
  }

  get cloudBasePositions(): THREE.Vector3[] {
    return this.cloudBase;
  }

  get flora(): THREE.InstancedMesh[] {
    return this.floraMeshes;
  }
}

function clamp01(v: number): number {
  return Math.max(0, Math.min(1, v));
}

function makeCloudTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d');
  if (!g) throw new Error('PandoraWorldGenerator: 2d canvas context unavailable');
  const grd = g.createRadialGradient(64, 64, 8, 64, 64, 62);
  grd.addColorStop(0, 'rgba(255,255,255,0.85)');
  grd.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grd;
  g.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}
