// PandoraTerrainWorker.ts — pure terrain math + the terrain Web Worker.
//
// ARCHITECTURE NOTE (single source of truth, no duplication):
// The v2 height function lives HERE, not in PandoraWorldGenerator.ts, even
// though the task sketch suggested the opposite direction. Reason: this file
// must stay free of THREE / Rapier / DOM (it is bundled into a Web Worker),
// and any module it imports would be bundled into the worker too —
// PandoraWorldGenerator.ts imports three and @dimforge/rapier3d-compat, so
// having the worker import the generator would drag the whole 3D stack into
// the worker bundle. Instead the generator (and RunwayPlacer) import the pure
// math FROM this file; PandoraWorldGenerator.ts re-exports heightAt etc. so
// the integrator-facing API is unchanged. The worker entry point at the
// bottom of this file is guarded (window === undefined) so importing this
// module on the main thread is side-effect free.
//
// TWO DEVIATIONS FROM THE TASK TEXT, BOTH DELIBERATE:
// 1. Noise: v2's height function is HASH-BASED VALUE noise (hash2/vnoise/
//    fbm/ridge below, copied verbatim from pandora-template.html lines
//    513-517), NOT simplex noise. Wiring simplex-noise into heightAt would
//    change every height and break the "match v2 exactly" requirement, which
//    is stated twice and is the verifiable one. mulberry32 is still provided
//    (and exported) as the seedable RNG to pass to simplex-noise's
//    createNoise2D/3D if a future detail layer wants simplex — it is just
//    deliberately NOT part of heightAt.
// 2. Scatter RNG: v2's `rand` is a Park-Miller LCG seeded 1337, and the
//    island/flora/cloud positions in v2 are that exact sequence consumed in
//    island -> flora -> cloud order. createV2Rand() reproduces it so the
//    world generator lands islands/flora/clouds at the v2 positions.
//    mulberry32 (task-requested) is a different sequence and would move them.

export const WORLD_SIZE = 6000; // v2 WORLD: world half-extent is +/-3000 m
export const WORLD_SEED = 1337; // v2 `rand` seed
export const TERRAIN_SEGMENTS = 150; // v2 terrain mesh SEG; -> 151x151 samples

const D2R = Math.PI / 180;

// ---------------------------------------------------------------------------
// Seeded RNGs
// ---------------------------------------------------------------------------

/** Small seedable RNG (mulberry32). Use this to seed simplex-noise's
 *  createNoise2D/3D for any future detail layer — see the header note. */
export function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Exact replica of v2's `rand`: Park-Miller LCG, seed 1337.
 *  `(s => () => (s = (s * 16807) % 2147483647) / 2147483647)(1337)`.
 *  The world generator consumes ONE instance of this in island -> flora ->
 *  cloud order, which reproduces the v2 scatter positions. */
export function createV2Rand(seed: number = WORLD_SEED): () => number {
  let s = seed;
  return () => (s = (s * 16807) % 2147483647) / 2147483647;
}

// ---------------------------------------------------------------------------
// v2 value noise (verbatim from pandora-template.html, lines 513-517)
// ---------------------------------------------------------------------------

function hash2(x: number, y: number): number {
  const s = Math.sin(x * 127.1 + y * 311.7) * 43758.5453;
  return s - Math.floor(s);
}

export function vnoise(x: number, y: number): number {
  const xi = Math.floor(x);
  const yi = Math.floor(y);
  const xf = x - xi;
  const yf = y - yi;
  const u = xf * xf * (3 - 2 * xf);
  const v = yf * yf * (3 - 2 * yf);
  return lerp(lerp(hash2(xi, yi), hash2(xi + 1, yi), u), lerp(hash2(xi, yi + 1), hash2(xi + 1, yi + 1), u), v);
}

export function fbm(x: number, y: number): number {
  return vnoise(x, y) * 0.55 + vnoise(x * 2.1, y * 2.1) * 0.28 + vnoise(x * 4.3, y * 4.3) * 0.17;
}

export function ridge(x: number, y: number): number {
  const n = 1 - Math.abs(2 * vnoise(x, y) - 1);
  return n * n;
}

export function clamp(v: number, a: number, b: number): number {
  return Math.max(a, Math.min(b, v));
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

// ---------------------------------------------------------------------------
// Runways — plain data (no THREE), so the pure math can use it.
// RunwayPlacer.ts re-exports this table as RUNWAYS.
// ---------------------------------------------------------------------------

export interface RunwayData {
  name: string;
  x: number;
  z: number;
  elev: number; // v2 computes this as groundH(x, z); it comes out 0 for all three
  hdg: number; // degrees, heading vector = (sin hdg, 0, cos hdg)
  len: number;
  width: number; // v2 `wid`
  dirt: boolean; // v2 `surf: 'dirt' | 'paved'`
}

/** Exact v2 RUNWAYS values (template lines ~538-542). */
export const RUNWAY_TABLE: RunwayData[] = [
  { name: 'Alpha', x: -1400, z: -900, elev: 0, hdg: 90, len: 650, width: 46, dirt: false },
  { name: 'Bravo', x: 800, z: 1400, elev: 0, hdg: 160, len: 600, width: 42, dirt: true },
  { name: 'Charlie', x: 1850, z: -350, elev: 0, hdg: 20, len: 650, width: 46, dirt: false },
];

/**
 * 0..1 how strongly the runway flattens the terrain here — verbatim v2
 * runwayBlend, INCLUDING the v1.1 fix: the local frame is lx ALONG the
 * heading vector (lx = dx*sin(hdg) + dz*cos(hdg)), lz across it. v1 rotated
 * by -hdg, so the flattened strip ran 90 deg off the runway. The strip stays
 * flat for its full length plus a 100 m overrun, feathering only outside that
 * box (over 90 m along, 70 m across, smoothstepped).
 */
export function runwayBlend(x: number, z: number): number {
  let b = 0;
  for (const r of RUNWAY_TABLE) {
    const dx = x - r.x;
    const dz = z - r.z;
    const sh = Math.sin(r.hdg * D2R);
    const ch = Math.cos(r.hdg * D2R);
    const lx = dx * sh + dz * ch;
    const lz = dx * ch - dz * sh; // lx along runway, lz across
    const ax = Math.max(0, (Math.abs(lx) - (r.len / 2 + 100)) / 90);
    const az = Math.max(0, (Math.abs(lz) - r.width / 2) / 70);
    const d = Math.max(ax, az);
    if (d < 1) b = Math.max(b, 1 - d * d * (3 - 2 * d));
  }
  return b;
}

/**
 * v2 groundH() in ORIGINAL-frame coordinates (no originShift added — the
 * caller adds the floating-origin shift when querying in scene coords).
 * Rolling base + canyon ridges + plateaus, floored at -25 m, then lerped
 * toward the runway elevation (0) by runwayBlend. At a runway centre this
 * returns exactly 0.0.
 */
export function heightAt(x: number, z: number): number {
  const nx = x / 900;
  const nz = z / 900;
  let h = fbm(nx, nz) * 160 - 40; // rolling base
  h += ridge(nx * 1.7 + 5, nz * 1.7 - 3) * 90 - 30; // canyon ridges
  h += Math.max(0, fbm(nx * 0.6 + 9, nz * 0.6 + 4) - 0.55) * 420; // plateaus
  h = Math.max(h, -25);
  const b = runwayBlend(x, z);
  if (b > 0) {
    let elev = 0;
    let w = 0;
    for (const r of RUNWAY_TABLE) {
      const d = Math.hypot(x - r.x, z - r.z);
      // NOTE — deliberate v2 fix (see header): v2 weighted the runway
      // elevation over a 400 m radius, but the flat zone is len/2 + 100 =
      // 425 m from the centre, so at |lx| in [400, 425] every weight was 0,
      // the lerp was skipped, and raw terrain (up to ~50 m) poked through
      // the strip mesh at the overrun tips. 600 m covers the whole blend
      // footprint (max ~523 m at the feather corners), so the strip is 0.0
      // end to end, as the v1.1 comment promises. All v2 elevations are 0,
      // so elev/w is 0 either way — only the radius changes.
      const k = Math.max(0, 1 - d / 600);
      elev += r.elev * k;
      w += k;
    }
    if (w > 0) h = lerp(h, elev / w, b);
  }
  return h;
}

// ---------------------------------------------------------------------------
// Chunk generation (worker payload)
// ---------------------------------------------------------------------------

export interface TerrainChunkRequest {
  /** Seed for the scatter RNG. NOTE: the height function itself is
   *  coordinate-determined hash noise, so `seed` does not change heights;
   *  it is carried for future seeded detail layers and for symmetry with
   *  the generator's build(seed). */
  seed: number;
  /** Chunk indices in a grid covering [-WORLD_SIZE/2, +WORLD_SIZE/2].
   *  Chunk (cx, cz) spans [cx*size - 3000, (cx+1)*size - 3000] in x and the
   *  same in z. The whole 6x6 km terrain is chunk (0, 0) with size 6000. */
  chunkX: number;
  chunkZ: number;
  /** Chunk edge length in metres. */
  size: number;
  /** Samples per side (inclusive of both edges, so resolution^2 samples).
   *  For the full terrain use 151 (150 segments + 1). Adjacent chunks share
   *  their edge samples by this convention. */
  resolution: number;
}

export interface TerrainChunkResponse extends TerrainChunkRequest {
  /** Row-major heights, index = iz * resolution + ix (x fastest).
   *  Sample (ix, iz) sits at x = chunkX*size - 3000 + ix*step,
   *  z = chunkZ*size - 3000 + iz*step, step = size/(resolution-1).
   *  This matches THREE.PlaneGeometry vertex order after rotateX(-PI/2)
   *  (verified numerically), and the Rapier heightfield layout used by
   *  PandoraWorldGenerator. Transferable via heights.buffer. */
  heights: Float32Array;
}

/** Pure height generation for one chunk — shared by the worker handler and
 *  by the main-thread fallback in PandoraWorldGenerator. */
export function generateHeights(req: TerrainChunkRequest): Float32Array {
  const { size, resolution, chunkX, chunkZ } = req;
  const heights = new Float32Array(resolution * resolution);
  const originX = chunkX * size - WORLD_SIZE / 2;
  const originZ = chunkZ * size - WORLD_SIZE / 2;
  const step = size / (resolution - 1);
  let i = 0;
  for (let iz = 0; iz < resolution; iz++) {
    const z = originZ + iz * step;
    for (let ix = 0; ix < resolution; ix++) {
      const x = originX + ix * step;
      heights[i++] = heightAt(x, z);
    }
  }
  return heights;
}

// ---------------------------------------------------------------------------
// Web Worker entry point. Guarded so importing this module on the main
// thread (for the pure math above) is side-effect free: in a real worker
// `window` is undefined and `self` is the DedicatedWorkerGlobalScope; in a
// browser main thread `window` is defined; in Node `self` is undefined.
// ---------------------------------------------------------------------------

const workerScope = (typeof self !== 'undefined' ? self : undefined) as unknown as {
  onmessage: ((event: MessageEvent<TerrainChunkRequest>) => void) | null;
  postMessage: (message: TerrainChunkResponse, transfer: Transferable[]) => void;
} | null;

if (typeof window === 'undefined' && workerScope) {
  workerScope.onmessage = (event: MessageEvent<TerrainChunkRequest>) => {
    const req = event.data;
    const heights = generateHeights(req);
    workerScope.postMessage({ ...req, heights }, [heights.buffer]);
  };
}
