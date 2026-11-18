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

export const WORLD_SIZE = 12000; // v4 WORLD: world half-extent is +/-6000 m
export const WORLD_SEED = 1337; // v2 `rand` seed
export const TERRAIN_SEGMENTS = 200; // v4 terrain mesh SEG; -> 201x201 samples
/** Grid spacing in metres (WORLD_SIZE / TERRAIN_SEGMENTS). The 6 km map
 *  used 40 m; the 12 km map uses 60 m, keeping the sample count (~40k)
 *  and the mesh / heightfield triangle count (~80k) about the same as
 *  the 6 km map so mobile performance is unchanged. Plateau flatness is
 *  analytic (exact where plateauWeight is 1), so the coarser grid does
 *  not cost runway fidelity. */
export const GRID_STEP = WORLD_SIZE / TERRAIN_SEGMENTS;

/** Map scale factor vs the 6 km terrain regeneration (WORLD_SIZE /
 *  6000 = 2 for the 12 km map). All noise frequencies, band positions
 *  and widths are multiplied by this so the terrain keeps the same
 *  proportions — same number of ridges across each range, same
 *  valley-pass geometry relative to the runways — at the larger size. */
const MAP_SCALE = WORLD_SIZE / 6000;

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

/** Smoothstep 0..1 between edges a and b (a < b). */
export function smoothstep(a: number, b: number, v: number): number {
  const t = clamp((v - a) / (b - a), 0, 1);
  return t * t * (3 - 2 * t);
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

/** Ridged multifractal (3 octaves of ridge()) — sharp crests, the
 *  mountain-range building block. Returns ~0..1 (mean ~0.33). */
export function ridgedFbm(x: number, y: number): number {
  return ridge(x, y) * 0.6 + ridge(x * 2.1 + 7, y * 2.1 - 3) * 0.28 + ridge(x * 4.3 - 5, y * 4.3 + 9) * 0.12;
}

// ---------------------------------------------------------------------------
// Mountain ranges — Himalaya as the reference (see heightAt below).
// Four parallel bands, like the Himalaya's Sivalik foothills / Lower
// range / Great range / Tibetan plateau, scaled to the 6 km world:
// the real relief is ~7,000 m over 20-30 km of horizontal distance
// (Nanga Parbat region), i.e. ~25-30% grade; the game keeps that
// proportion at ~700 m of relief over ~2.5 km. Ranges are placed in
// the outer bands of the map so the central basin — the carrier loop
// (hull bottom ~329 m) and the runway plateaus — stays clear of them.
// ---------------------------------------------------------------------------

interface RangeBand {
  /** 'z': band runs east-west (centreline z = c + arc*sin(coord/scale)). */
  axis: 'x' | 'z';
  /** Centreline offset (m) from the world centre on the band's axis. */
  c: number;
  /** Arc amplitude (m) of the centreline's sinusoidal bend. */
  arc: number;
  /** Arc wavelength (m). */
  arcScale: number;
  /** Half-width (m) of the flat core where the mask is 1. */
  core: number;
  /** Fade distance (m) from core edge to mask 0. */
  fade: number;
  /** Base lift (m) inside the band — the range sits on a high base. */
  lift: number;
  /** Peak amplitude (m) of the ridged crests. */
  amp: number;
  /** Valley pass through the band (a river-gorge-style gap, like the
   *  Kali Gandaki gorge through the Himalaya): the band's mask dips
   *  to ~15% within `halfWidth` of the along-axis coordinate `at`,
   *  so an airfield plateau sits in a valley mouth with the range
   *  rising on either side instead of burying the runway. */
  gap?: { at: number; halfWidth: number };
}

/** The four range bands. North is the "Great Range" (tallest, with the
 *  Tibetan-style bench behind it); south/west/east are lower. Each band
 *  near a runway has a valley pass at that runway's along-coordinate:
 *  west at Alpha's latitude (z=-1800), south at Bravo's longitude
 *  (x=1600), east at Charlie's latitude (z=-700). Band centres are at
 *  ±4600/±4900 m (twice the 6 km ±2300/±2450), so the central basin —
 *  runways + carrier loop — stays clear of them. */
const RANGE_BANDS: RangeBand[] = [
  { axis: 'z', c: -2300 * MAP_SCALE, arc: 150 * MAP_SCALE, arcScale: 1800 * MAP_SCALE, core: 350 * MAP_SCALE, fade: 450 * MAP_SCALE, lift: 130, amp: 720 },
  { axis: 'z', c: 2300 * MAP_SCALE, arc: 150 * MAP_SCALE, arcScale: 1600 * MAP_SCALE, core: 350 * MAP_SCALE, fade: 450 * MAP_SCALE, lift: 110, amp: 560, gap: { at: 800 * MAP_SCALE, halfWidth: 700 * MAP_SCALE } },
  { axis: 'x', c: -2300 * MAP_SCALE, arc: 150 * MAP_SCALE, arcScale: 1700 * MAP_SCALE, core: 350 * MAP_SCALE, fade: 450 * MAP_SCALE, lift: 100, amp: 520, gap: { at: -900 * MAP_SCALE, halfWidth: 700 * MAP_SCALE } },
  { axis: 'x', c: 2450 * MAP_SCALE, arc: 100 * MAP_SCALE, arcScale: 1700 * MAP_SCALE, core: 350 * MAP_SCALE, fade: 450 * MAP_SCALE, lift: 100, amp: 520, gap: { at: -350 * MAP_SCALE, halfWidth: 700 * MAP_SCALE } },
];

/** Mask 0..1 of one range band at (x, z): 1 in the core, fading to 0
 *  `fade` metres outside it, and dipping through the valley pass. */
function rangeMask(b: RangeBand, x: number, z: number): number {
  const along = b.axis === 'z' ? x : z;
  const across = b.axis === 'z' ? z : x;
  const centre = b.c + b.arc * Math.sin(along / b.arcScale);
  const d = Math.abs(across - centre);
  let m = 1 - smoothstep(b.core, b.core + b.fade, d);
  if (b.gap && m > 0) {
    const gd = Math.abs(along - b.gap.at);
    m *= 0.15 + 0.85 * smoothstep(b.gap.halfWidth * 0.5, b.gap.halfWidth, gd);
  }
  return m;
}

/**
 * Base terrain WITHOUT runway plateaus (the mountain/foothill/valley
 * field the plateaus sit in). Exported for probing / verification —
 * the game itself only ever calls heightAt().
 *
 * - Basin floor: gentle rolling hills, -20..50 m with fine detail.
 * - Ranges: domain-warped ridged noise, anisotropic so ridgelines run
 *   ALONG each band (long wavelength along the range, short across it),
 *   with a low-frequency carve factor that cuts saddles/valleys across
 *   the range at irregular intervals. Typical crests 350-550 m, peaks
 *   to ~700 m in the north (Great) range.
 * - North bench: a broad high bench (330 m) north of the main range,
 *   the Tibetan-plateau analogue — the Himalaya's asymmetric profile
 *   (steep south face, high plateau to the north).
 * Clamped to [-25, 750] m.
 */
export function baseHeight(x: number, z: number): number {
  let h = fbm(x / (900 * MAP_SCALE), z / (900 * MAP_SCALE)) * 70 - 20;
  h += fbm(x / (300 * MAP_SCALE) + 3, z / (300 * MAP_SCALE) - 5) * 25;
  // Low-frequency carve: saddles and valleys across the ranges.
  const carve = 0.45 + 0.55 * smoothstep(0.32, 0.72, fbm(x / (480 * MAP_SCALE) + 11, z / (480 * MAP_SCALE) - 7));
  for (const b of RANGE_BANDS) {
    const m = rangeMask(b, x, z);
    if (m <= 0) continue;
    // Domain warp bends the ridgelines (same warp for every band so
    // the field stays continuous where bands meet at the corners).
    const wx = x + (fbm(x / (700 * MAP_SCALE) + 5, z / (700 * MAP_SCALE) - 3) - 0.5) * 260 * MAP_SCALE;
    const wz = z + (fbm(x / (700 * MAP_SCALE) - 9, z / (700 * MAP_SCALE) + 4) - 0.5) * 260 * MAP_SCALE;
    // Anisotropic sampling: long wavelength ALONG the range, short
    // ACROSS it, so crests run parallel to the range axis.
    const r =
      b.axis === 'z'
        ? ridgedFbm(wx / (650 * MAP_SCALE), wz / (200 * MAP_SCALE))
        : ridgedFbm(wx / (200 * MAP_SCALE), wz / (650 * MAP_SCALE));
    h += m * (b.lift + Math.pow(r, 1.35) * b.amp) * carve;
  }
  // Tibetan-style bench north of the main (north) range.
  const benchT = clamp((-2350 * MAP_SCALE - z) / (400 * MAP_SCALE), 0, 1);
  h += 330 * benchT * benchT * (3 - 2 * benchT);
  return clamp(h, -25, 750);
}

// ---------------------------------------------------------------------------
// Runways — plain data (no THREE), so the pure math can use it.
// RunwayPlacer.ts re-exports this table as RUNWAYS.
// ---------------------------------------------------------------------------

export interface RunwayData {
  name: string;
  x: number;
  z: number;
  /** Plateau-top elevation (m). Each runway sits on its own flat-topped
   *  plateau: the top is exactly this high, the plateau is the local
   *  high ground (chosen above the highest base terrain within ~700 m
   *  of the runway), and the sides slope down to the surrounding
   *  terrain over the plateau skirt. */
  elev: number;
  hdg: number; // degrees, heading vector = (sin hdg, 0, cos hdg)
  len: number;
  width: number; // v2 `wid`
  dirt: boolean; // v2 `surf: 'dirt' | 'paved'`
}

/** Plateau elevations — each chosen above the highest base terrain
 *  within ~50 m of its flat-box boundary (verified by
 *  scripts/probe-terrain.ts), so every plateau is the local high ground
 *  and its sides slope down to the surrounding terrain. The 12 km
 *  runways are the 6 km positions doubled: Alpha (-2800,-1800) at the
 *  west-range valley pass (50 m; base edge max 33 m, ring max 40 m),
 *  Bravo (1600,2800) at the south-range pass (50 m; base edge max
 *  32 m, ring max 37 m), Charlie (3700,-700) a high mesa at the foot
 *  of the east range (100 m; base edge max 58 m, ring max 82 m). */
export const RUNWAY_TABLE: RunwayData[] = [
  { name: 'Alpha', x: -2800, z: -1800, elev: 50, hdg: 90, len: 650, width: 46, dirt: false },
  { name: 'Bravo', x: 1600, z: 2800, elev: 50, hdg: 160, len: 600, width: 42, dirt: true },
  { name: 'Charlie', x: 3700, z: -700, elev: 100, hdg: 20, len: 650, width: 46, dirt: false },
];

/** Plateau geometry (m): the flat top extends past the strip, then the
 *  skirt slopes down to the base terrain. */
export const PLATEAU_TOP_ALONG = 160; // flat top past each strip end (overrun is 100)
export const PLATEAU_TOP_ACROSS = 70; // flat top past each strip edge
export const PLATEAU_SKIRT = 350; // slope-down distance past the flat top

/**
 * 0..1 plateau weight at (x, z): 1 on a plateau's flat top, fading to 0
 * across the skirt. The max over runways (they are far apart, so in
 * practice at most one is non-zero). Exported for the terrain colouring
 * (plateau tops read as grass, not rock) and for verification.
 */
export function plateauWeight(x: number, z: number): number {
  let w = 0;
  for (const r of RUNWAY_TABLE) {
    const dx = x - r.x;
    const dz = z - r.z;
    const sh = Math.sin(r.hdg * D2R);
    const ch = Math.cos(r.hdg * D2R);
    const lx = dx * sh + dz * ch;
    const lz = dx * ch - dz * sh;
    const ex = Math.max(0, Math.abs(lx) - (r.len / 2 + PLATEAU_TOP_ALONG));
    const ez = Math.max(0, Math.abs(lz) - (r.width / 2 + PLATEAU_TOP_ACROSS));
    const d = Math.max(ex, ez);
    if (d < PLATEAU_SKIRT) w = Math.max(w, 1 - smoothstep(0, PLATEAU_SKIRT, d));
  }
  return w;
}

/**
 * 0..1 how strongly the runway flattens the terrain here — verbatim v2
 * runwayBlend, INCLUDING the v1.1 fix: the local frame is lx ALONG the
 * heading vector (lx = dx*sin(hdg) + dz*cos(hdg)), lz across it. v1 rotated
 * by -hdg, so the flattened strip ran 90 deg off the runway. The strip stays
 * flat for its full length plus a 100 m overrun, feathering only outside that
 * box (over 90 m along, 70 m across, smoothstepped).
 * Kept for the strip colouring and flora exclusion; the terrain flattening
 * itself is now the plateau blend in heightAt() (the plateau flat top
 * fully contains this strip zone).
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
 * Ground height in ORIGINAL-frame coordinates (no originShift added —
 * the caller adds the floating-origin shift when querying in scene
 * coords). Mountain ranges + plateau runways (see baseHeight and
 * plateauWeight above). The plateau blend replaces the v2 runway
 * flattening: where the plateau weight is 1 the height is exactly the
 * runway's plateau elevation, so the strip is flat for its full length
 * plus overrun on a flat-topped mesa whose sides slope down to the
 * surrounding terrain — no hills at the runway ends.
 */
export function heightAt(x: number, z: number): number {
  const h = baseHeight(x, z);
  // Plateau blend: the plateau is the local high ground, so blend the
  // base terrain toward the plateau elevation by the plateau weight.
  // Where the weight is 1 (the flat top) this returns exactly elev.
  let w = 0;
  let elev = 0;
  for (const r of RUNWAY_TABLE) {
    const dx = x - r.x;
    const dz = z - r.z;
    const sh = Math.sin(r.hdg * D2R);
    const ch = Math.cos(r.hdg * D2R);
    const lx = dx * sh + dz * ch;
    const lz = dx * ch - dz * sh;
    const ex = Math.max(0, Math.abs(lx) - (r.len / 2 + PLATEAU_TOP_ALONG));
    const ez = Math.max(0, Math.abs(lz) - (r.width / 2 + PLATEAU_TOP_ACROSS));
    const d = Math.max(ex, ez);
    if (d < PLATEAU_SKIRT) {
      const wr = 1 - smoothstep(0, PLATEAU_SKIRT, d);
      if (wr > w) {
        w = wr;
        elev = r.elev;
      }
    }
  }
  if (w > 0) return lerp(h, elev, w);
  return h;
}

/** Height of the RAPIER HEIGHTFIELD surface at (x, z) — the exact
 *  piecewise-linear triangle surface the physics collider (and the
 *  visual mesh) actually are, interpolated over the 201x201 sampled
 *  grid (60 m spacing, x/z from -6000, row-major heights,
 *  index = iz * res + ix).
 *
 *  This is NOT the bilinear interpolation (sampledHeightAt in
 *  PandoraWorldGenerator): Rapier splits each quad along the
 *  (i+1,j)–(i,j+1) diagonal (corners h10–h01), and the bilinear
 *  surface sits up to ~7.4 m away from the triangle surface at the
 *  steepest spots (measured at the (840..880, -2520..-2480) cell on
 *  the 6 km map: bilinear 572.73 vs raycast 580.17). The ANALYTIC
 *  heightAt() is worse still (up to ~107 m off the sampled surface
 *  where the ridged-fbm features are undersampled by the grid).
 *
 *  The triangle diagonal was verified against Rapier raycasts: 6
 *  points inside the steep cell above agree with this formula to
 *  0.000 m; the other diagonal is off by up to 7.44 m.
 *
 *  Use this — never heightAt() or the bilinear — for anything that
 *  must agree with where a jet physically rests: the controller's
 *  terrainHeightAt (ground detection, respawn elevation clamp), the
 *  crash-VFX ground height, and the camera terrain clamp. A jet
 *  resting on a mountainside reads grounded=true and G=1.0 with this
 *  (the v2 play-test hang was a jet resting on the collider while the
 *  analytic height put it 90+ m in the air, so the controller never
 *  saw it land and the respawn never fired). */
export function heightfieldHeightAt(
  heights: Float32Array,
  res: number,
  x: number,
  z: number,
): number {
  const gx = (x + WORLD_SIZE / 2) / GRID_STEP;
  const gz = (z + WORLD_SIZE / 2) / GRID_STEP;
  const ix = Math.max(0, Math.min(res - 2, Math.floor(gx)));
  const iz = Math.max(0, Math.min(res - 2, Math.floor(gz)));
  const fx = Math.max(0, Math.min(1, gx - ix));
  const fz = Math.max(0, Math.min(1, gz - iz));
  const h00 = heights[iz * res + ix]!;
  const h10 = heights[iz * res + ix + 1]!;
  const h01 = heights[(iz + 1) * res + ix]!;
  const h11 = heights[(iz + 1) * res + ix + 1]!;
  // Rapier's heightfield diagonal: (i+1,j)–(i,j+1), i.e. h10–h01.
  // fx+fz<=1 is the h00/h10/h01 triangle, else h10/h11/h01.
  if (fx + fz <= 1) return h00 + (h10 - h00) * fx + (h01 - h00) * fz;
  return h11 + (h01 - h11) * (1 - fx) + (h10 - h11) * (1 - fz);
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
