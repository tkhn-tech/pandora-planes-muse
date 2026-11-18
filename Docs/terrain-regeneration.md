# Terrain regeneration — plateau runways + mountain ranges

Date: 17 Nov 2026. No push, no browser check (parent's job).

## Reference

The Himalaya, per Wikipedia: four parallel ranges south→north
(Sivalik Hills / Lower Himalayan Range / Great Himalayas / Tibetan
Himalayas); the Great range averages 6,000 m elevation and ~25 km
width; the greatest relief is ~7,000 m over 20–30 km of horizontal
distance (Nanga Parbat region), i.e. a ~25–30% grade; the south face
is steep, the north side is a broad plateau at 4,500–5,200 m; average
slopes often exceed 30°. The game keeps those proportions at its own
scale: ~700 m of relief over ~2.5 km of horizontal distance, four
parallel bands, a steep south face on the main range and a high bench
behind it. No DEM/heightmap file was needed — everything is procedural
from the published profiles above; nothing for the user to download.

## What changed

### `src/world/PandoraTerrainWorker.ts` (the height function — the whole change lives here)
- `baseHeight(x, z)` (new, exported for probing): replaces the old
  rolling-base + canyon-ridges + plateaus field.
  - Basin floor: gentle rolling hills, −20..50 m, with fine detail.
  - Four range bands (mask: core 350 m, fade 450 m): north "Great
    Range" (lift 130 m, peaks to ~720 m), south (lift 110 m, ~560 m),
    west and east (lift 100 m, ~520 m). Domain-warped ridged
    multifractal noise, sampled anisotropically so ridgelines run
    ALONG each band; a low-frequency carve factor cuts saddles and
    valleys across the ranges at irregular intervals.
  - Valley passes (river-gorge style, cf. the Kali Gandaki gorge):
    each band near a runway dips to ~15% mask at that runway's
    along-coordinate — west at Alpha's latitude (z=−1800), south at
    Bravo's longitude (x=1600), east at Charlie's latitude (z=−700) —
    so each airfield sits in a valley mouth with the range rising on
    either side instead of burying the runway.
  - Tibetan-style bench: a broad 330 m bench north of the main range
    (z < −2350). Clamped to [−25, 750] m.
- `plateauWeight(x, z)` (new, exported): 1 on a plateau's flat top
  (strip + 160 m along, + 70 m across), smoothstepping to 0 over a
  350 m skirt. `heightAt()` now blends the base terrain toward the
  runway's plateau elevation by this weight — where the weight is 1
  the height is exactly the plateau elevation. This replaces the old
  `runwayBlend` flattening (which cut a flat strip into hilly ground
  and left hills at the runway ends). `runwayBlend` survives unchanged
  for the strip colouring and flora exclusion.
- `RUNWAY_TABLE` elevations (were all 0): Alpha **50 m** (−2800,−1800,
  hdg 90, 650 m), Bravo **50 m** (1600,2800, hdg 160, 600 m, dirt),
  Charlie **100 m** (3700,−700, hdg 20, 650 m). Each was chosen above
  the highest base terrain within ~50 m of its flat-box boundary
  (Alpha 33.1 m, Bravo 31.9 m, Charlie 57.7 m), so every plateau is
  the local high ground and its sides slope down to the surrounding
  terrain.
- Helpers added: `smoothstep()`, `ridgedFbm()`. The worker stays
  free of THREE/Rapier/DOM. Same seed (1337). **12 km map (V4 pass
  2, issue 10):** the grid is now **201×201 at 60 m spacing**
  (`WORLD_SIZE = 12000`, `TERRAIN_SEGMENTS = 200`, `GRID_STEP = 60`),
  so the sample count (~40k) and the mesh/heightfield (~80k tris)
  are the same as the 6 km map and mobile performance is unchanged.
  All noise frequencies, band positions and widths scale by
  `MAP_SCALE = WORLD_SIZE / 6000` (= 2): range bands at c ±4600/±4900
  (core 700, fade 900), valley gaps at the new runway
  along-coordinates (west z=−1800, south x=1600, east z=−700), the
  north bench north of z=−4700. The carrier loop waypoints are
  radius ~3000 (start (−1900,350,−1800), 900 m dead ahead of Alpha).

### `src/world/PandoraWorldGenerator.ts`
- Terrain colouring: plateau tops (plateauWeight > 0.5) stay grassy
  instead of going grey rock with height (Charlie's top is 220 m);
  rock grey above 120 m on mountain slopes, snow white above 550 m.
- `plateauWeight` imported and re-exported. Nothing else changed:
  mesh + heightfield from the same heights, runway box colliders
  (top at elev + 0.05, automatic from the table), floating-origin
  registration, islands/flora/clouds/waterfalls untouched.

### `src/world/RunwayPlacer.ts`
- Comments only: the flattening is now the plateau blend, not
  `runwayBlend`. No behaviour change — strip meshes sit at
  `heightAt(centre)` = plateau elevation, spawn/respawn already use
  `r.elev` everywhere.

### `scripts/probe-terrain.ts` (new)
- 22 node checks: per-runway strip flatness (analytic + sampled
  grid), plateau-above-surroundings, skirt rejoins base, plateau
  weight 1 across the flat top, carrier-loop clearance, takeoff
  corridor, mountain mass, basin valley floor.

### `scripts/takeoff-driver.ts`
- Elevation assumptions updated, nothing else: the section-E
  flatness checks and the spawn-settle / F-35A hover checks now
  compare against `alpha.elev` instead of 0 / 2.2 m absolute. The
  performance thresholds are untouched (lift-off < 650 m, speed
  < 100 m/s, max AoA < 30°). This is not a loosening — the same
  strictness, only the reference elevation moved with the terrain.

## Verification (all green)

- `scripts/probe-terrain.ts`: **22/22 PASS** — every strip exactly
  flat (worst |h − elev| = 0.00, analytic and grid); plateaus clear
  their edges (Alpha 85 vs 69 m base, Bravo 70 vs 55 m, Charlie 220
  vs 202 m); carrier loop max terrain 125.8 m (hull bottom ~329 m);
  takeoff corridor max 83.5 m at the overrun tip; peaks 750 m
  (capped), 10.1% of the map above 500 m, 5.8% above 600 m; basin
  valley floor −7 m.
- `scripts/takeoff-driver.ts`: **52/52 PASS** — takeoffs unchanged:
  F-18 133 kt/260 m, F-35A 156 kt/345 m, F-22 147 kt/247 m,
  F-15E 158 kt/283 m (flaps 0); F-35A hover 94%/74% checks pass;
  all 4 deck spawns pass; 60° and 80° turns pass.
- `npm run build`: **0 tsc errors**, built in ~5.3 s.

## Deliberately unchanged

- World size (**12×12 km** since V4 pass 2, issue 10), seed (1337),
  grid (**201×201, 60 m**), floating origin, helicarrier (flies its
  loop at y=350 terrain-following since issue 8; terrain along the
  loop stays below 48 m, so nothing clips it), spawn points (Alpha
  runway / carrier deck, both elev-relative already), islands/flora/
  clouds/waterfalls (Pandora flavour kept), runway positions and
  headings, the runway box colliders, the AI flight/damage models.
- The north bench + overlapping ranges hit the 750 m cap in the far
  north-west corner — a small flat-topped patch at exactly 750 m.
  It reads as a high plateau; left as-is.
- The 165 m rise at x=1600 on the z=−900 line was Charlie's plateau
  skirt (its old 220 m mesa) on the 6 km map; on the 12 km map
  Charlie sits at (3700,−700, elev 100 m) and the takeoff corridor
  (Alpha, hdg 90) reaches max 50.0 m at 425 m downrange — the
  plateau itself, not a hill.
