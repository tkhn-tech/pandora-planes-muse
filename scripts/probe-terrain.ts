// probe-terrain.ts — node verification for the plateau-runway /
// mountain-range terrain regeneration.
//
// Proves, from the same pure height function the mesh and the Rapier
// heightfield are built from:
//  1. Every runway strip (+ overrun) is exactly flat at its plateau
//     elevation (analytic heightAt AND the sampled 40 m grid).
//  2. Every plateau is the local high ground (elev clears the highest
//     base terrain within ~50 m of its flat-box boundary).
//  3. The carrier loop stays clear of terrain (hull bottom ~329 m).
//  4. The takeoff corridor east of Alpha stays low (no hill at the
//     runway end — the v1 bug).
//  5. The ranges are real mountains (peaks 600-750 m, valleys low).
//
// Usage: node_modules/.bin/esbuild scripts/probe-terrain.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-terrain.mjs
//   && node /tmp/probe-terrain.mjs

import {
  baseHeight,
  heightAt,
  plateauWeight,
  generateHeights,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
  GRID_STEP,
} from '../src/world/PandoraTerrainWorker';
import { CARRIER_START_POS, CARRIER_WAYPOINTS } from '../src/entities/HelicarrierController';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

function main(): void {
  const res = TERRAIN_SEGMENTS + 1;
  const heights = generateHeights({
    seed: 1337,
    chunkX: 0,
    chunkZ: 0,
    size: WORLD_SIZE,
    resolution: res,
  });
  // Bilinear over the sampled grid — the surface the mesh and the
  // heightfield actually render/collide with.
  const half = WORLD_SIZE / 2;
  const bilinear = (x: number, z: number): number => {
    const gx = (x + half) / GRID_STEP;
    const gz = (z + half) / GRID_STEP;
    const ix = Math.max(0, Math.min(res - 2, Math.floor(gx)));
    const iz = Math.max(0, Math.min(res - 2, Math.floor(gz)));
    const fx = gx - ix;
    const fz = gz - iz;
    const h00 = heights[iz * res + ix]!;
    const h10 = heights[iz * res + ix + 1]!;
    const h01 = heights[(iz + 1) * res + ix]!;
    const h11 = heights[(iz + 1) * res + ix + 1]!;
    return h00 * (1 - fx) * (1 - fz) + h10 * fx * (1 - fz) + h01 * (1 - fx) * fz + h11 * fx * fz;
  };

  for (const r of RUNWAY_TABLE) {
    const sh = Math.sin((r.hdg * Math.PI) / 180);
    const ch = Math.cos((r.hdg * Math.PI) / 180);
    const toWorld = (lx: number, lz: number): [number, number] => [
      r.x + lx * sh + lz * ch,
      r.z + lx * ch - lz * sh,
    ];
    // 1. Strip + overrun exactly flat (analytic).
    let worst = 0;
    let worstAt = '';
    for (let lx = -(r.len / 2 + 100); lx <= r.len / 2 + 100; lx += 5) {
      for (const lz of [0, -r.width / 4, r.width / 4]) {
        const [x, z] = toWorld(lx, lz);
        const d = Math.abs(heightAt(x, z) - r.elev);
        if (d > worst) {
          worst = d;
          worstAt = `(${x.toFixed(0)},${z.toFixed(0)})`;
        }
      }
    }
    check(
      `${r.name} strip + 100 m overrun exactly flat at elev ${r.elev} m (analytic)`,
      worst < 1e-6,
      `worst |h - elev| = ${worst.toExponential(2)} at ${worstAt}`,
    );
    // 2. Flat on the sampled grid over the takeoff roll (0..350 m along
    //    the takeoff heading from the centre).
    let gridWorst = 0;
    for (let d = 0; d <= 350; d += 5) {
      const [x, z] = toWorld(d, 0);
      gridWorst = Math.max(gridWorst, Math.abs(bilinear(x, z) - r.elev));
    }
    check(
      `${r.name} roll path flat on the sampled ${GRID_STEP} m grid too`,
      gridWorst < 0.01,
      `worst |h - elev| = ${gridWorst.toFixed(4)}`,
    );
    // 3. Plateau is the local high ground: elev clears the highest
    //    BASE terrain within ~50 m of the flat-box boundary, and sits
    //    above the ring mean by a clear margin.
    const LA = r.len / 2 + 160;
    const LC = r.width / 2 + 70;
    let ringMax = -1e9;
    let ringSum = 0;
    let ringN = 0;
    for (let a = 0; a < 360; a += 6) {
      const rad = (a * Math.PI) / 180;
      for (const rr of [520, 600, 700]) {
        const x = r.x + Math.cos(rad) * rr;
        const z = r.z + Math.sin(rad) * rr;
        const h = baseHeight(x, z);
        ringSum += h;
        ringN++;
        if (h > ringMax) ringMax = h;
      }
    }
    const ringMean = ringSum / ringN;
    let edgeMax = -1e9;
    for (let lx = -LA - 50; lx <= LA + 50; lx += 10) {
      for (const lz of [-LC - 50, LC + 50]) {
        const [x, z] = toWorld(lx, lz);
        edgeMax = Math.max(edgeMax, baseHeight(x, z));
      }
    }
    for (let lz = -LC - 50; lz <= LC + 50; lz += 10) {
      for (const lx of [-LA - 50, LA + 50]) {
        const [x, z] = toWorld(lx, lz);
        edgeMax = Math.max(edgeMax, baseHeight(x, z));
      }
    }
    check(
      `${r.name} plateau (${r.elev} m) clears the base terrain at its edge`,
      r.elev > edgeMax,
      `highest base within 50 m of the flat box = ${edgeMax.toFixed(1)} m`,
    );
    check(
      `${r.name} plateau sits above its surroundings`,
      r.elev > ringMean + 10,
      `ring (520-700 m) mean base = ${ringMean.toFixed(1)} m, max = ${ringMax.toFixed(1)} m`,
    );
    // Skirt blends back to base terrain past the skirt (w = 0 there).
    const [sx, sz] = toWorld(LA + 400, 0);
    const skirtDev = Math.abs(heightAt(sx, sz) - baseHeight(sx, sz));
    check(
      `${r.name} skirt rejoins base terrain past the skirt`,
      skirtDev < 0.01,
      `|heightAt - base| = ${skirtDev.toFixed(4)} at 400 m past the flat box`,
    );
    // Plateau weight is 1 across the whole flat top.
    let wMin = 1;
    for (let lx = -LA; lx <= LA; lx += 25) {
      for (let lz = -LC; lz <= LC; lz += 25) {
        const [x, z] = toWorld(lx, lz);
        wMin = Math.min(wMin, plateauWeight(x, z));
      }
    }
    check(`${r.name} plateau weight is 1 across the flat top`, wMin === 1, `min w = ${wMin}`);
  }

  // 4. Carrier loop clearance: the helicarrier flies at y=350 with its
  //    hull bottom ~329 m — terrain along its whole circuit must stay
  //    well below that.
  const loop: [number, number][] = [
    [CARRIER_START_POS.x, CARRIER_START_POS.z],
    ...CARRIER_WAYPOINTS.map((w): [number, number] => [w.x, w.z]),
    [CARRIER_START_POS.x, CARRIER_START_POS.z],
  ];
  let loopMax = -1e9;
  let loopAt = '';
  for (let s = 0; s < loop.length - 1; s++) {
    const [x0, z0] = loop[s]!;
    const [x1, z1] = loop[s + 1]!;
    const steps = Math.ceil(Math.hypot(x1 - x0, z1 - z0) / 25);
    for (let i = 0; i <= steps; i++) {
      const x = x0 + ((x1 - x0) * i) / steps;
      const z = z0 + ((z1 - z0) * i) / steps;
      const h = heightAt(x, z);
      if (h > loopMax) {
        loopMax = h;
        loopAt = `(${x.toFixed(0)},${z.toFixed(0)})`;
      }
    }
  }
  check(
    'Carrier loop clear of terrain (hull bottom ~329 m)',
    loopMax < 300,
    `max terrain on the loop = ${loopMax.toFixed(1)} m at ${loopAt}`,
  );

  // 5. Takeoff corridor east of Alpha stays low (the v1 bug: a hill at
  //    the runway end). From the overrun tip to 2 km downrange along
  //    Alpha's takeoff heading — where the jet is still low and
  //    climbing. (Computed from Alpha's table entry, so it follows the
  //    runway if the table moves.)
  const alpha = RUNWAY_TABLE[0]!;
  const ash = Math.sin((alpha.hdg * Math.PI) / 180);
  const ach = Math.cos((alpha.hdg * Math.PI) / 180);
  const tipAlong = alpha.len / 2 + 100;
  let corridorMax = -1e9;
  let corridorAt = '';
  for (let d = tipAlong; d <= tipAlong + 2000; d += 25) {
    const x = alpha.x + d * ash;
    const z = alpha.z + d * ach;
    const h = heightAt(x, z);
    if (h > corridorMax) {
      corridorMax = h;
      corridorAt = `${d.toFixed(0)} m downrange`;
    }
  }
  check(
    'Takeoff corridor east of Alpha stays low (no hill at the runway end)',
    corridorMax < 100,
    `max = ${corridorMax.toFixed(1)} m at ${corridorAt}`,
  );

  // 6. The ranges are real mountains: peaks 600-750 m, valleys low.
  let gmax = -1e9;
  let gmaxAt = '';
  let above500 = 0;
  let above600 = 0;
  let total = 0;
  for (let x = -6000; x <= 6000; x += 60) {
    for (let z = -6000; z <= 6000; z += 60) {
      const h = baseHeight(x, z);
      total++;
      if (h > 500) above500++;
      if (h > 600) above600++;
      if (h > gmax) {
        gmax = h;
        gmaxAt = `(${x},${z})`;
      }
    }
  }
  check(
    'Mountain peaks reach 600-750 m',
    gmax >= 600 && gmax <= 750,
    `max base height = ${gmax.toFixed(1)} m at ${gmaxAt}`,
  );
  check(
    'Ranges have real mass (area above 500 m)',
    above500 / total > 0.03,
    `${((above500 / total) * 100).toFixed(1)}% of the map above 500 m, ${((above600 / total) * 100).toFixed(1)}% above 600 m`,
  );
  // Basin valley floor stays low.
  let basinMin = 1e9;
  for (let x = -2400; x <= 2400; x += 60) {
    for (let z = -2000; z <= 2000; z += 60) {
      basinMin = Math.min(basinMin, baseHeight(x, z));
    }
  }
  check('Central basin has a low valley floor', basinMin < 60, `basin min = ${basinMin.toFixed(1)} m`);

  console.log(failures === 0 ? '\nALL TERRAIN CHECKS PASSED' : `\n${failures} TERRAIN CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
