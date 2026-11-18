// probe-carrier-start.ts — CARRIER DOWN false alarm at round start
// (pass 5, item 4).
//
// The v4 play-test: a "CARRIER DOWN — A helicarrier is lost" banner
// fires within seconds of EVERY round start (00:01-00:09), 0 enemies,
// while the carrier is visibly still flying. The only round-start
// path that fires it is HelicarrierController.update()'s instant
// terrain-death safety net: `if (pos.y < groundY + 10) { alive =
// false; onCarrierDown(); }`. At round start the carrier resets to
// the loop start (y=350) and terrain-following can only climb 15
// m/s, so if the heightfield directly below the loop start (or any
// early-loop point) is above ~340 m, the carrier dies on the first
// frame — before it can climb out. This probe measures the real
// heightfield (the same generateHeights + heightfieldHeightAt the
// game uses) at the loop start and along the whole loop, and
// requires the carrier's cruise altitude to clear the ground by
// more than the death margin at every point it will fly over in
// the first minute.
//
// Usage: node_modules/.bin/esbuild scripts/probe-carrier-start.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-carrier-start.mjs
//   && node /tmp/probe-carrier-start.mjs

import {
  generateHeights,
  heightfieldHeightAt,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';
import {
  CARRIER_START_POS,
  CARRIER_WAYPOINTS,
} from '../src/entities/HelicarrierController';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

function main(): void {
  const heights = generateHeights({
    seed: 1337,
    chunkX: 0,
    chunkZ: 0,
    size: WORLD_SIZE,
    resolution: TERRAIN_SEGMENTS + 1,
  });
  const res = TERRAIN_SEGMENTS + 1;
  const hAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);
  // The instant-death margin: pos.y < groundY + 10 kills the
  // carrier. At the loop start the carrier is at y=350, so the
  // ground below must be < 340 — and with terrain-following the
  // carrier climbs only 15 m/s, so a ground above 340 anywhere on
  // the early loop is a death sentence on the first frames.
  const startGround = hAt(CARRIER_START_POS.x, CARRIER_START_POS.z);
  check(
    'loop start ground is below the instant-death line (350 - 10 = 340 m)',
    startGround < 340,
    `ground at loop start (${CARRIER_START_POS.x}, ${CARRIER_START_POS.z}) = ${startGround.toFixed(1)} m, carrier y=350`,
  );
  // Sample the whole loop (waypoints + midpoints, ~120 points):
  // the carrier must clear the death line everywhere, not just at
  // the start — terrain-following climbs toward max(350, ahead+150)
  // but the death check reads the ground DIRECTLY below, so a steep
  // rise under the carrier can kill it even while it climbs.
  let worst = -Infinity;
  let worstAt = '';
  const N = 120;
  for (let i = 0; i < N; i++) {
    const t = i / N;
    const seg = Math.floor(t * CARRIER_WAYPOINTS.length) % CARRIER_WAYPOINTS.length;
    const a = CARRIER_WAYPOINTS[seg]!;
    const b = CARRIER_WAYPOINTS[(seg + 1) % CARRIER_WAYPOINTS.length]!;
    const f = t * CARRIER_WAYPOINTS.length - seg;
    const x = a.x + (b.x - a.x) * f;
    const z = a.z + (b.z - a.z) * f;
    const g = hAt(x, z);
    if (g > worst) {
      worst = g;
      worstAt = `(${x.toFixed(0)}, ${z.toFixed(0)})`;
    }
  }
  check(
    'whole loop clears the instant-death line (ground < 340 m everywhere)',
    worst < 340,
    `worst ground on the loop = ${worst.toFixed(1)} m at ${worstAt}`,
  );
  console.log(failures === 0 ? '\nALL CARRIER-START CHECKS PASSED' : `\n${failures} CARRIER-START CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
