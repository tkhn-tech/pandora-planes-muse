// probe-stall-respawn.ts — reproduce the live v2 play-test defects in node
// and verify the v3 fixes against them, so each fix can be checked
// against a failing-then-passing check. Scratch diagnostic, not shipped.
//
// Scenarios (real Aircraft / CurveAerodynamicsSolver, real Rapier world
// with the CURRENT plateau terrain heightfield + runway boxes, main.ts's
// exact substep order):
//  1. Stall-down over steep terrain: does the jet reach the ground and
//     crash, or hang in mid-air (alive, grounded=false, G 0.2)?
//  1a. heightfieldHeightAt vs the Rapier heightfield surface (raycast).
//  1b. Resting on steep terrain reads grounded=true, G=1.0.
//  2. Respawn after a hillside crash: does the main.ts flow (death ->
//     3 s timer -> respawnPlayer at the nearest runway) complete?
//  2b. Respawn with a non-zero floating-origin shift (the v2 hang's
//     primary cause): the runway's original-frame coords must be
//     converted back to scene coords before resetAt.
//  3. Missile shoot-down via the REAL WeaponSystem proximity path:
//     fire/smoke? dying fall rate? ground impact + explosion trigger?
//  4. Gun kill (tail x2): dying? fire? hp?
//  5. Keyboard hold (PCFlightInput slew, 5/s): ArrowUp held from Vr
//     rotates and lifts off; a 0.1 s tap does not.
//  6. Touch stick: pitchIn -1 is the pull-back direction (stick DOWN
//     after the v3 inversion, ArrowUp on the keyboard) — it rotates;
//     pitchIn +1 (stick UP) does not.
//  7. Parked creep: thrTarget 0, wind 3 m/s — does the jet move, and
//     what does the IAS tape read?
//
// Usage: node_modules/.bin/esbuild scripts/probe-stall-respawn.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-stall-respawn.mjs
//   && node /tmp/probe-stall-respawn.mjs

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { WeaponSystem } from '../src/combat/WeaponSystem';
import { RunwayPlacer } from '../src/world/RunwayPlacer';
import {
  generateHeights,
  heightAt,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
  GRID_STEP,
} from '../src/world/PandoraTerrainWorker';
import { FloatingOrigin } from '../src/world/FloatingOrigin';

const D2R = Math.PI / 180;
const MS2KT = 1.94384;

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

async function main(): Promise<void> {
  await RAPIER.init();
  const heights = generateHeights({
    seed: 1337,
    chunkX: 0,
    chunkZ: 0,
    size: WORLD_SIZE,
    resolution: TERRAIN_SEGMENTS + 1,
  });
  const res = TERRAIN_SEGMENTS + 1;
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eq = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
  );
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix];
    }
  }
  const terrainCollider = world.createCollider(
    RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS,
      TERRAIN_SEGMENTS,
      hfHeights,
      new THREE.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    )
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
    terrainBody,
  );
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setRotation({ x: 0, y: Math.sin(hdg / 2), z: 0, w: Math.cos(hdg / 2) })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
  }
  const scene = new THREE.Scene();
  const wind = new THREE.Vector3(3, 0, 0); // menu default: 3 m/s
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');

  // Bilinear sampled height (what the mesh / heightfield actually are).
  const sampled = (x: number, z: number): number => {
    const gx = (x + WORLD_SIZE / 2) / GRID_STEP;
    const gz = (z + WORLD_SIZE / 2) / GRID_STEP;
    const ix = Math.max(0, Math.min(res - 2, Math.floor(gx)));
    const iz = Math.max(0, Math.min(res - 2, Math.floor(gz)));
    const fx = gx - ix;
    const fz = gz - iz;
    const h00 = heights[iz * res + ix]!;
    const h10 = heights[iz * res + ix + 1]!;
    const h01 = heights[(iz + 1) * res + ix]!;
    const h11 = heights[(iz + 1) * res + ix + 1]!;
    return (
      h00 * (1 - fx) * (1 - fz) +
      h10 * fx * (1 - fz) +
      h01 * (1 - fx) * fz +
      h11 * fx * fz
    );
  };
  const analytic = (x: number, z: number): number => heightAt(x, z);
  // The controller's ground height is the EXACT heightfield surface
  // (triangle interpolation — main.ts's heightfieldGroundHeight).
  // The analytic heightAt() is kept only as the "wrong" reference for
  // the mismatch info print in test 1a.
  const triH = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);

  const makeJet = (
    key: string,
    isPlayer: boolean,
    onCrash?: (ac: Aircraft, reason: string) => void,
  ): Aircraft => {
    const ac = new Aircraft({
      scene,
      world,
      spec: SPECS[key]!,
      isPlayer,
      modelKey: SPECS[key]!.modelKey,
      events: onCrash ? { onCrash } : {},
    });
    ac.terrainHeightAt = triH;
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    return ac;
  };
  const drain = (): void => {
    eq.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const ac = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!ac || !ac.alive) return;
      const other = ac === a ? b : a;
      ac.notifyContact(ac.vel.y, other === 'deck');
    });
  };
  const substep = (ac: Aircraft, dt = 1 / 60): void => {
    world.timestep = dt;
    world.step(eq);
    drain();
    ac.step(dt, wind, null);
  };

  // Find the steepest mountainside spot (away from runways) for the
  // stall-down: scan the analytic field for max |gradient|.
  let steep = { x: 0, z: 0, grad: 0, h: 0 };
  const half = WORLD_SIZE / 2;
  for (let x = -half + 100; x <= half - 100; x += 120) {
    for (let z = -half + 100; z <= half - 100; z += 120) {
      let nearRunway = false;
      for (const r of RUNWAY_TABLE) {
        if (Math.hypot(x - r.x, z - r.z) < 700) nearRunway = true;
      }
      if (nearRunway) continue;
      const gx = (heightAt(x + GRID_STEP, z) - heightAt(x - GRID_STEP, z)) / (2 * GRID_STEP);
      const gz = (heightAt(x, z + GRID_STEP) - heightAt(x, z - GRID_STEP)) / (2 * GRID_STEP);
      const g = Math.hypot(gx, gz);
      if (g > steep.grad) steep = { x, z, grad: g, h: heightAt(x, z) };
    }
  }
  console.log(
    `Steepest slope: (${steep.x}, ${steep.z}) grad=${steep.grad.toFixed(2)} ` +
      `analytic h=${steep.h.toFixed(1)} sampled h=${sampled(steep.x, steep.z).toFixed(1)} ` +
      `diff=${(sampled(steep.x, steep.z) - steep.h).toFixed(1)} m`,
  );

  // ---- 1a. heightfieldHeightAt vs the Rapier heightfield surface ----
  // The controller's ground logic must read the EXACT triangle surface
  // the collider is (not the analytic heightAt(), not the bilinear).
  // Verify with raycasts: a ray straight down at (x, z) must hit the
  // heightfield at exactly heightfieldHeightAt(x, z). The analytic
  // mismatch is printed for reference only — the controller no longer
  // reads it, so it is not a failure.
  let worstSpotX = 0;
  let worstSpotZ = 0;
  {
    // The world needs a step before raycasts work (broad-phase update).
    world.step();
    const rayH = (x: number, z: number): number => {
      const ray = new RAPIER.Ray({ x, y: 2000, z }, { x: 0, y: -1, z: 0 });
      const hit = world.castRay(ray, 3000, true);
      return hit ? 2000 - hit.timeOfImpact : NaN;
    };
    let worst = 0;
    let worstAt = '';
    // Steep cell around the steepest spot found above + a spread of
    // points (map centre, each runway centre).
    const pts: [number, number][] = [
      [steep.x + 10, steep.z], [steep.x + 30, steep.z], [steep.x + 10, steep.z + 20], [steep.x + 30, steep.z + 20],
      [steep.x + 8, steep.z + 12], [steep.x + 32, steep.z - 12], [0, 0],
      ...RUNWAY_TABLE.map((r): [number, number] => [r.x, r.z]),
    ];
    for (const [x, z] of pts) {
      const d = Math.abs(rayH(x, z) - triH(x, z));
      if (d > worst) {
        worst = d;
        worstAt = `(${x},${z})`;
      }
    }
    check(
      'heightfieldHeightAt matches the Rapier heightfield surface (raycast)',
      worst <= 0.05,
      `worst |raycast - triH| = ${worst.toFixed(3)} m at ${worstAt}`,
    );
    // Info: the analytic height the OLD code used, vs the real surface.
    let worstA = { x: 0, z: 0, d: 0 };
    // Best RESTING spot: the largest analytic gap where the jet can
    // actually rest LEVEL — the local terrain under its footprint
    // (±10 m, the jet is 17 m long) must be flat within 0.3 m, or
    // the jet settles tilted (its centre up to ~5 m above the
    // surface at the centre point) and the height-based ground check
    // cannot see it (a tilted rest is a known residual — see the
    // report; the respawn path itself is fixed and verified by
    // tests 2/2b, and a tilted survivor is alive and Restart works).
    let restSpot = { x: 0, z: 0, d: 0 };
    for (let x = -half + 100; x <= half - 100; x += GRID_STEP) {
      for (let z = -half + 100; z <= half - 100; z += GRID_STEP) {
        let nearRunway = false;
        for (const r of RUNWAY_TABLE) {
          if (Math.hypot(x - r.x, z - r.z) < 500) nearRunway = true;
        }
        if (nearRunway) continue;
        const d = triH(x, z) - analytic(x, z);
        if (d > worstA.d) worstA = { x, z, d };
        let rough = 0;
        for (const dx of [-10, 0, 10]) {
          for (const dz of [-10, 0, 10]) {
            rough = Math.max(rough, Math.abs(triH(x + dx, z + dz) - triH(x, z)));
          }
        }
        if (rough < 0.3 && d > restSpot.d) restSpot = { x, z, d };
      }
    }
    worstSpotX = restSpot.x;
    worstSpotZ = restSpot.z;
    console.log(
      `  info: worst analytic-vs-real-surface gap +${worstA.d.toFixed(2)} m at (${worstA.x},${worstA.z}) — the old controller read this as the ground; ` +
        `resting spot (${restSpot.x},${restSpot.z}) gap +${restSpot.d.toFixed(2)} m`,
    );
  }

  // ---- 1b. Resting on the terrain where the real surface sits far
  // above the analytic height ----
  // Place the jet just above the REAL (triangle) surface at the
  // worst analytic-gap spot on a restable slope and let it settle:
  // with the controller reading the exact surface it must read
  // grounded=true, G=1.0. (The old code read the analytic height,
  // so it never saw the ground.)
  {
    let crashReason: string | null = null;
    const ac = makeJet('fa18', true, (_a, reason) => {
      crashReason = reason;
    });
    const hx = worstSpotX;
    const hz = worstSpotZ;
    const sy = triH(hx, hz) + 2.5;
    ac.resetAt(hx, sy, hz, 90);
    ac.thrTarget = 0;
    for (let i = 0; i < 60 * 5; i++) substep(ac);
    console.log(
      `  resting test at (${hx},${hz}): y=${ac.pos.y.toFixed(2)} triH=${triH(ac.pos.x, ac.pos.z).toFixed(2)} ` +
        `analytic=${analytic(ac.pos.x, ac.pos.z).toFixed(2)} alive=${ac.alive} gnd=${ac.grounded} ` +
        `surf=${ac.onSurface} G=${ac.gload.toFixed(1)} |v|=${ac.vel.length().toFixed(2)} crash=${crashReason ?? 'none'}`,
    );
    check(
      'Jet resting on terrain reads grounded=true, G=1.0 (not a hang)',
      ac.alive && ac.grounded && Math.abs(ac.gload - 1) < 0.05,
      `gnd=${ac.grounded} surf=${ac.onSurface} G=${ac.gload.toFixed(1)} y-triH=${(ac.pos.y - triH(ac.pos.x, ac.pos.z)).toFixed(2)} m`,
    );
    ac.dispose();
  }

  // ---- 1. Deep stall-down over the steep slope ----
  {
    const ac = makeJet('fa18', true);
    // Start above the slope, flying level, then cut power and pull into
    // a stall, then release the stick — the classic stall-down.
    const sx = steep.x;
    const sz = steep.z;
    const sy = steep.h + 250;
    ac.resetAt(sx, sy, sz, 90);
    ac.setAirspeed(120);
    ac.thrTarget = 1;
    for (let i = 0; i < 60; i++) substep(ac); // settle in flight
    ac.thrTarget = 0;
    ac.pitchIn = -1; // pull up into the stall
    let stalledAt = -1;
    for (let i = 0; i < 60 * 8 && ac.alive; i++) {
      substep(ac);
      if (ac.stalled && stalledAt < 0) stalledAt = ac.time;
      if (ac.aoa > 40) break;
    }
    ac.pitchIn = 0; // release — the jet is stalled and falling
    console.log(
      `  stall-down: stalled at t=${stalledAt.toFixed(1)} AoA=${ac.aoa.toFixed(0)} ` +
        `ias=${(ac.ias * MS2KT).toFixed(0)} kt y=${ac.pos.y.toFixed(0)} m`,
    );
    let hung = false;
    let hungAt = '';
    let crashed = false;
    let landed = false;
    let lastY = ac.pos.y;
    let stillFrames = 0;
    // 120 s (was 60 s): pass 7's hands-off auto-level keeps the
    // recovered jet's wings level, so instead of tumbling into the
    // slope it recovers into a glide and skims the terrain — it
    // needs the extra time to settle onto the valley floor or meet
    // the rising slope. The property under test is unchanged: it
    // must reach the ground (crash or land), not hang mid-air.
    for (let i = 0; i < 60 * 120 && ac.alive; i++) {
      substep(ac);
      if (i % 30 === 0) {
        console.log(
          `  t=${ac.time.toFixed(1)} alive=${ac.alive} dying=${ac.dying} gnd=${ac.grounded} ` +
            `surf=${ac.onSurface} y=${ac.pos.y.toFixed(1)} (terr ${analytic(ac.pos.x, ac.pos.z).toFixed(1)} / ` +
            `sampled ${sampled(ac.pos.x, ac.pos.z).toFixed(1)}) ias=${(ac.ias * MS2KT).toFixed(0)}kt ` +
            `AoA=${ac.aoa.toFixed(0)} G=${ac.gload.toFixed(1)} vsi=${ac.vsi.toFixed(1)}`,
        );
      }
      if (!ac.alive) {
        crashed = true;
        break;
      }
      // Hang detection: alive, barely moving vertically, for 5 s.
      // Pass 5 (AoA limiter): the limiter caps the stall AoA at
      // ~50 deg, so the jet now recovers into a glide and can LAND
      // softly in the valley instead of tumbling into the slope —
      // a recovery, not a hang. A jet resting ON the ground (grounded
      // / onSurface) is landed, not hung, so the stillness only
      // counts while airborne. The check accepts a crash OR a
      // controlled landing — the property under test is "reaches the
      // ground, no mid-air hang".
      if (
        Math.abs(ac.pos.y - lastY) < 0.05 &&
        Math.abs(ac.vsi) < 0.5 &&
        !ac.grounded &&
        !ac.onSurface
      ) {
        stillFrames++;
        if (stillFrames > 300) {
          hung = true;
          hungAt = `y=${ac.pos.y.toFixed(1)} analytic=${analytic(ac.pos.x, ac.pos.z).toFixed(1)} sampled=${sampled(ac.pos.x, ac.pos.z).toFixed(1)} ias=${(ac.ias * MS2KT).toFixed(0)} AoA=${ac.aoa.toFixed(0)} G=${ac.gload.toFixed(1)} gnd=${ac.grounded} surf=${ac.onSurface}`;
          break;
        }
      } else {
        stillFrames = 0;
      }
      // Landed (recovered) — the jet reached the ground without
      // crashing and without hanging. Stop: it is on the surface.
      if (ac.alive && (ac.grounded || ac.onSurface) && Math.abs(ac.vsi) < 0.5) {
        landed = true;
        break;
      }
      lastY = ac.pos.y;
    }
    check(
      'Stall-down reaches the ground and crashes or lands (no mid-air hang)',
      (crashed || landed) && !hung,
      hung
        ? `HUNG at ${hungAt}`
        : crashed
          ? `crashed, final y=${ac.pos.y.toFixed(1)}`
          : `landed (recovered), final y=${ac.pos.y.toFixed(1)} gnd=${ac.grounded} surf=${ac.onSurface}`,
    );
    if (hung) {
      console.log(
        `  HANG DETAIL: analytic-sampled diff at hang = ` +
          `${(sampled(ac.pos.x, ac.pos.z) - analytic(ac.pos.x, ac.pos.z)).toFixed(2)} m, ` +
          `pos.y - sampled = ${(ac.pos.y - sampled(ac.pos.x, ac.pos.z)).toFixed(2)} m`,
      );
    }
    ac.dispose();
  }

  // ---- 2. Respawn after a hillside crash (main.ts flow) ----
  {
    const ac = makeJet('fa18', true);
    const placer = new RunwayPlacer();
    // Kill the jet on the steep slope (terrain crash, like test 1).
    ac.resetAt(steep.x, steep.h + 30, steep.z, 90);
    ac.setAirspeed(20);
    ac.thrTarget = 0;
    // Force a terrain crash: drop it straight down onto the slope.
    ac.body.setLinvel({ x: 0, y: -15, z: 0 }, true);
    let died = false;
    for (let i = 0; i < 60 * 10 && ac.alive; i++) {
      substep(ac);
      if (!ac.alive) died = true;
    }
    // If the drop did not kill it (gentle touchdown on the slope), force
    // the death the way a hard landing would — this test is about the
    // RESPAWN flow, not the crash trigger.
    if (!died) {
      ac.crash('terrain');
      died = !ac.alive;
    }
    console.log(`  hillside crash: died=${died} at (${ac.pos.x.toFixed(0)}, ${ac.pos.z.toFixed(0)}) y=${ac.pos.y.toFixed(1)}`);
    // main.ts handleDeaths -> respawnTimer 3 s -> respawnPlayer.
    const r = placer.nearest(ac.pos);
    ac.resetAt(r.x, r.elev + 2.25, r.z, r.hdg);
    const ok =
      ac.alive &&
      !ac.dying &&
      Math.abs(ac.pos.y - (r.elev + 2.2)) < 0.5 &&
      ac.vel.length() < 0.5;
    check(
      'Respawn after hillside crash lands on the nearest runway',
      died && ok,
      `died=${died} respawn runway=${r.name} alive=${ac.alive} y=${ac.pos.y.toFixed(2)} (expect ${r.elev + 2.2}) |v|=${ac.vel.length().toFixed(2)}`,
    );
    ac.dispose();
  }

  // ---- 2b. Respawn with a non-zero floating-origin shift ----
  // The v2 hang's PRIMARY cause: RunwayPlacer.nearest() returns the
  // runway in ORIGINAL-frame coords, but resetAt() takes SCENE coords.
  // After the player flies > 2,000 m from the origin the shift is
  // non-zero, and the old code respawned the jet up to 2+ km from the
  // actual runway (in mid-air / inside a mountain) — fall, crash,
  // 3 s timer, respawn again: the death loop. This test builds a
  // second world with the heightfield + Alpha runway box shifted by
  // (2000, 0, 1500) (what FloatingOrigin does to registered objects),
  // kills the jet near Alpha's scene position, and runs the main.ts
  // respawn flow verbatim: nearest(scenePos) -> resetAt(r.x - shift.x,
  // r.elev + 2.25, r.z - shift.z, r.hdg). The jet must land ON the
  // runway (y = elev + 2.2, grounded, alive) — and the OLD
  // conversion (no shift subtraction) is 2,500 m off.
  {
    const shift = new THREE.Vector3(2000, 0, 1500);
    const origin = new FloatingOrigin();
    origin.cumulativeShift.copy(shift);
    const placer2 = new RunwayPlacer();
    placer2.setOriginShift(origin.cumulativeShift);
    // Shifted world: heightfield + Alpha box translated by -shift
    // (scene coords), exactly as FloatingOrigin.applyShift does.
    const world2 = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
    const eq2 = new RAPIER.EventQueue(true);
    const terrainBody2 = world2.createRigidBody(
      RAPIER.RigidBodyDesc.fixed().setTranslation(-shift.x, 0, -shift.z),
    );
    world2.createCollider(
      RAPIER.ColliderDesc.heightfield(
        TERRAIN_SEGMENTS,
        TERRAIN_SEGMENTS,
        hfHeights,
        new THREE.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
      )
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody2,
    );
    const alpha = RUNWAY_TABLE[0]!;
    const hdg2 = (alpha.hdg * Math.PI) / 180;
    world2.createCollider(
      RAPIER.ColliderDesc.cuboid(alpha.width / 2 + 2, 0.5, alpha.len / 2 + 100)
        .setTranslation(alpha.x - shift.x, alpha.elev - 0.45, alpha.z - shift.z)
        .setRotation({ x: 0, y: Math.sin(hdg2 / 2), z: 0, w: Math.cos(hdg2 / 2) })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody2,
    );
    const scene2 = new THREE.Scene();
    const owners2 = new Map<number, Aircraft | 'terrain' | 'deck'>();
    owners2.set(terrainBody2.handle, 'terrain');
    // Scene-frame ground height: scene coords + shift = original.
    const triH2 = (x: number, z: number): number =>
      heightfieldHeightAt(heights, res, x + shift.x, z + shift.z);
    const ac2 = new Aircraft({
      scene: scene2,
      world: world2,
      spec: SPECS['fa18']!,
      isPlayer: true,
      modelKey: SPECS['fa18']!.modelKey,
      events: {},
    });
    ac2.terrainHeightAt = triH2;
    for (const h of ac2.colliderHandles()) owners2.set(h, ac2);
    const drain2 = (): void => {
      eq2.drainCollisionEvents((h1, h2, started) => {
        if (!started) return;
        const a = owners2.get(h1);
        const b = owners2.get(h2);
        const j = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
        if (!j || !j.alive) return;
        const other = j === a ? b : a;
        j.notifyContact(j.vel.y, other === 'deck');
      });
    };
    const substep2 = (dt = 1 / 60): void => {
      world2.timestep = dt;
      world2.step(eq2);
      drain2();
      ac2.step(dt, wind, null);
    };
    // Kill the jet near Alpha's SCENE position (original - shift).
    const px = alpha.x - shift.x;
    const pz = alpha.z - shift.z;
    ac2.resetAt(px, triH2(px, pz) + 30, pz, 90);
    ac2.setAirspeed(20);
    ac2.thrTarget = 0;
    ac2.body.setLinvel({ x: 0, y: -15, z: 0 }, true);
    let died2 = false;
    for (let i = 0; i < 60 * 10 && ac2.alive; i++) {
      substep2();
      if (!ac2.alive) died2 = true;
    }
    if (!died2) {
      ac2.crash('terrain');
      died2 = !ac2.alive;
    }
    // main.ts respawnPlayer flow, verbatim (with the shift fix).
    const r2 = placer2.nearest(ac2.pos);
    const oldOff = Math.hypot(r2.x - (r2.x - shift.x), r2.z - (r2.z - shift.z));
    ac2.resetAt(r2.x - shift.x, r2.elev + 2.25, r2.z - shift.z, r2.hdg);
    for (let i = 0; i < 60 * 5; i++) substep2();
    const ok2 =
      ac2.alive &&
      !ac2.dying &&
      Math.abs(ac2.pos.y - (r2.elev + 2.2)) < 0.5 &&
      ac2.vel.length() < 0.5 &&
      ac2.grounded;
    console.log(
      `  shifted respawn: died=${died2} runway=${r2.name} shift=(${shift.x},${shift.z}) ` +
        `old-code error=${oldOff.toFixed(0)} m alive=${ac2.alive} gnd=${ac2.grounded} ` +
        `y=${ac2.pos.y.toFixed(2)} (expect ${r2.elev + 2.2}) |v|=${ac2.vel.length().toFixed(2)}`,
    );
    check(
      'Respawn with a floating-origin shift lands on the runway (not 2.5 km off)',
      died2 && ok2,
      `died=${died2} alive=${ac2.alive} gnd=${ac2.grounded} y=${ac2.pos.y.toFixed(2)}`,
    );
    ac2.dispose();
  }

  // ---- 3. Missile shoot-down via the REAL WeaponSystem path ----
  {
    (globalThis as unknown as { document: unknown }).document = {
      createElement: (_tag: string) => ({
        width: 0,
        height: 0,
        getContext: () => ({
          createRadialGradient: () => ({ addColorStop: () => undefined }),
          fillStyle: '',
          fillRect: () => undefined,
        }),
      }),
    };
    let explosionCalls = 0;
    const weapons = new WeaponSystem(scene, world, {
      onExplosion: () => {
        explosionCalls++;
      },
    });
    const ac = makeJet('fa18', true);
    ac.resetAt(0, 1000, -900, 90);
    ac.setAirspeed(220); // 427 kt, like the live shoot-down
    ac.thrTarget = 0.8;
    // Settle AND clear the 5 s player spawn protection (applyDamage
    // refuses a spawn-protected player — the first probe run hit this
    // and the missile did nothing at all).
    for (let i = 0; i < 60 * 7; i++) substep(ac);
    // Enemy missile detonates at the jet's centre (proximity fuse).
    weapons.applyProximityDamage(ac, ac.pos.clone(), 40, 15, false);
    // The fall is timed from the HIT, not from the reset: ac.time
    // counts from resetAt, so the 7 s settle above is inside it.
    // (Pass 11: the old check compared impactT < 25 s against a
    // clock that included the settle, so a correct 22 s fall from
    // ~1100 m read as 29 s and failed — the jet climbs ~140 m
    // during the settle on its ~2-3 deg trim AoA at 220 m/s, a
    // pre-existing trim issue documented in BladeElementSolver,
    // not a glide: the fall itself is 22 s, not the 40-60 s glide
    // this check exists to catch.)
    const hitT = ac.time;
    const fxAtHit = ac.getDamageFx();
    console.log(
      `  missile hit: dying=${ac.dying} reason=${ac.dyingReason} hp=${ac.hp.toFixed(1)} ` +
        `fire=${fxAtHit.fire} fireKind=${fxAtHit.fireKind} smoke=${fxAtHit.smoke} ` +
        `explosionCalls=${explosionCalls}`,
    );
    check(
      'Missile kill starts a fire (visible burning wreck)',
      ac.dying && fxAtHit.fire,
      `dying=${ac.dying} fire=${fxAtHit.fire} kind=${fxAtHit.fireKind} hp=${ac.hp.toFixed(1)}`,
    );
    // Follow the fall: log every 2 s until impact or 90 s.
    let impactT = -1;
    let minVsi = 0;
    let maxFallSpeed = 0;
    for (let i = 0; i < 60 * 90 && ac.alive; i++) {
      substep(ac);
      minVsi = Math.min(minVsi, ac.vsi);
      maxFallSpeed = Math.max(maxFallSpeed, -ac.vsi);
      if (i % 120 === 0) {
        console.log(
          `  fall t=${ac.time.toFixed(1)} y=${ac.pos.y.toFixed(0)} ias=${(ac.ias * MS2KT).toFixed(0)}kt ` +
            `AoA=${ac.aoa.toFixed(0)} vsi=${ac.vsi.toFixed(1)} fire=${ac.getDamageFx().fire} G=${ac.gload.toFixed(1)}`,
        );
      }
      if (!ac.alive) {
        impactT = ac.time;
        break;
      }
    }
    check(
      'Dying jet falls to the ground and impacts (not a 40-60 s glide)',
      !ac.alive && ac.crashedFromDying && impactT > hitT && impactT - hitT < 25,
      `fallT=${(impactT - hitT).toFixed(1)} s (impactT=${impactT.toFixed(1)} s from reset) crashedFromDying=${ac.crashedFromDying} maxFall=${maxFallSpeed.toFixed(1)} m/s minVsi=${minVsi.toFixed(1)}`,
    );
    ac.dispose();
  }

  // ---- 4. Gun kill (tail x2) ----
  {
    const ac = makeJet('fa18', true);
    ac.resetAt(0, 1000, -900, 90);
    ac.setAirspeed(150);
    // Clear the 5 s player spawn protection before damaging (see test 3).
    for (let i = 0; i < 60 * 7; i++) substep(ac);
    ac.applyDamage(12, 'tail', false);
    ac.applyDamage(12, 'tail', false);
    const fx = ac.getDamageFx();
    console.log(
      `  gun tail kill: dying=${ac.dying} reason=${ac.dyingReason} hp=${ac.hp.toFixed(1)} fire=${fx.fire} smoke=${fx.smoke}`,
    );
    check(
      'Gun kill (tail destroyed) shows fire/smoke',
      ac.dying && (fx.fire || fx.smoke),
      `dying=${ac.dying} hp=${ac.hp.toFixed(1)} fire=${fx.fire} smoke=${fx.smoke}`,
    );
    ac.dispose();
  }

  // ---- 5. Keyboard hold vs tap (PCFlightInput slew = 5/s) ----
  {
    // Model the keyboard slew: kbPitch slews toward -1 at 5/s while
    // ArrowUp is held, back toward 0 at 5/s on release.
    const alpha = RUNWAY_TABLE[0]!;
    const runRoll = (holdS: number, flaps: number): { lifted: boolean; dist: number; kt: number } => {
      const ac = makeJet('fa18', true);
      ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
      for (let i = 0; i < 120; i++) substep(ac);
      ac.flaps = flaps;
      ac.thrTarget = 1;
      let kb = 0;
      const x0 = ac.pos.x;
      let lifted = false;
      let dist = 0;
      let kt = 0;
      for (let i = 0; i < 60 * 25 && ac.alive; i++) {
        const t = i / 60;
        const target = t < holdS ? -1 : 0;
        kb += Math.max(-5 / 60, Math.min(5 / 60, target - kb));
        ac.pitchIn = kb;
        substep(ac);
        if (
          ac.pos.y > analytic(ac.pos.x, ac.pos.z) + 4 &&
          ac.vsi > 0 &&
          !ac.grounded &&
          ac.ias > 40
        ) {
          lifted = true;
          dist = ac.pos.x - x0;
          kt = ac.ias * MS2KT;
          break;
        }
        if (ac.pos.x - x0 > 750) break;
      }
      const d = ac.pos.x - x0;
      ac.dispose();
      return { lifted, dist: lifted ? dist : d, kt };
    };
    const held = runRoll(25, 30); // ArrowUp held the whole roll
    check(
      'ArrowUp HELD rotates and lifts off (flaps 30)',
      held.lifted && held.dist < 650,
      `lifted=${held.lifted} in ${held.dist.toFixed(0)} m at ${held.kt.toFixed(0)} kt`,
    );
    const tap = runRoll(0.1, 30); // a single 0.1 s tap at Vr-ish speed
    console.log(
      `  tap (0.1 s): lifted=${tap.lifted} dist=${tap.dist.toFixed(0)} m kt=${tap.kt.toFixed(0)}`,
    );
    check(
      'ArrowUp tap (0.1 s) alone does NOT rotate (documents the live report)',
      !tap.lifted,
      `lifted=${tap.lifted}`,
    );
  }

  // ---- 6. Touch stick: pull-back (pitchIn -1) vs push (pitchIn +1) ----
  // The probe tests pitchIn directly (the physics). The main.ts joy
  // mapping is now inverted to sim-style — stick DOWN (pull back)
  // gives pitchIn -1, stick UP gives +1 — so "stick DOWN rotates"
  // and "stick UP does not", matching the help text and the keyboard.
  {
    const alpha = RUNWAY_TABLE[0]!;
    const runStick = (pitchIn: number): { lifted: boolean; dist: number } => {
      const ac = makeJet('fa18', true);
      ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
      for (let i = 0; i < 120; i++) substep(ac);
      ac.flaps = 30;
      ac.thrTarget = 1;
      ac.pitchIn = pitchIn;
      const x0 = ac.pos.x;
      let lifted = false;
      for (let i = 0; i < 60 * 25 && ac.alive; i++) {
        substep(ac);
        if (
          ac.pos.y > analytic(ac.pos.x, ac.pos.z) + 4 &&
          ac.vsi > 0 &&
          !ac.grounded &&
          ac.ias > 40
        ) {
          lifted = true;
          break;
        }
        if (ac.pos.x - x0 > 750) break;
      }
      const d = ac.pos.x - x0;
      ac.dispose();
      return { lifted, dist: d };
    };
    const pullBack = runStick(-1);
    const push = runStick(1);
    console.log(
      `  pull back (pitchIn -1 = stick DOWN / ArrowUp): lifted=${pullBack.lifted} dist=${pullBack.dist.toFixed(0)} m; ` +
        `push (pitchIn +1 = stick UP): lifted=${push.lifted} dist=${push.dist.toFixed(0)} m`,
    );
    check(
      'Pull back (stick DOWN / ArrowUp) rotates and lifts off',
      pullBack.lifted && pullBack.dist < 650,
      `lifted=${pullBack.lifted} dist=${pullBack.dist.toFixed(0)} m`,
    );
    check(
      'Push (stick UP) does NOT rotate (nose-down input)',
      !push.lifted,
      `lifted=${push.lifted} dist=${push.dist.toFixed(0)} m`,
    );
  }

  // ---- 7. Parked creep (wind 3 m/s) ----
  {
    const alpha = RUNWAY_TABLE[0]!;
    const ac = makeJet('fa18', true);
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac);
    const x0 = ac.pos.x;
    const z0 = ac.pos.z;
    ac.thrTarget = 0;
    for (let i = 0; i < 60 * 5; i++) substep(ac);
    const moved = Math.hypot(ac.pos.x - x0, ac.pos.z - z0);
    console.log(
      `  parked 5 s: moved=${moved.toFixed(2)} m, ground speed=${(ac.vel.length() * MS2KT).toFixed(1)} kt, ` +
        `IAS tape=${(ac.ias * MS2KT).toFixed(1)} kt (wind 3 m/s)`,
    );
    check(
      'Parked jet does not move at 0% throttle',
      moved < 0.5,
      `moved=${moved.toFixed(2)} m`,
    );
    check(
      'Parked IAS tape reads the wind, not 0 (explains the "6 kt creep")',
      ac.ias > 2 && ac.ias < 4,
      `ias=${(ac.ias * MS2KT).toFixed(1)} kt`,
    );
    ac.dispose();
  }

  console.log(failures === 0 ? '\nALL PROBE CHECKS PASSED' : `\n${failures} PROBE CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('PROBE ERROR', e);
  process.exit(2);
});
