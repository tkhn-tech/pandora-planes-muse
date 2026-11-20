// probe-terrain-wiring.ts — pass 11 (infinite terrain, main.ts wiring).
//
// The TerrainStreamer was built and verified by its subagent driving
// the streamer directly (probe-infinite-flight: 76 km, 36 shifts, 0
// holes), but it was barred from main.ts — so the wiring itself is
// verified here:
//  1. main.ts constructs the streamer after worldGen.build() with
//     collider callbacks into colliderOwners, swaps
//     heightfieldGroundHeight to terrainStreamer.heightAt, captures
//     the origin shift in animate() and moves the CAMERA with it
//     (the glitch-back fix — the camera was never registered with the
//     FloatingOrigin, so the chase lerp left it ~2 km behind the jet
//     for ~0.6 s at every shift), and calls terrainStreamer.update()
//     once per frame after the shift check.
//  2. WeaponSystem gets origin: floatingOrigin, and in-flight
//     missiles/bombs shift with the world: a live missile is launched,
//     the origin is shifted, and the missile's body AND mesh must move
//     together by the shift (so the missile-target distance is
//     unchanged and the missile keeps closing), and the same for a
//     dropped bomb. Without the registration the ordnance jumps ~2 km
//     relative to its (registered) target and flies wrong.
//
// Usage: node_modules/.bin/esbuild scripts/probe-terrain-wiring.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-terrain-wiring.mjs
//   && node /tmp/probe-terrain-wiring.mjs

if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
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
}

import * as THREE from 'three';
import * as fs from 'node:fs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { WeaponSystem } from '../src/combat/WeaponSystem';
import { FloatingOrigin } from '../src/world/FloatingOrigin';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

async function main(): Promise<void> {
  await RAPIER.init();

  // ---- Wiring in main.ts / WeaponSystem.ts (source checks).
  let src = '';
  for (const p of ['src/main.ts', '/home/hatch/workspace/pandora-planes-vite/src/main.ts']) {
    try { src = fs.readFileSync(p, 'utf8'); break; } catch { /* try next */ }
  }
  check('main.ts readable for the wiring checks', src.length > 0, `${src.length} bytes`);
  check(
    'main.ts constructs the TerrainStreamer after worldGen.build()',
    src.includes('new TerrainStreamer(scene, rapierWorld, floatingOrigin') &&
      src.indexOf('new TerrainStreamer(') > src.indexOf('await worldGen.build()'),
    'streamer constructed with collider callbacks',
  );
  check(
    'main.ts ground height reads through the streamer',
    src.includes('terrainStreamer.heightAt(x, z)'),
    'heightfieldGroundHeight = terrainStreamer.heightAt',
  );
  check(
    'main.ts captures the origin shift and moves the camera with it',
    src.includes('const shift = worldGen.shiftOriginIfNeeded(') &&
      src.includes('camera.position.x -= shift.x') &&
      src.includes('camera.position.z -= shift.z'),
    'camera follows the shift (glitch-back fix)',
  );
  check(
    'main.ts streams terrain once per frame after the shift check',
    src.includes('terrainStreamer.update(player?.pos'),
    'update() after shiftOriginIfNeeded',
  );
  check(
    'main.ts passes the origin to WeaponSystem',
    src.includes('origin: floatingOrigin,'),
    'weapons hooks include origin',
  );
  let wsrc = '';
  for (const p of ['src/combat/WeaponSystem.ts', '/home/hatch/workspace/pandora-planes-vite/src/combat/WeaponSystem.ts']) {
    try { wsrc = fs.readFileSync(p, 'utf8'); break; } catch { /* try next */ }
  }
  check(
    'WeaponSystem registers missile/bomb bodies + meshes with the origin',
    (wsrc.match(/this\.origin\?\.registerBody\(body\)/g) ?? []).length >= 2 &&
      (wsrc.match(/this\.origin\?\.registerObject\(/g) ?? []).length >= 2,
    'launchMissile + dropBomb register',
  );
  check(
    'WeaponSystem unregisters ordnance on removal',
    (wsrc.match(/this\.origin\?\.unregisterBody\(/g) ?? []).length >= 2 &&
      (wsrc.match(/this\.origin\?\.unregisterObject\(/g) ?? []).length >= 2,
    'removeMissile + removeBomb unregister',
  );

  // ---- Live ordnance shift test.
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const scene = new THREE.Scene();
  const origin = new FloatingOrigin(2000);
  const groundH = () => 0;
  const makeJet = (key: string, isPlayer: boolean): Aircraft => {
    const ac = new Aircraft({
      scene,
      world,
      spec: SPECS[key]!,
      isPlayer,
      modelKey: SPECS[key]!.modelKey,
      events: {},
    });
    ac.terrainHeightAt = groundH;
    origin.registerBody(ac.body);
    return ac;
  };
  const weapons = new WeaponSystem(scene, world, { origin });
  const eq = new RAPIER.EventQueue(true);
  const wind = new THREE.Vector3(0, 0, 0);
  // Step the world + weapons + the given aircraft (the aircraft step
  // is what re-syncs an aircraft's logical pos from its Rapier body —
  // the body is the ground truth after a shift).
  const step = (dt: number, jets: Aircraft[] = []): void => {
    world.timestep = dt;
    world.step(eq);
    for (const j of jets) j.step(dt, wind, null);
    weapons.update(dt, []);
  };
  const bodyVec = (b: RAPIER.RigidBody): THREE.Vector3 => {
    const t = b.translation();
    return new THREE.Vector3(t.x, t.y, t.z);
  };

  // Missile: shooter + target flying +X together, past the 2 km
  // threshold, so the origin shift fires.
  const shooter = makeJet('fa18', true);
  const target = makeJet('f22', false);
  shooter.resetAt(2500, 600, 0, 90);
  target.resetAt(3000, 600, 0, 90);
  shooter.setAirspeed(150);
  target.setAirspeed(150);
  shooter.msl = 6;
  for (let i = 0; i < 10; i++) step(1 / 60, [shooter, target]);
  weapons.launchMissile(shooter, target);
  for (let i = 0; i < 30; i++) step(1 / 60, [shooter, target]);
  // Reach into the private missile list for the measurement.
  const missiles = (weapons as unknown as { missiles: Array<{ mesh: THREE.Mesh; body: RAPIER.RigidBody }> }).missiles;
  check('missile launched and live', missiles.length === 1, `missiles=${missiles.length}`);
  if (missiles.length === 1) {
    const m = missiles[0]!;
    const bodyBefore = m.body.translation();
    const meshBefore = m.mesh.position.clone();
    // Distances measured body-to-body (ground truth): the logical
    // target.pos only re-syncs on the aircraft's next step.
    const distBefore = m.mesh.position.distanceTo(bodyVec(target.body));
    const shift = origin.shiftIfNeeded(shooter.pos);
    check('origin shift fires past 2 km', shift !== null, `shift=${shift ? `(${shift.x.toFixed(0)}, ${shift.z.toFixed(0)})` : 'null'}`);
    if (shift) {
      origin.applyShift(shift);
      // Measure immediately — no step in between, so no flight
      // contaminates the shift measurement.
      const bodyAfter = m.body.translation();
      const meshAfter = m.mesh.position.clone();
      check(
        'missile body shifts with the world',
        Math.abs((bodyAfter.x - bodyBefore.x) + shift.x) < 0.5,
        `body.x ${bodyBefore.x.toFixed(1)} -> ${bodyAfter.x.toFixed(1)} (shift ${shift.x.toFixed(0)})`,
      );
      check(
        'missile mesh shifts with the body (no desync)',
        Math.abs((meshAfter.x - meshBefore.x) + shift.x) < 0.5 &&
          Math.abs(meshAfter.x - bodyAfter.x) < 1.0,
        `mesh.x ${meshBefore.x.toFixed(1)} -> ${meshAfter.x.toFixed(1)}`,
      );
      const distAfter = m.mesh.position.distanceTo(bodyVec(target.body));
      check(
        'missile-target distance unchanged by the shift',
        Math.abs(distAfter - distBefore) < 1.0,
        `dist ${distBefore.toFixed(1)} -> ${distAfter.toFixed(1)} m`,
      );
      // The missile still closes on its (shifted) target.
      for (let i = 0; i < 120; i++) step(1 / 60, [shooter, target]);
      const distLater = missiles.length === 1
        ? missiles[0]!.mesh.position.distanceTo(bodyVec(target.body))
        : -1;
      check(
        'missile still closes on the target after the shift',
        distLater >= 0 && distLater < distAfter,
        `dist ${distAfter.toFixed(1)} -> ${distLater.toFixed(1)} m over 2 s`,
      );
    }
  }
  // Unregister before dispose, like main.ts clearAircraftList — a
  // disposed body left registered traps the next applyShift (Rapier
  // "unreachable" on a removed body).
  origin.unregisterBody(shooter.body);
  origin.unregisterBody(target.body);
  shooter.dispose();
  target.dispose();

  // Bomb: same shift test through the second creation site.
  const bomber = makeJet('f15e', true);
  bomber.resetAt(2500, 600, 0, 90);
  bomber.setAirspeed(150);
  bomber.bombs = 4;
  for (let i = 0; i < 10; i++) step(1 / 60, [bomber]);
  weapons.dropBomb(bomber);
  for (let i = 0; i < 10; i++) step(1 / 60, [bomber]);
  const bombs = (weapons as unknown as { bombs: Array<{ mesh: THREE.Group; body: RAPIER.RigidBody }> }).bombs;
  check('bomb dropped and live', bombs.length === 1, `bombs=${bombs.length}`);
  if (bombs.length === 1) {
    const b = bombs[0]!;
    const bodyBefore = b.body.translation();
    const meshBefore = b.mesh.position.clone();
    const shift = origin.shiftIfNeeded(bomber.pos);
    check('origin shift fires for the bomb run', shift !== null, `shift=${shift ? shift.x.toFixed(0) : 'null'}`);
    if (shift) {
      origin.applyShift(shift);
      const bodyAfter = b.body.translation();
      const meshAfter = b.mesh.position.clone();
      check(
        'bomb body + mesh shift together with the world',
        Math.abs((bodyAfter.x - bodyBefore.x) + shift.x) < 0.5 &&
          Math.abs((meshAfter.x - meshBefore.x) + shift.x) < 0.5,
        `body.x ${bodyBefore.x.toFixed(1)} -> ${bodyAfter.x.toFixed(1)}, mesh.x ${meshBefore.x.toFixed(1)} -> ${meshAfter.x.toFixed(1)}`,
      );
    }
  }
  origin.unregisterBody(bomber.body);
  bomber.dispose();

  console.log(failures === 0 ? '\nALL TERRAIN-WIRING CHECKS PASSED' : `\n${failures} TERRAIN-WIRING CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('PROBE ERROR', e);
  process.exit(2);
});
