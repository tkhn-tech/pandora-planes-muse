// probe-ai-hover.ts — pass 6, item 11: the live test had 3 Veteran
// enemies ignore a hovering F-35A for 2.5 min ("NO LOCK", never fired)
// while the AI flights died on terrain after 20-25 s. This driver puts
// 2 Veteran enemies against a hovering player (F-35A, VEC 90, 350 m)
// and checks the AI engages, fires, and survives.
// Usage: esbuild scripts/probe-ai-hover.ts --bundle --platform=node
//   --format=esm --outfile=/tmp/probe-ai-hover.mjs && node /tmp/probe-ai-hover.mjs
import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { AircraftAIController } from '../src/entities/AircraftAIController';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import {
  generateHeights,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

const WIND = new THREE.Vector3(3, 0, 0);
let failures = 0;

// Deterministic Math.random (pass 12, coordinator): the AI controller
// and WeaponSystem use Math.random() directly (bomb-entry rolls, evade
// choices, gun error, flare/ignition rolls), so two runs of the same
// code diverged wildly — the enemy survived 90 s, died at t=8.0 s, or
// died at t=39.0 s — from unseeded randomness alone. probe-ai-live
// already seeds for exactly this reason ("a probe must be
// reproducible"). Seed it from a SEPARATE LCG so the terrain seed
// above is untouched. This is probe scaffolding only — the live game
// keeps the real Math.random.
const rngMathState = { s: 987654321 };
Math.random = (): number => {
  rngMathState.s = (rngMathState.s * 1103515245 + 12345) & 0x7fffffff;
  return rngMathState.s / 0x7fffffff;
};

function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

async function main(): Promise<void> {
  await RAPIER.init();
  const heights = generateHeights({ seed: 1337, chunkX: 0, chunkZ: 0, size: WORLD_SIZE, resolution: TERRAIN_SEGMENTS + 1 });
  const res = TERRAIN_SEGMENTS + 1;
  const heightAt = (x: number, z: number): number => heightfieldHeightAt(heights, res, x, z);
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0));
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) for (let ix = 0; ix < res; ix++) hfHeights[ix * res + iz] = heights[iz * res + ix];
  const terrainCollider = world.createCollider(
    RAPIER.ColliderDesc.heightfield(TERRAIN_SEGMENTS, TERRAIN_SEGMENTS, hfHeights, new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE))
      .setFriction(0).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min).setRestitution(0)
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
        .setFriction(0).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min).setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
  }
  const scene = new THREE.Scene();
  const origin = new FloatingOrigin();
  const owners = new Map<number, Aircraft | 'terrain'>();
  owners.set(terrainCollider.handle, 'terrain');
  const alpha = RUNWAY_TABLE[0]!;

  // Hovering player: F-35A at 350 m over Alpha, VEC 90, hover throttle.
  const player = new Aircraft({ scene, world, spec: SPECS['f35a']!, isPlayer: true, modelKey: SPECS['f35a']!.modelKey });
  player.terrainHeightAt = heightAt;
  origin.registerBody(player.body);
  for (const h of player.colliderHandles()) owners.set(h, player);
  player.resetAt(alpha.x, 350, alpha.z, alpha.hdg);
  player.vector = 90;
  player.throttle = 0.94;
  player.thrTarget = 0.94;

  // 2 Veteran enemies, 1500 m away at 500 m.
  const enemies: Aircraft[] = [];
  const ais: AircraftAIController[] = [];
  const carrierStub = { deckHeightAt: () => -Infinity } as unknown as Parameters<AircraftAIController['update']>[2];
  for (let i = 0; i < 2; i++) {
    const e = new Aircraft({ scene, world, spec: SPECS['mig29']!, isPlayer: false, modelKey: SPECS['mig29']!.modelKey });
    e.terrainHeightAt = heightAt;
    origin.registerBody(e.body);
    for (const h of e.colliderHandles()) owners.set(h, e);
    const a = (i * 2 * Math.PI) / 2 + 0.6;
    e.resetAt(alpha.x + Math.cos(a) * 1500, 500, alpha.z + Math.sin(a) * 1500, (a * 180) / Math.PI);
    e.setAirspeed(150);
    e.gear = false;
    e.grounded = false;
    e.thrTarget = 0.8;
    const ai = new AircraftAIController(e, 'veteran', 'enemy', {
      groundHeight: heightAt,
      originShift: origin.cumulativeShift,
    });
    origin.registerVector(ai.navTarget);
    ais.push(ai);
    enemies.push(e);
  }
  const drain = (): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, false);
    });
  };

  let everEngage = false;
  let everFire = false;
  let minDist = Infinity;
  let enemyDeathAt = -1;
  for (let i = 0; i < 5400; i++) {
    const t = i / 60;
    world.timestep = 1 / 60;
    world.step(eventQueue);
    drain();
    player.step(1 / 60, WIND, null);
    for (let j = 0; j < enemies.length; j++) {
      const e = enemies[j]!;
      const ai = ais[j]!;
      if (!e.alive) {
        if (enemyDeathAt < 0) enemyDeathAt = t;
        continue;
      }
      const ctrl = ai.update(1 / 60, player, carrierStub, t);
      if (ai.state === 'engage') everEngage = true;
      if (ai.fireGun || ai.fireMissile) everFire = true;
      minDist = Math.min(minDist, e.pos.distanceTo(player.pos));
      e.step(1 / 60, ctrl, WIND, heightAt, null);
    }
    if (i % 600 === 0) {
      console.log(
        `  [t=${t.toFixed(0)}s] player y=${player.pos.y.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)}kt ` +
        enemies.map((e, j) => `e${j} alive=${e.alive} state=${ais[j]!.state} d=${e.pos.distanceTo(player.pos).toFixed(0)}m y=${e.pos.y.toFixed(0)}`).join(' '),
      );
    }
    if (!player.alive) break;
  }
  check('AI engages the hovering player (state=engage)', everEngage, everEngage ? '' : 'never left patrol');
  check('AI fires at the hovering player (gun or missile)', everFire, everFire ? '' : `closest approach ${minDist.toFixed(0)} m`);
  check(
    'AI survives 90 s (no terrain death)',
    enemyDeathAt < 0,
    enemyDeathAt >= 0 ? `first enemy died at t=${enemyDeathAt.toFixed(1)}s` : 'both alive at 90 s',
  );
  check('hovering player survives 90 s', player.alive, player.alive ? `y=${player.pos.y.toFixed(0)}` : `died: ${player.lastCrashReason ?? '-'}`);

  for (const e of enemies) e.dispose();
  player.dispose();
  console.log(failures === 0 ? 'ALL AI-HOVER CHECKS PASSED' : `${failures} AI-HOVER CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
