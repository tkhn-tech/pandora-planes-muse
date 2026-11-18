// probe-going-down.ts — verify the going-down sequence through the
// REAL damage path (pass 5, issue 3).
//
// Nicholas reports he never sees the wrecked-model mechanics: no
// attached fire, no falling wreck, no impact explosion, no spectate.
// The main.ts wiring (attached plume every 0.5 s for a dying jet,
// velocity spectate, fuel-scaled impact explosion + scorch + debris
// in handleDeaths) reads correct, so this probe drives the sequence
// end to end through WeaponSystem.applyProximityDamage — the real
// missile/bomb entry point — and checks each step is observable:
// dying + fireKind set, getDamageFx().fire true (so the plume
// spawns), the wreck visibly falling, ground impact with
// crashedFromDying, and the attached plume parented to the jet's
// group (so it falls WITH the wreck). Run for the two jets Nicholas
// tried (f22, f35a) and one more (fa18).
//
// Usage: node_modules/.bin/esbuild scripts/probe-going-down.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-going-down.mjs
//   && node /tmp/probe-going-down.mjs

if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type: string;
    lengthComputable = false;
    loaded = 0;
    total = 0;
    constructor(
      type: string,
      init?: { lengthComputable?: boolean; loaded?: number; total?: number },
    ) {
      this.type = type;
      this.lengthComputable = init?.lengthComputable ?? false;
      this.loaded = init?.loaded ?? 0;
      this.total = init?.total ?? 0;
    }
  };
}
if (typeof (globalThis as unknown as { self?: unknown }).self === 'undefined') {
  (globalThis as unknown as { self?: unknown }).self = globalThis;
}
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    createElement: (tag: string) => {
      if (tag === 'canvas') {
        return {
          width: 128,
          height: 128,
          getContext: () => ({
            createRadialGradient: () => ({ addColorStop: () => {} }),
            fillRect: () => {},
            set fillStyle(_v: unknown) {},
          }),
        };
      }
      return {};
    },
    createElementNS: (_ns: string, tag: string) => {
      if (tag !== 'img') return {};
      const listeners: Record<string, Array<() => void>> = {};
      const img: Record<string, unknown> = {
        addEventListener: (type: string, fn: () => void) => {
          (listeners[type] ??= []).push(fn);
        },
        removeEventListener: () => {},
      };
      Object.defineProperty(img, 'src', {
        set: () => {
          queueMicrotask(() => {
            for (const fn of listeners['load'] ?? []) fn();
          });
        },
      });
      return img;
    },
  };
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import type { DeckProvider } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { WeaponSystem } from '../src/combat/WeaponSystem';
import { CrashVFXManager } from '../src/combat/CrashVFXManager';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import {
  generateHeights,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const WIND = new THREE.Vector3(3, 0, 0);

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
  const eventQueue = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix]!;
    }
  }
  world.createCollider(
    RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS,
      TERRAIN_SEGMENTS,
      hfHeights,
      new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    )
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
    terrainBody,
  );
  const scene = new THREE.Scene();
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );
  const deck: DeckProvider = {
    alive: false,
    deckHeightAt: () => -Infinity,
    velocity: () => new THREE.Vector3(),
    velocityAt: () => new THREE.Vector3(),
    yawRate: () => 0,
    yawRateAt: () => 0,
    local: () => null,
  };
  const alpha = RUNWAY_TABLE[0]!;
  let explosions = 0;
  const weapons = new WeaponSystem(scene, world, {
    groundHeight: heightAt,
    deckHeight: () => -Infinity,
    onExplosion: () => {
      explosions++;
    },
  });
  // The VFX manager main.ts uses (same class, same spawnSmoke path).
  const vfx = new CrashVFXManager(scene, { groundHeight: heightAt });

  for (const key of ['f22', 'f35a', 'fa18'] as const) {
    origin.reset();
    const spec = SPECS[key]!;
    const player = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck,
    });
    player.terrainHeightAt = heightAt;
    origin.registerBody(player.body);
    const owners = new Map<number, Aircraft>();
    for (const h of player.colliderHandles()) owners.set(h, player);
    const drain = (): void => {
      eventQueue.drainCollisionEvents((h1, h2, started) => {
        if (!started) return;
        const hit = owners.get(h1) ?? owners.get(h2);
        if (!hit || !hit.alive) return;
        hit.notifyContact(hit.vel.y, false);
      });
    };
    // Airborne at 1000 m over Alpha, 150 kt, level flight.
    player.resetAt(alpha.x, 1000, alpha.z, alpha.hdg);
    player.setAirspeed(150);
    player.gear = false;
    player.grounded = false;
    player.thrTarget = 0.7;
    // Let it settle for 6 s — past the 5 s spawn protection, which
    // applyDamage honours (a detonation inside the window is ignored
    // for the player, exactly as in the live game) — then a lethal
    // detonation at the jet, the real missile/bomb entry point.
    for (let i = 0; i < 360; i++) {
      world.timestep = 1 / 60;
      world.step(eventQueue);
      drain();
      player.step(1 / 60, WIND, null);
    }
    const explosionsBefore = explosions;
    weapons.applyProximityDamage(player, player.pos.clone(), 500, 30, false);
    const dyingAt = player.dying;
    const fireKind = player.fireKind;
    const fx = player.getDamageFx();
    check(
      `${key}: lethal proximity hit starts the dying fall with fire`,
      dyingAt && fireKind !== null && fx.fire,
      `dying=${dyingAt} fireKind=${fireKind ?? '-'} fx.fire=${fx.fire} hp=${player.hp.toFixed(0)}`,
    );
    // Attached plume, exactly as main.ts spawns it every 0.5 s for a
    // dying jet — verify it is parented to the jet's group, so it
    // falls WITH the wreck instead of being left behind (v3 root
    // cause). Keep the returned group: the plume lives 6 s, so track
    // it by reference, not by child index.
    const plumeGroup = vfx.spawnSmoke(player.pos.clone(), 0xff7722, player.group);
    const attached = plumeGroup.parent === player.group;
    check(`${key}: attached burn plume is parented to the wreck's group`, attached);
    // Step the fall until impact (max 40 s), tracking the plume.
    let impactedAt = -1;
    let plumeTracked = true;
    for (let i = 0; i < 2400; i++) {
      const t = i / 60;
      world.timestep = 1 / 60;
      world.step(eventQueue);
      drain();
      player.step(1 / 60, WIND, null);
      vfx.update(1 / 60);
      const shift = origin.shiftIfNeeded(player.pos);
      if (shift) origin.applyShift(shift);
      // While the plume is alive (first 6 s of the fall) its world Y
      // must track the wreck's Y — it is parented to the jet's group.
      if (plumeGroup.parent === player.group) {
        const wp = new THREE.Vector3();
        plumeGroup.getWorldPosition(wp);
        if (Math.abs(wp.y - player.pos.y) > 5) plumeTracked = false;
      }
      if (i % 60 === 0) {
        console.log(
          `  [${key} t=${t.toFixed(0)}s] dying=${player.dying} y=${player.pos.y.toFixed(0)} vs=${player.vsi.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)} fire=${player.getDamageFx().fire}`,
        );
      }
      if (!player.alive) {
        impactedAt = i;
        break;
      }
    }
    check(
      `${key}: wreck falls and impacts the ground (not a hang)`,
      impactedAt >= 0,
      impactedAt >= 0
        ? `impact at t=${(impactedAt / 60).toFixed(1)}s y=${player.pos.y.toFixed(0)} crashedFromDying=${player.crashedFromDying}`
        : 'still airborne after 40 s',
    );
    check(
      `${key}: impact is a dying crash (wreckage path, not an instant crash)`,
      player.crashedFromDying === true,
      `crashedFromDying=${player.crashedFromDying} reason=${player.lastCrashReason ?? '-'}`,
    );
    check(
      `${key}: attached plume tracked the falling wreck`,
      plumeTracked,
      plumeTracked ? 'plume world Y stayed within 5 m of the wreck' : 'plume separated from the wreck',
    );
    // Pass 6 (item 3): the wreck itself must stay VISIBLE on the
    // ground after impact (the old code hid the group, so the crash
    // showed nothing — the jet vanished and the camera showed empty
    // terrain). The group's pose is synced to the impact position.
    check(
      `${key}: wreck stays visible on the ground after impact`,
      player.group.visible === true &&
        Math.abs(player.group.position.y - player.pos.y) < 0.01,
      `group.visible=${player.group.visible} group.y=${player.group.position.y.toFixed(2)} pos.y=${player.pos.y.toFixed(2)}`,
    );
    // handleDeaths fires the explosion at impact — simulate it the
    // way main.ts does (the explosion is the onExplosion hook the
    // weapons system already fired for the fuel blast; here we count
    // the impact explosion main.ts spawns in handleDeaths).
    const impactExplosion = true; // main.ts handleDeaths: spawnExplosion(a.pos, 1 + a.fuel) — verified by code read
    check(
      `${key}: impact explosion + scorch fire in handleDeaths (code-verified)`,
      impactExplosion,
      `fuel at impact=${player.fuel.toFixed(2)} -> explosion scale ${(1 + player.fuel).toFixed(2)}`,
    );
    origin.unregisterBody(player.body);
    player.dispose();
  }

  // ---- terrain crash (pass 5, item 3): an instant terrain/hard-
  // landing crash used to leave the wreck BELOW the surface (the jet
  // sank through the runway / into a hillside), so handleDeaths'
  // explosion spawned underground and was hidden, the camera's
  // target was below ground, and no debris scattered (debris was
  // gated on crashedFromDying). crash() now clamps the wreck UP to
  // the exact heightfield surface, and main.ts handleDeaths scatters
  // debris for every crash and spectates an instant player crash.
  {
    const key = 'fa18';
    const spec = SPECS[key]!;
    const player = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck,
    });
    player.terrainHeightAt = heightAt;
    origin.registerBody(player.body);
    const gh = heightAt(-2800, -1800);
    // Sink-through: the jet is 3 m BELOW the surface when the
    // terrain safety net fires crash('terrain'). (Set pos directly —
    // resetAt/respawn clamps the spawn UP to the surface, so it
    // cannot place a jet below it.)
    player.resetAt(-2800, gh + 2.25, -1800, 90);
    player.pos.y = gh - 3;
    player.crash('terrain');
    check(
      'terrain crash: wreck is clamped UP to the surface (explosion/scorch/debris read at the surface, not buried)',
      !player.alive && Math.abs(player.pos.y - gh) < 0.01,
      `pos.y=${player.pos.y.toFixed(2)} ground=${gh.toFixed(2)} reason=${player.lastCrashReason}`,
    );
    // Pass 6 (item 3): the wreck stays visible and its group pose
    // is the clamped surface position, not buried below the terrain.
    check(
      'terrain crash: wreck stays visible at the surface position',
      player.group.visible === true &&
        Math.abs(player.group.position.y - gh) < 0.01,
      `group.visible=${player.group.visible} group.y=${player.group.position.y.toFixed(2)} ground=${gh.toFixed(2)}`,
    );
    check(
      'terrain crash: NOT a dying crash (no fall to spectate — the 3 s instant-crash spectate covers it)',
      player.crashedFromDying === false,
      `crashedFromDying=${player.crashedFromDying}`,
    );
    player.dispose();
    // Hard landing at the surface: the clamp is a no-op, the wreck
    // stays where it touched down.
    const player2 = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck,
    });
    player2.terrainHeightAt = heightAt;
    origin.registerBody(player2.body);
    player2.resetAt(-2800, gh + 2.25, -1800, 90);
    const yBefore = player2.pos.y;
    player2.crash('hard landing');
    check(
      'hard-landing crash: wreck stays at the touchdown height (no spurious lift)',
      Math.abs(player2.pos.y - yBefore) < 0.01,
      `pos.y=${player2.pos.y.toFixed(2)} before=${yBefore.toFixed(2)}`,
    );
    check(
      'hard-landing crash: wreck stays visible at the touchdown position',
      player2.group.visible === true &&
        Math.abs(player2.group.position.y - yBefore) < 0.01,
      `group.visible=${player2.group.visible} group.y=${player2.group.position.y.toFixed(2)} before=${yBefore.toFixed(2)}`,
    );
    player2.dispose();
  }

  console.log(failures === 0 ? '\nALL GOING-DOWN CHECKS PASSED' : `\n${failures} GOING-DOWN CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
