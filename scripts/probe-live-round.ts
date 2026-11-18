// probe-live-round.ts — full live-round replication (pass 5).
//
// Nicholas reports the jet "appears and immediately explodes" on a
// runway spawn (F-35A AND F-22) and appears floating + collides with
// a disc on a helicarrier spawn — but the plain spawn driver
// (probe-spawn-all) shows all six jets surviving both spawns. The
// difference must be in the live-only pieces, so this driver runs
// the COMPLETE startRound + main loop for the two reported jets:
// real GLB attachModel, 4 veteran enemies (2 close + 2 far) with AI
// controllers, WeaponSystem updates, the live 3 m/s wind, and the
// exact substep order (carrier update -> Rapier step -> drain ->
// player step -> AI steps -> weapons). 30 s per jet: 5 s parked, then
// a takeoff roll. Any death is reported with the exact step, reason,
// position and state.
//
// Usage: node_modules/.bin/esbuild scripts/probe-live-round.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-live-round.mjs
//   && node /tmp/probe-live-round.mjs

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB. (Same stubs as probe-carrier-deck.ts.)
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
import { HelicarrierController, CARRIER_START_POS, CARRIER_WAYPOINTS } from '../src/entities/HelicarrierController';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';
import { AircraftAIController } from '../src/entities/AircraftAIController';
import { WeaponSystem } from '../src/combat/WeaponSystem';
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
const R2D = 180 / Math.PI;

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
  const terrainCollider = world.createCollider(
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
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    const box = world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
    owners.set(box.handle, 'terrain');
  }
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(
      heights,
      res,
      x + origin.cumulativeShift.x,
      z + origin.cumulativeShift.z,
    );

  const carrier = new HelicarrierController(scene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt,
  });
  await carrier.build();
  const deckBody = carrier.deckBodyRef;
  if (deckBody) {
    for (let i = 0; i < deckBody.numColliders(); i++) {
      owners.set(deckBody.collider(i).handle, 'deck');
    }
  }
  carrier.registerWithFloatingOrigin(origin);
  const deckProvider: DeckProvider = {
    get alive() {
      return carrier.active && carrier.alive;
    },
    deckHeightAt: (x, z) => carrier.deckHeightAt(x, z),
    velocity: () => carrier.vel.clone(),
    velocityAt: (x, z) => {
      const rx = x - carrier.pos.x;
      const rz = z - carrier.pos.z;
      const w = carrier.yawRate;
      return new THREE.Vector3(carrier.vel.x + w * rz, carrier.vel.y, carrier.vel.z - w * rx);
    },
    yawRate: () => carrier.yawRate,
    yawRateAt: (_x, _z) => carrier.yawRate,
    local: (x, z) => {
      const v = carrier.localPosition(new THREE.Vector3(x, 0, z));
      return v ? { x: v.x, z: v.z } : null;
    },
  };
  const weapons = new WeaponSystem(scene, world, {
    groundHeight: heightAt,
    deckHeight: (x, z) => carrier.deckHeight(x, z),
    carrierPodPositions: () =>
      carrier.podObjects.map((p) => p.getWorldPosition(new THREE.Vector3())),
    onExplosion: (_pos, _scale) => {},
    onCarrierPodHit: (pod, dmg) => carrier.damagePod(pod, dmg),
  });

  const alpha = RUNWAY_TABLE[0]!;

  for (const key of ['f22', 'f35a'] as const) {
    origin.reset();
    carrier.reset(
      CARRIER_START_POS.clone(),
      Math.atan2(
        CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
        CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
      ),
    );
    // Player, exactly like main.ts startRound (fresh aircraft, real
    // model attach, then the runway spawn).
    const spec = SPECS[key]!;
    const player = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
    });
    player.terrainHeightAt = heightAt;
    origin.registerBody(player.body);
    for (const h of player.colliderHandles()) owners.set(h, player);
    await player.attachModel();
    console.log(`[${key}] model: ${player.modelStatus} tris=${player.modelTris}`);
    player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    // Enemies (default config: 4, veteran; first two close ahead).
    const enemies: Aircraft[] = [];
    const aiControllers = new Map<Aircraft, AircraftAIController>();
    for (let i = 0; i < 4; i++) {
      const ekey = i % 2 === 0 ? 'mig29' : 'su34';
      const e = new Aircraft({
        scene,
        world,
        spec: SPECS[ekey]!,
        isPlayer: false,
        modelKey: SPECS[ekey]!.modelKey,
        deck: deckProvider,
      });
      e.terrainHeightAt = heightAt;
      origin.registerBody(e.body);
      for (const h of e.colliderHandles()) owners.set(h, e);
      let ex: number;
      let ey: number;
      let ez: number;
      if (i < 2) {
        const fwd = player.fwd();
        const rgt = new THREE.Vector3(fwd.z, 0, -fwd.x);
        const along = 450 + i * 150;
        const side = i === 0 ? 120 : -150;
        const up = 120 + i * 60;
        ex = player.pos.x + fwd.x * along + rgt.x * side;
        ey = player.pos.y + up;
        ez = player.pos.z + fwd.z * along + rgt.z * side;
      } else {
        const a = Math.random() * Math.PI * 2;
        const d = 1200 * (0.8 + Math.random() * 0.5);
        ex = player.pos.x + Math.cos(a) * d;
        ey = 650 + Math.random() * 350;
        ez = player.pos.z + Math.sin(a) * d;
      }
      e.resetAt(ex, ey, ez, Math.random() * 360);
      e.setAirspeed(150);
      e.gear = false;
      e.grounded = false;
      e.thrTarget = 0.8;
      const ai = new AircraftAIController(e, 'veteran', 'enemy', {
        groundHeight: heightAt,
        originShift: origin.cumulativeShift,
      });
      origin.registerVector(ai.navTarget);
      aiControllers.set(e, ai);
      enemies.push(e);
    }
    const drain = (): void => {
      eventQueue.drainCollisionEvents((h1, h2, started) => {
        if (!started) return;
        const a = owners.get(h1);
        const b = owners.get(h2);
        const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
        if (!hit || !hit.alive) return;
        const other = hit === a ? b : a;
        hit.notifyContact(hit.vel.y, other === 'deck');
      });
    };
    let diedAt = -1;
    let deathDetail = '';
    for (let i = 0; i < 1800; i++) {
      const t = i / 60;
      if (i === 300) {
        player.thrTarget = 1;
        player.pitchIn = -1;
      }
      carrier.update(1 / 60);
      world.timestep = 1 / 60;
      world.step(eventQueue);
      drain();
      player.step(1 / 60, WIND, null);
      for (const e of enemies) {
        if (!e.alive) continue;
        const ai = aiControllers.get(e)!;
        const ctrl = ai.update(1 / 60, player, carrier, t);
        if (ai.fireGun) weapons.aiGunShot(e, player, ai.skill.err);
        if (ai.fireMissile) weapons.launchMissile(e, player);
        if (ai.dropBomb) weapons.dropBomb(e);
        e.step(1 / 60, ctrl, WIND, heightAt, null);
      }
      weapons.update(1 / 60, enemies);
      const shift = origin.shiftIfNeeded(player.pos);
      if (shift) origin.applyShift(shift);
      if (!player.alive) {
        diedAt = i;
        deathDetail = `reason=${player.lastCrashReason ?? '-'} dying=${player.dying} dyingReason=${player.dyingReason ?? '-'} hp=${player.hp.toFixed(0)} fire=${player.fireKind ?? '-'} pos=(${player.pos.x.toFixed(0)},${player.pos.y.toFixed(0)},${player.pos.z.toFixed(0)})`;
        break;
      }
      if (i % 60 === 0) {
        console.log(
          `  [${key} t=${t.toFixed(0)}s] alive=${player.alive} y=${player.pos.y.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)} kt gnd=${player.grounded} hp=${player.hp.toFixed(0)}`,
        );
      }
    }
    check(
      `${key} live round: player survives 30 s (5 s parked + takeoff) with 4 veteran enemies`,
      diedAt === -1,
      diedAt >= 0 ? `DIED at step ${diedAt} (t=${(diedAt / 60).toFixed(1)}s) ${deathDetail}` : `alive, y=${player.pos.y.toFixed(0)}`,
    );
    for (const e of enemies) {
      origin.unregisterBody(e.body);
      e.dispose();
    }
    origin.unregisterBody(player.body);
    player.dispose();
  }

  console.log(failures === 0 ? '\nALL LIVE-ROUND CHECKS PASSED' : `\n${failures} LIVE-ROUND CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
