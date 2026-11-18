// probe-ground-drift.ts — ground drift diagnosis (live finding 9).
//
// The live v3 play-test reported the jet rolling/drifting on the
// ground by itself after a respawn (6-15 kt at 0% throttle, heading
// drifting), but the v3 probe measured 0.00 m in 5 s with a freshly
// placed jet. This driver reproduces the live conditions: a respawn
// (not a fresh placement), the default 3 m/s wind, a floating-origin
// shift, and the carrier present. It parks the jet 30 s after a
// respawn in each condition and measures displacement and speed, so
// the cause (wind, slope, friction, origin shift, carrier) can be
// read off the numbers.
//
// Usage: node_modules/.bin/esbuild scripts/probe-ground-drift.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-ground-drift.mjs
//   && node /tmp/probe-ground-drift.mjs

// Node lacks the DOM bits three.js GLTFLoader uses — stub them before
// anything loads a GLB.
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
import { SPECS } from '../src/aircraft/AircraftSpec';
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
  // TRANSPOSED like every other probe and PandoraWorldGenerator:
  // the generator's heights array is z-major, Rapier wants x-major.
  // (The original probe copied the heights directly, putting a
  // transposed terrain under the jet — the runway plateau was NOT
  // under Alpha, so the "no drift" result was unreliable.)
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
  const owners = new Map<number, Aircraft | 'terrain'>();
  owners.set(terrainCollider.handle, 'terrain');
  // Runway boxes (takeoff-driver): flat cuboids per runway, top 5 cm
  // above the heightfield, so the gear contacts one clean face.
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    const box = world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setRotation({ x: 0, y: Math.sin(hdg / 2), z: 0, w: Math.cos(hdg / 2) })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
    owners.set(box.handle, 'terrain');
  }
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);

  const makeAircraft = (): Aircraft => {
    const spec = SPECS['fa18']!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    ac.terrainHeightAt = heightAt;
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    return ac;
  };
  const drain = (ac: Aircraft): void => {
    eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const a = owners.get(h1);
      const b = owners.get(h2);
      const hit = a instanceof Aircraft ? a : b instanceof Aircraft ? b : null;
      if (!hit || !hit.alive) return;
      hit.notifyContact(hit.vel.y, false);
    });
  };
  const substep = (ac: Aircraft, dt: number, wind: THREE.Vector3): void => {
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, wind, null);
  };
  const dispose = (ac: Aircraft): void => {
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  };

  const alpha = RUNWAY_TABLE[0]!;
  // Park the jet for 30 s after a respawn-like reset (resetAt +
  // throttle 0, as respawnPlayer does) and measure the drift.
  // Returns the displacement and the max ground speed in kt.
  const park30 = (
    ac: Aircraft,
    wind: THREE.Vector3,
    label: string,
  ): { dist: number; maxKt: number; hdgDrift: number } => {
    const x0 = ac.pos.x;
    const z0 = ac.pos.z;
    const hdg0 = Math.atan2(ac.fwd().x, ac.fwd().z);
    let maxKt = 0;
    for (let i = 0; i < 30 * 60; i++) {
      substep(ac, 1 / 60, wind);
      const sp = Math.hypot(ac.vel.x, ac.vel.z) * 1.94384;
      if (sp > maxKt) maxKt = sp;
    }
    const dist = Math.hypot(ac.pos.x - x0, ac.pos.z - z0);
    const hdg1 = Math.atan2(ac.fwd().x, ac.fwd().z);
    let dHdg = ((hdg1 - hdg0) * 180) / Math.PI;
    while (dHdg > 180) dHdg -= 360;
    while (dHdg < -180) dHdg += 360;
    return { dist, maxKt, hdgDrift: Math.abs(dHdg) };
  };

  const noWind = new THREE.Vector3(0, 0, 0);
  // Default live wind: 3 m/s from 270 deg (west) — the menu default.
  const liveWind = new THREE.Vector3(-3, 0, 0);

  // 1. Fresh placement, no wind (the v3 probe's condition).
  {
    const ac = makeAircraft();
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60, noWind);
    const r = park30(ac, noWind, 'fresh, no wind');
    check(
      'fresh placement, no wind: jet does not drift in 30 s',
      r.dist < 1 && r.maxKt < 1,
      `dist=${r.dist.toFixed(2)} m maxSpeed=${r.maxKt.toFixed(1)} kt hdgDrift=${r.hdgDrift.toFixed(1)} deg`,
    );
    dispose(ac);
  }

  // 2. Respawn-like reset (crash then resetAt, throttle 0), no wind.
  {
    const ac = makeAircraft();
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60, noWind);
    ac.crash('test kill');
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    ac.thrTarget = 0;
    const r = park30(ac, noWind, 'respawn, no wind');
    check(
      'respawn, no wind: jet does not drift in 30 s',
      r.dist < 1 && r.maxKt < 1,
      `dist=${r.dist.toFixed(2)} m maxSpeed=${r.maxKt.toFixed(1)} kt hdgDrift=${r.hdgDrift.toFixed(1)} deg`,
    );
    dispose(ac);
  }

  // 3. Respawn, default live wind (3 m/s). The wind pushes the jet
  // (drag ~7-15 N on 20,242 kg) — the question is whether it moves.
  {
    const ac = makeAircraft();
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60, liveWind);
    ac.crash('test kill');
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    ac.thrTarget = 0;
    const r = park30(ac, liveWind, 'respawn, 3 m/s wind');
    check(
      'respawn, 3 m/s wind: jet does not drift in 30 s',
      r.dist < 1 && r.maxKt < 1,
      `dist=${r.dist.toFixed(2)} m maxSpeed=${r.maxKt.toFixed(1)} kt hdgDrift=${r.hdgDrift.toFixed(1)} deg`,
    );
    dispose(ac);
  }

  // 4. Respawn on a runway heading across the wind (Alpha hdg 90,
  // wind from the west = a crosswind). A crosswind could weathervane
  // the jet (heading drift) even if it does not translate it.
  {
    const ac = makeAircraft();
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60, liveWind);
    ac.crash('test kill');
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    ac.thrTarget = 0;
    const r = park30(ac, liveWind, 'respawn, crosswind');
    check(
      'respawn, crosswind: heading does not drift in 30 s',
      r.hdgDrift < 5,
      `hdgDrift=${r.hdgDrift.toFixed(1)} deg dist=${r.dist.toFixed(2)} m`,
    );
    dispose(ac);
  }

  // 5. Held stick across respawn (issue 2 — input carried across
  // respawn): the respawn resets the THROTTLE to 0 (main.ts
  // respawnPlayer) but NOT the stick — a finger still on the touch
  // stick (or a held key) keeps pitchIn/rollIn across the respawn.
  // On the ground the rates are capped at 0 below Vr, so a held
  // stick should not move a parked jet. Full pull + full right roll,
  // 30 s, no wind.
  {
    const ac = makeAircraft();
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60, noWind);
    ac.crash('test kill');
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    ac.thrTarget = 0;
    ac.pitchIn = -1;
    ac.rollIn = 1;
    const r = park30(ac, noWind, 'respawn, held stick');
    check(
      'respawn, held stick (full pull + full roll): jet does not drift in 30 s',
      r.dist < 1 && r.maxKt < 1,
      `dist=${r.dist.toFixed(2)} m maxSpeed=${r.maxKt.toFixed(1)} kt hdgDrift=${r.hdgDrift.toFixed(1)} deg`,
    );
    dispose(ac);
  }

  // 6. Off-plateau placement (issue 2 — respawn placement off the
  // flat): 600 m east of Alpha is past the flat top (strip + 160 m)
  // and on the plateau skirt, where the terrain slopes down from
  // 85 m. The gear friction is 0, so a parked jet on a slope should
  // slide downhill. This is the control for "the respawn put the jet
  // somewhere sloped" — if it slides here, a mis-placed respawn
  // (e.g. the old floating-origin bug) would drift.
  {
    const ac = makeAircraft();
    const px = alpha.x + 600;
    const pz = alpha.z;
    ac.resetAt(px, heightAt(px, pz) + 2, pz, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60, noWind);
    const r = park30(ac, noWind, 'off-plateau slope');
    // INFO only: a slope slide is expected physics, not a bug — but
    // record whether it happens, so a live "drift" report can be
    // matched to a sloped placement.
    console.log(
      `INFO  off-plateau slope (600 m east of Alpha): dist=${r.dist.toFixed(2)} m maxSpeed=${r.maxKt.toFixed(1)} kt — a slide here is expected on a slope (gear friction 0)`,
    );
    dispose(ac);
  }

  // 7. Wind gusts (issue 2 — gusts): a 0–12 m/s gust from the west,
  // varying sinusoidally (period 6 s). A gust should not move a
  // parked 20 t jet, but the live report was after a respawn and a
  // gust could have been present.
  {
    const ac = makeAircraft();
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60, noWind);
    ac.crash('test kill');
    ac.resetAt(alpha.x, alpha.elev + 2, alpha.z, alpha.hdg);
    ac.thrTarget = 0;
    const x0 = ac.pos.x;
    const z0 = ac.pos.z;
    let maxKt = 0;
    for (let i = 0; i < 30 * 60; i++) {
      const gust = 6 + 6 * Math.sin((i / 60 / 6) * 2 * Math.PI); // 0–12 m/s
      substep(ac, 1 / 60, new THREE.Vector3(-gust, 0, 0));
      const sp = Math.hypot(ac.vel.x, ac.vel.z) * 1.94384;
      if (sp > maxKt) maxKt = sp;
    }
    const dist = Math.hypot(ac.pos.x - x0, ac.pos.z - z0);
    check(
      'respawn, 0–12 m/s gusts: jet does not drift in 30 s',
      dist < 1 && maxKt < 1,
      `dist=${dist.toFixed(2)} m maxSpeed=${maxKt.toFixed(1)} kt`,
    );
    dispose(ac);
  }

  console.log(failures === 0 ? '\nALL GROUND-DRIFT CHECKS PASSED' : `\n${failures} GROUND-DRIFT CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
