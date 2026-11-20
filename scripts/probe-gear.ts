// probe-gear.ts — pass 10, item 3: landing gears for ALL player
// aircraft (F/A-18F, F-22, F-15E, F-35A) must be VISIBLE on the ground
// / taxi / takeoff roll, STOW after takeoff, stay stowed in flight,
// and DEPLOY on approach/landing — and down in VTOL hover (F-35A).
//
// Two defects the gear work fixes (measured, not assumed):
// - The F/A-18F's GLB carries its OWN gear (nose twin wheels + 2
//   main wheels, baked into the "ground" airframe mesh — pass 11's
//   "no included gear" claim was WRONG, measured on the normalized
//   model where the wheel bottoms had already been aligned to
//   -0.8 m). It is separated into gear_front / gear_L / gear_R by
//   separateIncludedGear() and animated with the existing gearDeploy
//   factor; the F-18 shows ONLY its own gear, no procedural extras.
//   The F-22/F-15E only have a flat "landingOff" belly fairing (the
//   gear-UP configuration) and the F-35A's lowest point is its belly,
//   so they keep procedural gear (strut + wheel x3, built by
//   AircraftRigBindings at the physics gear-post stations).
//   Re-verified 2026-11-20 on the RAW GLBs with MeshoptDecoder:
//   fa18 has the included gear above; f22/f35a/f15e have none.
// - The visual models were mounted too low: normalizeAircraft centred
//   them on the bbox centre, but the tall tail pushes that centre up,
//   so the F-18/F-15E bellies sat 0.35–0.41 m UNDERGROUND on their
//   gear and the physics gear feet (-2.2 m) were hidden inside the
//   fuselage. The model is now shifted so its belly lands on the
//   physics belly (-0.8 m) and the gear feet sit 1.4 m below it.
//
// Pass 11: the strut tops used to stop at bellyY + 0.25 (-0.55 m),
// but the airframe surface at the posts is HIGHER than the belly
// plane (F-15E mains -0.08 m / nose +0.17 m, F-18 mains +1.62 m),
// so the struts visibly floated below the jet — Nicholas's F-15E
// "gears disconnected from the jet" report. Each strut top is now
// raycast to the lowest airframe surface above its station (+0.10 m
// overlap), per jet per station; this probe asserts that contact,
// that the three procedural legs are the ONLY gear (no duplicates),
// and that the wheels sit at the physics feet (-2.2 m).
//
// Per jet, through the LIVE spawn/takeoff/landing path (variable dt,
// live wind): gear down + visible on the runway, stowed after climbing
// past 100 m AGL, auto-deployed on a slow descent below 120 m AGL.
// Plus: F-35A gear down in VTOL hover, and F-18 gear down on the
// helicarrier deck and stowed after a deck takeoff.
//
// Usage: node_modules/.bin/esbuild scripts/probe-gear.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-gear.mjs
//   && node /tmp/probe-gear.mjs

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
// GLTFLoader uses `new Image()` for textures; node has none, so stub
// it (model-inspect.ts pattern — fires 'load' on the next microtask).
// Without this the GLB load throws "Image is not defined" and
// attachModel silently falls back to the procedural jet, so no probe
// has ever tested a real GLB model in node.
if (typeof (globalThis as unknown as { Image?: unknown }).Image === 'undefined') {
  (globalThis as unknown as { Image?: unknown }).Image = class {
    private listeners: Record<string, Array<() => void>> = {};
    width = 1;
    height = 1;
    onload: (() => void) | null = null;
    onerror: (() => void) | null = null;
    addEventListener(type: string, fn: () => void): void {
      (this.listeners[type] ??= []).push(fn);
    }
    removeEventListener(): void {}
    set src(_v: string) {
      queueMicrotask(() => {
        for (const fn of this.listeners['load'] ?? []) fn();
        this.onload?.();
      });
    }
  };
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
import {
  HelicarrierController,
  CARRIER_START_POS,
  CARRIER_WAYPOINTS,
} from '../src/entities/HelicarrierController';
import { HELICARRIER_GLB } from '../src/assets/models/helicarrier';
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

const DT_PATTERN = [0.016, 0.033, 0.05, 0.016, 0.1, 0.033, 0.25, 0.05, 0.016, 0.033];
const WIND = new THREE.Vector3(3, 0, 0);
const DECK_SPAWN_AFT_M = 85;

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
  const terrainBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
  );
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
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
  }
  const scene = new THREE.Scene();
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');

  const carrier = new HelicarrierController(scene, world, {
    modelGlbUri: HELICARRIER_GLB,
    deckTopSourceY: 29.78,
    heightAt: (x, z) => heightfieldHeightAt(heights, res, x, z),
    waypoints: CARRIER_WAYPOINTS,
    startIndex: 0,
  });
  await carrier.build();
  const dc = carrier.deckColliderRef;
  if (dc) owners.set(dc.handle, 'deck');
  const deckProvider: DeckProvider = {
    get alive() {
      return carrier.alive;
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
    yawRateAt: () => carrier.yawRate,
    local: (x, z) => {
      const v = carrier.localPosition(new THREE.Vector3(x, 0, z));
      return v ? { x: v.x, z: v.z } : null;
    },
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
  const substep = (ac: Aircraft, dt: number): void => {
    carrier.update(dt);
    world.timestep = dt;
    world.step(eventQueue);
    drain(ac);
    ac.step(dt, WIND, null);
  };
  const stepFor = (ac: Aircraft, seconds: number): void => {
    let t = 0;
    let pi = 0;
    while (t < seconds) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      const sub = Math.ceil(rawDt / 0.05);
      const dt = rawDt / sub;
      for (let s = 0; s < sub; s++) substep(ac, dt);
      t += rawDt;
    }
  };
  const agl = (ac: Aircraft): number => ac.pos.y - ac.floorY;
  const gearVisible = (ac: Aircraft): boolean => {
    const r = ac.rig;
    if (!r) return false;
    return ['gear_front', 'gear_L', 'gear_R'].every(
      (s) => r.bound[s as 'gear_front' | 'gear_L' | 'gear_R']?.visible,
    );
  };

  carrier.reset(CARRIER_START_POS.clone(), Math.PI / 2);
  const alpha = RUNWAY_TABLE[0]!;

  for (const key of ['fa18', 'f22', 'f35a', 'f15e'] as const) {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
    });
    ac.terrainHeightAt = (x, z) => heightfieldHeightAt(heights, res, x, z);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    await ac.attachModel();
    check(
      `${key}: real GLB loaded (not the procedural fallback)`,
      ac.modelStatus === 'ok',
      `modelStatus=${ac.modelStatus} error=${ac.modelError ?? 'none'} tris=${ac.modelTris}`,
    );
    const rig = ac.rig!;

    // --- Static geometry, model frame (before resetAt moves the
    // aircraft): the gear is the GLB's own on the F-18 (separated
    // gear_front / gear_L / gear_R, no procedural legs) and
    // procedural on the other jets — assert the three legs are the
    // ONLY gear, their wheels sit at the physics feet, and their
    // strut tops reach the airframe surface at each station.
    {
      const model = ac.model!;
      model.updateMatrixWorld(true);
      const allMeshes: THREE.Mesh[] = [];
      model.traverse((o) => {
        if ((o as THREE.Mesh).isMesh) allMeshes.push(o as THREE.Mesh);
      });
      const legGroups: THREE.Object3D[] = [];
      model.traverse((o) => {
        if (o.name.startsWith('gear_leg_')) legGroups.push(o);
      });
      const glbGear = (['gear_front', 'gear_L', 'gear_R'] as const)
        .map((s) => model.getObjectByName(s))
        .filter((o): o is THREE.Object3D => !!o);
      if (spec.includedGear) {
        check(
          `${key}: NO procedural gear legs (the GLB's own gear is used)`,
          legGroups.length === 0,
          `found ${legGroups.length}: ${legGroups.map((g) => g.name).join(', ')}`,
        );
        check(
          `${key}: exactly three separated GLB gear groups`,
          glbGear.length === 3,
          `found ${glbGear.length}: ${glbGear.map((g) => g.name).join(', ')}`,
        );
      } else {
        check(
          `${key}: exactly three procedural gear legs`,
          legGroups.length === 3,
          `found ${legGroups.length}: ${legGroups.map((g) => g.name).join(', ')}`,
        );
      }
      const underGear = (m: THREE.Object3D): boolean => {
        let p: THREE.Object3D | null = m.parent;
        while (p) {
          if (
            p.name.startsWith('gear_leg_') ||
            p.name === 'gear_front' ||
            p.name === 'gear_L' ||
            p.name === 'gear_R'
          )
            return true;
          p = p.parent;
        }
        return m.name === 'gear_front' || m.name === 'gear_L' || m.name === 'gear_R';
      };
      // A stray gear mesh is one below -1.0 m that is not one of the
      // three legs. The airframe ('ground') and the stores
      // ('Weapon_pack') legitimately extend below the belly — the
      // F-18's airframe carries a centreline ventral feature at
      // -1.63 m and stores at -1.45 m — so they are not gear and
      // are excluded by name.
      const stray = allMeshes.filter((m) => {
        const bb = new THREE.Box3().setFromObject(m);
        return (
          bb.min.y < -1.0 &&
          !underGear(m) &&
          m.name !== 'ground' &&
          m.name !== 'Weapon_pack'
        );
      });
      check(
        `${key}: no gear meshes outside the three legs (no duplicates)`,
        stray.length === 0,
        stray.length > 0
          ? `stray below -1.0 m: ${stray.map((m) => `'${m.name}'`).join(', ')}`
          : `${allMeshes.filter((m) => underGear(m)).length} gear meshes, all under the three legs`,
      );
      const ray2 = new THREE.Raycaster();
      // Gear stations: the GLB's own stations for the F-18 (its gear
      // sits at the model's stations, not the physics posts — nose
      // z +3.57, mains z -2.26), the physics posts for the procedural
      // jets (nose (0, +0.35L), mains (x = ±1.6, -0.15L)).
      const stations: Array<['gear_front' | 'gear_L' | 'gear_R', number, number]> = spec.includedGear
        ? spec.includedGear.map((st) => [
            st.slot,
            (st.x0 + st.x1) / 2,
            (st.z0 + st.z1) / 2,
          ])
        : [
            ['gear_front', 0, 0.35 * spec.length],
            ['gear_L', -1.6, -0.15 * spec.length],
            ['gear_R', 1.6, -0.15 * spec.length],
          ];
      for (const [slot, sx, sz] of stations) {
        const leg = rig.bound[slot];
        if (!leg) {
          check(`${key}: ${slot} leg bound`, false);
          continue;
        }
        const kids: THREE.Mesh[] = [];
        leg.traverse((o) => {
          if ((o as THREE.Mesh).isMesh) kids.push(o as THREE.Mesh);
        });
        let strutTop = -Infinity;
        let wheelFoot = Infinity;
        for (const k of kids) {
          const bb = new THREE.Box3().setFromObject(k);
          strutTop = Math.max(strutTop, bb.max.y);
          wheelFoot = Math.min(wheelFoot, bb.min.y);
        }
        const others = allMeshes.filter((m) => !underGear(m));
        ray2.set(new THREE.Vector3(sx, -10, sz), new THREE.Vector3(0, 1, 0));
        ray2.far = 30;
        const hits = ray2.intersectObjects(others, false);
        const belly = hits.length > 0 ? hits[0]!.point.y : NaN;
        // The included gear is cut 0.05 m short of the airframe (the
        // cap margin) plus a triangle-straddle, so its strut top can
        // sit up to ~0.08 m below the surface — still attached, not
        // floating (the F-15E's old 0.72 m gap is what this catches).
        // Procedural legs overlap the airframe by 0.10 m, so they
        // sit above the surface instead.
        const gapTol = spec.includedGear ? 0.1 : 0.02;
        check(
          `${key}: ${slot} strut top touches the belly (no floating gear)`,
          !Number.isNaN(belly) && strutTop >= belly - gapTol && strutTop <= belly + 0.3,
          `strutTop=${strutTop.toFixed(2)} belly=${Number.isNaN(belly) ? 'NO HIT' : belly.toFixed(2)} gap=${Number.isNaN(belly) ? '?' : (belly - strutTop).toFixed(2)}`,
        );
        check(
          `${key}: ${slot} wheel foot at the physics gear foot (-2.2 m)`,
          Math.abs(wheelFoot - -2.2) < 0.05,
          `wheelFoot=${wheelFoot.toFixed(2)} (${kids.length} meshes in leg)`,
        );
      }
      if (key === 'fa18') {
        const helmet = model.getObjectByName('helmet002');
        check(
          'fa18: pilot helmet hidden (helmet002, not helmet.002)',
          !!helmet && !helmet.visible,
          helmet ? `visible=${helmet.visible}` : 'helmet002 not found',
        );
      }
    }

    // --- Spawn on Alpha: gear down, visible, belly on the physics belly.
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    ac.brake = true;
    ac.thrTarget = 0;
    stepFor(ac, 2);
    check(
      `${key}: model belly aligns with the physics belly (-0.8 m, not underground)`,
      Math.abs(rig.bellyY - -0.8) < 0.15,
      `bellyY=${rig.bellyY.toFixed(2)} (was -2.61 fa18 / -2.55 f15e / -2.03 f22 / -1.72 f35a)`,
    );
    check(
      `${key}: three gear legs bound (${spec.includedGear ? 'GLB own gear' : 'procedural strut + wheel'})`,
      !!rig.bound.gear_front && !!rig.bound.gear_L && !!rig.bound.gear_R,
      `bound: ${rig.bound.gear_front ? 'front' : '-'} ${rig.bound.gear_L ? 'L' : '-'} ${rig.bound.gear_R ? 'R' : '-'}`,
    );
    check(
      `${key}: gear DOWN and visible on the runway at spawn`,
      ac.alive && ac.gear && rig.gearDeploy > 0.95 && gearVisible(ac) && ac.onSurface,
      `gear=${ac.gear} deploy=${rig.gearDeploy.toFixed(2)} visible=${gearVisible(ac)} onSurf=${ac.onSurface} agl=${agl(ac).toFixed(1)}`,
    );

    // --- Takeoff + climb past 100 m AGL: gear stows.
    ac.brake = false;
    ac.thrTarget = 1;
    const startY = ac.pos.y;
    let t = 0;
    let pi = 0;
    let climbed = false;
    while (t < 40 && !climbed) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      const sub = Math.ceil(rawDt / 0.05);
      const dt = rawDt / sub;
      for (let s = 0; s < sub; s++) substep(ac, dt);
      t += rawDt;
      if (ac.onSurface && ac.ias > 55) ac.pitchIn = -1;
      else if (!ac.onSurface && ac.pos.y < startY + 15) ac.pitchIn = -0.3;
      else if (!ac.onSurface && agl(ac) < 110) ac.pitchIn = -0.3;
      else ac.pitchIn = 0;
      if (!ac.onSurface && agl(ac) > 110) climbed = true;
      if (!ac.alive) break;
    }
    // Let the 1.2 s stow transition finish in level flight.
    ac.pitchIn = 0;
    stepFor(ac, 3);
    check(
      `${key}: gear STOWED after takeoff (climbed past 100 m AGL)`,
      ac.alive && !ac.gear && rig.gearDeploy < 0.05 && !gearVisible(ac),
      `gear=${ac.gear} deploy=${rig.gearDeploy.toFixed(2)} visible=${gearVisible(ac)} agl=${agl(ac).toFixed(1)} ias=${ac.ias.toFixed(0)} m/s`,
    );

    // --- High-speed stow hold (v4.6 live failure, 2026-11-20): the
    // F-18's gear was DOWN in flight at 499–773 kt / 5,861–29,777 ft
    // while the other three jets stowed correctly — F-18-specific,
    // and the harness had missed it twice (it only checked the stow
    // right after takeoff, at ~113 m/s). The cause was the F-18's
    // INCLUDED gear baked into the airframe mesh: it never stowed
    // because it was never animated, while the procedural legs the
    // probe checked did stow. Now the included gear IS the animated
    // gear, so hold it at high speed. The jet is teleported to level
    // flight at 250 m/s (~486 kt) at 2,000 m — the same teleport the
    // approach check uses, because flying there by pitch control
    // porpoises (the jet trades speed for altitude and never gets
    // fast) — then the live physics, variable dt and wind run for
    // 10 s and the gear must STILL be stowed.
    {
      const hdgRad = (alpha.hdg * Math.PI) / 180;
      const fwdX = Math.sin(hdgRad);
      const fwdZ = Math.cos(hdgRad);
      const tx = alpha.x;
      const ty = alpha.elev + 2000;
      const tz = alpha.z;
      const spd = 250;
      const half = hdgRad / 2;
      ac.body.setTranslation({ x: tx, y: ty, z: tz }, true);
      ac.body.setRotation({ x: 0, y: Math.sin(half), z: 0, w: Math.cos(half) }, true);
      ac.body.setLinvel({ x: fwdX * spd, y: 0, z: fwdZ * spd }, true);
      ac.pos.set(tx, ty, tz);
      ac.vel.set(fwdX * spd, 0, fwdZ * spd);
      ac.quat.set(0, Math.sin(half), 0, Math.cos(half));
      ac.pitchIn = 0;
      ac.rollIn = 0;
      ac.yawIn = 0;
      ac.thrTarget = 1;
      stepFor(ac, 10);
      const fastIas = ac.ias;
      const fastAltFt = ac.pos.y * 3.28084;
      check(
        `${key}: gear STAYS stowed at high speed (no F-18 re-deploy in flight)`,
        ac.alive && ac.ias > 200 && !ac.gear && rig.gearDeploy < 0.05 && !gearVisible(ac),
        `ias=${fastIas.toFixed(0)} m/s (${(fastIas * 1.944).toFixed(0)} kt) alt=${fastAltFt.toFixed(0)} ft gear=${ac.gear} deploy=${rig.gearDeploy.toFixed(2)} visible=${gearVisible(ac)} agl=${agl(ac).toFixed(1)}`,
      );
      // Climb away / slow down before the approach teleport.
      ac.thrTarget = 0.5;
      ac.pitchIn = -0.3;
      stepFor(ac, 3);
    }

    // --- Approach: teleport to a controlled approach state and
    // descend through the deploy band. Flying a full scripted
    // approach (takeoff -> climb -> slow down -> descend) proved too
    // fragile across models — each jet stalls or accelerates out of
    // the narrow deploy window (55–67 m/s, AGL < 120 m, descending)
    // at a different point. The teleport places the jet at 150 m
    // above Alpha, level, heading along the runway at 58 m/s, gear
    // UP (already stowed) — the live descent and the auto-deploy
    // logic are what is tested, not the setup. The body is moved
    // directly (resetAt/respawn would reset the gear to down and
    // defeat the check).
    {
      const hdgRad = (alpha.hdg * Math.PI) / 180;
      const fwdX = Math.sin(hdgRad);
      const fwdZ = Math.cos(hdgRad);
      const tx = alpha.x;
      const ty = alpha.elev + 150;
      const tz = alpha.z;
      const spd = 58;
      const half = hdgRad / 2;
      ac.body.setTranslation({ x: tx, y: ty, z: tz }, true);
      ac.body.setRotation({ x: 0, y: Math.sin(half), z: 0, w: Math.cos(half) }, true);
      ac.body.setLinvel({ x: fwdX * spd, y: 0, z: fwdZ * spd }, true);
      ac.pos.set(tx, ty, tz);
      ac.vel.set(fwdX * spd, 0, fwdZ * spd);
      ac.quat.set(0, Math.sin(half), 0, Math.cos(half));
      ac.pitchIn = 0;
      ac.rollIn = 0;
      ac.yawIn = 0;
      // Very gentle descent (pitchIn +0.02) in the dirty
      // configuration — full flaps + airbrake — so gravity does not
      // accelerate it out of the deploy window; idle-ish throttle.
      // pitchIn +0.1 dives steeply enough that gravity outruns the
      // airbrake and the jet is past 67 m/s before it reaches
      // 120 m AGL; the light F-35A needs the flaps' extra drag to
      // hold speed in the descent.
      ac.thrTarget = 0.05;
      ac.airbrake = 1;
      ac.flaps = 30;
      ac.pitchIn = 0.02;
    }
    let deployAgl = -1;
    let deployed = false;
    let minIas = ac.ias;
    t = 0;
    pi = 0;
    while (t < 30 && !deployed) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      const sub = Math.ceil(rawDt / 0.05);
      const dt = rawDt / sub;
      for (let s = 0; s < sub; s++) substep(ac, dt);
      t += rawDt;
      if (!ac.alive) break;
      minIas = Math.min(minIas, ac.ias);
      if (ac.gear && rig.gearDeploy > 0.95) {
        deployed = true;
        deployAgl = agl(ac);
      }
      if (agl(ac) < 25) break; // safety: never fly into the terrain
    }
    ac.airbrake = 0;
    ac.flaps = 0;
    check(
      `${key}: gear DEPLOYS on approach (slow descent below 120 m AGL, still airborne)`,
      ac.alive && deployed && deployAgl > 0 && deployAgl < 120 && !ac.onSurface,
      `deployAgl=${deployAgl < 0 ? 'never' : deployAgl.toFixed(1) + ' m'} gear=${ac.gear} deploy=${rig.gearDeploy.toFixed(2)} onSurf=${ac.onSurface} ias=${ac.ias.toFixed(0)} m/s minIas=${minIas.toFixed(0)} m/s agl=${agl(ac).toFixed(1)}`,
    );
    // Climb away so the jet does not fly into the terrain.
    ac.thrTarget = 1;
    ac.pitchIn = -0.5;
    stepFor(ac, 8);

    for (const h of ac.colliderHandles()) owners.delete(h);
    ac.dispose();
  }

  // --- F-35A VTOL hover: gear down while hovering.
  {
    const spec = SPECS['f35a']!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
    });
    ac.terrainHeightAt = (x, z) => heightfieldHeightAt(heights, res, x, z);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    await ac.attachModel();
    check(
      'f35a-hover: real GLB loaded (not the procedural fallback)',
      ac.modelStatus === 'ok',
      `modelStatus=${ac.modelStatus} error=${ac.modelError ?? 'none'}`,
    );
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    ac.vectorDeg = 90;
    ac.thrTarget = 1;
    ac.brake = false;
    stepFor(ac, 6); // break contact at full throttle
    ac.thrTarget = 0.94; // hover point — the hold maintains altitude
    stepFor(ac, 14);
    const rig = ac.rig!;
    check(
      'f35a: gear DOWN in VTOL hover (vector 90, slow, airborne)',
      ac.alive && !ac.onSurface && ac.vectorDeg >= 70 && ac.ias < 25 && ac.gear && rig.gearDeploy > 0.95 && gearVisible(ac),
      `alive=${ac.alive} onSurf=${ac.onSurface} vector=${ac.vectorDeg.toFixed(0)} ias=${ac.ias.toFixed(1)} gear=${ac.gear} deploy=${rig.gearDeploy.toFixed(2)} agl=${agl(ac).toFixed(1)}`,
    );
    for (const h of ac.colliderHandles()) owners.delete(h);
    ac.dispose();
  }

  // --- F-18 on the helicarrier deck: gear down parked, stowed after
  // a deck takeoff.
  {
    const spec = SPECS['fa18']!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
    });
    ac.terrainHeightAt = (x, z) => heightfieldHeightAt(heights, res, x, z);
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    await ac.attachModel();
    const fwd = new THREE.Vector3(Math.sin(carrier.yaw), 0, Math.cos(carrier.yaw));
    const sx = carrier.pos.x - fwd.x * DECK_SPAWN_AFT_M;
    const sz = carrier.pos.z - fwd.z * DECK_SPAWN_AFT_M;
    ac.resetAt(sx, carrier.deckTopY + 2.2, sz, (carrier.yaw * 180) / Math.PI);
    const dv = deckProvider.velocityAt(sx, sz);
    ac.body.setLinvel({ x: dv.x, y: dv.y, z: dv.z }, true);
    ac.vel.copy(dv);
    ac.brake = true;
    ac.thrTarget = 0;
    stepFor(ac, 10);
    const rig = ac.rig!;
    check(
      'fa18 deck: gear DOWN and visible parked on the helicarrier deck',
      ac.alive && ac.onDeck && ac.gear && rig.gearDeploy > 0.95 && gearVisible(ac),
      `onDeck=${ac.onDeck} gear=${ac.gear} deploy=${rig.gearDeploy.toFixed(2)} visible=${gearVisible(ac)}`,
    );
    ac.brake = false;
    ac.thrTarget = 1;
    const startY = ac.pos.y;
    let t = 0;
    let pi = 0;
    let climbed = false;
    while (t < 30 && !climbed) {
      const rawDt = DT_PATTERN[pi % DT_PATTERN.length]!;
      pi++;
      const sub = Math.ceil(rawDt / 0.05);
      const dt = rawDt / sub;
      for (let s = 0; s < sub; s++) substep(ac, dt);
      t += rawDt;
      if (ac.onSurface && ac.ias > 55) ac.pitchIn = -1;
      else if (!ac.onSurface && ac.pos.y < startY + 15) ac.pitchIn = -0.3;
      else if (!ac.onSurface && agl(ac) < 110) ac.pitchIn = -0.3;
      else ac.pitchIn = 0;
      if (!ac.onSurface && agl(ac) > 110) climbed = true;
      if (!ac.alive) break;
    }
    ac.pitchIn = 0;
    stepFor(ac, 3);
    check(
      'fa18 deck: gear STOWED after a deck takeoff',
      ac.alive && !ac.gear && rig.gearDeploy < 0.05 && !gearVisible(ac),
      `gear=${ac.gear} deploy=${rig.gearDeploy.toFixed(2)} visible=${gearVisible(ac)} agl=${agl(ac).toFixed(1)}`,
    );
    for (const h of ac.colliderHandles()) owners.delete(h);
    ac.dispose();
  }

  console.log(failures === 0 ? 'ALL PASS' : `${failures} FAILURES`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
