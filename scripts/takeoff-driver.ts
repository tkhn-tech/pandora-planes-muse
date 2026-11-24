// takeoff-driver.ts — node verification for the Vite takeoff fixes.
//
// Replicates the live takeoff against the FINAL code: real Aircraft /
// CurveAerodynamicsSolver, a real Rapier world with the real terrain
// heightfield (transposed, like PandoraWorldGenerator), and main.ts's
// exact per-substep order (world.timestep = dt -> world.step ->
// drainCollisionEvents -> ac.step). No browser, no models (attachModel
// is never called — the physics does not need the mesh).
//
// Usage: node_modules/.bin/esbuild scripts/takeoff-driver.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/takeoff-driver.mjs
//   && node /tmp/takeoff-driver.mjs

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import {
  generateHeights,
  heightAt,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
  GRID_STEP,
} from '../src/world/PandoraTerrainWorker';
import {
  CARRIER_START_POS,
  CARRIER_WAYPOINTS,
  HelicarrierController,
} from '../src/entities/HelicarrierController';
import type { DeckProvider } from '../src/physics/FlightPhysicsController';

const D2R = Math.PI / 180;
const MS2KT = 1.94384;

interface TakeoffResult {
  key: string;
  flaps: number;
  crashed: string | null;
  liftOffSpeedMs: number | null;
  liftOffKt: number | null;
  liftOffTimeS: number | null;
  liftOffDistM: number | null;
  maxAoADeg: number;
  accel05s: number;
  fuelAtLiftOff: number;
  gOnGround: number;
}

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

async function main(): Promise<void> {
  await RAPIER.init();

  // ---- world: real terrain heightfield, transposed like the generator ----
  // ZEROHEIGHTS=1 flattens the whole heightfield (diagnostic: separates
  // terrain-data triggers from aircraft/controller triggers).
  const heights = generateHeights({
    seed: 1337,
    chunkX: 0,
    chunkZ: 0,
    size: WORLD_SIZE,
    resolution: TERRAIN_SEGMENTS + 1,
  });
  if (process.env['ZEROHEIGHTS'] === '1') heights.fill(0);
  const res = TERRAIN_SEGMENTS + 1;
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);
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
      new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    )
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
    terrainBody,
  );

  // ---- runway boxes (same as PandoraWorldGenerator.buildHeightfieldCollider)
  // A flat cuboid per runway, top 5 cm above the heightfield, so the gear
  // contacts one clean face instead of the heightfield's 40 m triangles.
  // Without these a gear ski crossing a cell boundary can make Rapier's
  // contact solver explode (probed: a 35 m/s velocity change in one step
  // on the F-35A flaps-0 roll at x=-1326.8, nose ski crossing -1320).
  const runwayBoxHandles: number[] = [];
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    const box = world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setRotation({
          x: 0,
          y: Math.sin(hdg / 2),
          z: 0,
          w: Math.cos(hdg / 2),
        })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
    runwayBoxHandles.push(box.handle);
  }

  const scene = new THREE.Scene();
  const wind = new THREE.Vector3(0, 0, 0);
  const owners = new Map<number, Aircraft | 'terrain' | 'deck'>();
  owners.set(terrainCollider.handle, 'terrain');
  for (const h of runwayBoxHandles) owners.set(h, 'terrain');

  // Helicarrier (procedural fallback hull — no GLB in the driver), as in
  // main.ts startRound: reset to CARRIER_START_POS, heading east toward waypoint 0.
  const carrier = new HelicarrierController(scene, world, {});
  await carrier.build();
  carrier.reset(
    CARRIER_START_POS.clone(),
    Math.atan2(
      CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
      CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
    ),
  );
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

  const makeAircraft = (key: string): Aircraft => {
    const spec = SPECS[key]!;
    let crashReason: string | null = null;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
      deck: deckProvider,
      events: {
        onCrash: (_a, reason) => {
          crashReason = reason;
        },
      },
    });
    (ac as unknown as { __crash: () => string | null }).__crash = () =>
      crashReason;
    // The controller's ground height is the EXACT heightfield surface
    // (triangle interpolation — main.ts's heightfieldGroundHeight),
    // not the analytic heightAt() the old driver used.
    ac.terrainHeightAt = (x: number, z: number) =>
      heightfieldHeightAt(heights, res, x, z);
    if (process.env['NOCCD'] === '1') ac.body.enableCcd(false);
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
      if (contactLog) {
        contactLog.push(
          `contact t=${hit.time.toFixed(3)} h1=${h1} h2=${h2} ` +
            `vel=(${hit.vel.x.toFixed(2)},${hit.vel.y.toFixed(2)},${hit.vel.z.toFixed(2)})`,
        );
      }
      hit.notifyContact(hit.vel.y, false);
    });
  };
  // Set while tracing: collects contact events for the traced run.
  let contactLog: string[] | null = null;

  /** One main.ts-style substep. */
  // velProbe: when set, records the Rapier linvel before world.step,
  // after world.step, and after ac.step() — pins down where a velocity
  // jump originates (Rapier contact solver vs controller setLinvel).
  let velProbe: {
    pre: { x: number; y: number; z: number };
    postWorld: { x: number; y: number; z: number };
    postStep: { x: number; y: number; z: number };
  } | null = null;
  const substep = (ac: Aircraft, dt: number): void => {
    world.timestep = dt;
    if (velProbe !== null) {
      const v0 = ac.body.linvel();
      velProbe.pre = { x: v0.x, y: v0.y, z: v0.z };
    }
    world.step(eventQueue);
    if (velProbe !== null) {
      const v1 = ac.body.linvel();
      velProbe.postWorld = { x: v1.x, y: v1.y, z: v1.z };
    }
    drain(ac);
    ac.step(dt, wind, null);
    if (velProbe !== null) {
      const v2 = ac.body.linvel();
      velProbe.postStep = { x: v2.x, y: v2.y, z: v2.z };
    }
  };

  /** Deck-aware substep: carrier.update() first, as in main.ts. */
  const substepDeck = (ac: Aircraft, dt: number): void => {
    carrier.update(dt);
    substep(ac, dt);
  };

  const crashOf = (ac: Aircraft): string | null =>
    (ac as unknown as { __crash: () => string | null }).__crash();

  const alpha = RUNWAY_TABLE[0]!;

  // ---- E. Runway flatness (analytic + sampled grid) ----
  // Alpha is a plateau runway (elev from the table): the strip must be
  // exactly flat AT the plateau elevation, not at 0 — the check is
  // elev-relative with the same strictness (exact flatness), only the
  // reference moved.
  let flatWorst = 0;
  for (let d = -425; d <= 425; d += 5) {
    const h = heightAt(alpha.x + d, alpha.z);
    flatWorst = Math.max(flatWorst, Math.abs(h - alpha.elev));
  }
  check('Alpha strip + 100 m overrun flat (heightAt == elev)', flatWorst === 0, `worst |h - elev| = ${flatWorst}`);
  // Sampled grid, bilinear at the exact centreline points the jet rolls
  // over (the takeoff roll — every jet is airborne by ~300 m).
  // Nearest-node sampling would catch feather-zone nodes past the
  // overrun tips — those are expected to rise (the analytic strip is
  // flat to 425 m, but the grid feathers early at the tips; the strip
  // mesh itself is flat and the jet is airborne by then).
  const bilinear = (x: number, z: number): number => {
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
  let gridWorst = 0;
  for (let d = 0; d <= 350; d += 5) {
    gridWorst = Math.max(gridWorst, Math.abs(bilinear(alpha.x + d, alpha.z) - alpha.elev));
  }
  check('Alpha roll path flat on the sampled grid too', gridWorst < 0.01, `worst |h - elev| = ${gridWorst.toFixed(4)}`);

  // ---- F. Carrier bearing from Alpha ----
  // Pass 13 (coordinator): the old check demanded the carrier dead ahead
  // (offNose < 1 deg, dist < 1200 m) — i.e. sitting on Alpha's extended
  // centreline, which is exactly the defect pass 13 fixed (jets fly under
  // the carrier on Alpha takeoffs). The new loop keeps the carrier clear
  // of every runway corridor (>= 500 m) while still ahead and visible:
  // slot-0 start is 18.4 deg off the nose at 1897 m, 600 m north of the
  // centreline. Assert that geometry instead.
  const dx = CARRIER_START_POS.x - alpha.x;
  const dz = CARRIER_START_POS.z - alpha.z;
  const bearing = ((Math.atan2(dx, dz) / D2R) % 360 + 360) % 360;
  const offNose = Math.abs(((bearing - alpha.hdg + 540) % 360) - 180);
  const dist = Math.hypot(dx, dz);
  // Cross-track distance from Alpha's centreline: dot the displacement
  // with the perpendicular unit vector (cos h, -sin h). For hdg 90 deg
  // this is |dz| = 600 m at the slot-0 start.
  const hdgRad = alpha.hdg * D2R;
  const crossTrack = Math.abs(
    dx * Math.cos(hdgRad) - dz * Math.sin(hdgRad),
  );
  check(
    'Carrier ahead of Alpha but clear of the runway corridor',
    offNose < 30 && dist < 3000 && crossTrack > 500,
    `bearing ${bearing.toFixed(1)} deg (runway hdg ${alpha.hdg}), ${offNose.toFixed(1)} deg off the nose, ${dist.toFixed(0)} m, cross-track ${crossTrack.toFixed(0)} m`,
  );
  const wp0 = CARRIER_WAYPOINTS[0]!;
  const wpBearing = ((Math.atan2(wp0.x - CARRIER_START_POS.x, wp0.z - CARRIER_START_POS.z) / D2R) % 360 + 360) % 360;
  check('Carrier first waypoint east (takeoff direction)', Math.abs(wpBearing - 90) < 1, `waypoint bearing ${wpBearing.toFixed(1)} deg`);

  // ---- A/B. Spawn settle + takeoff per jet ----
  const jets = ['fa18', 'f35a', 'f22', 'f15e'];
  const results: TakeoffResult[] = [];
  // TRACE=key:flaps (e.g. TRACE=f15e:30) prints a 0.25 s time series
  // for one takeoff roll instead of the pass/fail line.
  const traceArg = process.env['TRACE'] ?? '';
  const [traceKey, traceFlaps] = traceArg.split(':');
  const traceWinArg = process.env['TRACEWIN'] ?? '';
  const traceWin = traceWinArg
    ? (traceWinArg.split(',').map(Number) as [number, number])
    : null;

  const pitchDegOf = (ac: Aircraft): number => {
    const f = ac.fwd();
    return (Math.asin(Math.max(-1, Math.min(1, f.y))) * 180) / Math.PI;
  };

  for (const key of jets) {
    const spec = SPECS[key]!;
    // Body mass check (density math in createCollider).
    const probe = makeAircraft(key);
    const rapierMass = probe.body.mass();
    check(
      `${key} Rapier mass == spec.mass`,
      Math.abs(rapierMass - spec.mass) < 1,
      `${rapierMass.toFixed(1)} kg vs ${spec.mass} kg`,
    );
    check(`${key} has 4 colliders (fuselage + 3 gear posts)`, probe.colliderHandles().length === 4);
    probe.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');

    for (const flaps of [0, 30]) {
      const ac = makeAircraft(key);
      // --- spawn settle: 2 s, no input ---
      ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
      let gGround = 0;
      for (let i = 0; i < 120; i++) {
        substep(ac, 1 / 60);
        if (i === 119) gGround = ac.gload;
        if (!ac.alive) break;
      }
      const settled =
        ac.alive &&
        Math.abs(ac.pos.y - (alpha.elev + 2.2)) < 0.15 &&
        ac.vel.length() < 0.5;
      check(
        `${key} flaps ${flaps}: spawn settles, no crash (2 s, no input)`,
        settled,
        `alive=${ac.alive} pos.y=${ac.pos.y.toFixed(2)} |v|=${ac.vel.length().toFixed(2)} crash=${crashOf(ac)}`,
      );
      check(
        `${key} flaps ${flaps}: G reads 1.0 on the ground`,
        Math.abs(gGround - 1) < 0.05,
        `gload=${gGround.toFixed(2)}`,
      );

      // --- takeoff roll: full throttle, full pull, flaps as set ---
      // (pull held from the start, as the live player did — the rotation
      // clamp in the controller must hold the nose down until Vr).
      ac.flaps = flaps;
      ac.thrTarget = 1;
      ac.pitchIn = -1;
      const tracing = traceKey === key && Number(traceFlaps) === flaps;
      if (tracing) {
        contactLog = [];
        console.log('t(s) kt x(m) y(m) pitch(deg) AoA grounded vsi thr');
      }
      const x0 = ac.pos.x;
      const t0 = ac.time;
      let vAt2s = 0;
      let vAt5s = 0;
      let maxAoA = -90;
      let liftOff: TakeoffResult | null = null;
      let crashed: string | null = null;
      for (let i = 0; i < 60 * 30; i++) {
        // Arm the velocity probe for steps inside the fine-trace window
        // (set before substep so it records this step's three phases).
        if (
          tracing &&
          traceWin &&
          i / 60 >= traceWin[0] &&
          i / 60 <= traceWin[1]
        ) {
          velProbe = {
            pre: { x: 0, y: 0, z: 0 },
            postWorld: { x: 0, y: 0, z: 0 },
            postStep: { x: 0, y: 0, z: 0 },
          };
        } else {
          velProbe = null;
        }
        substep(ac, 1 / 60);
        const t = ac.time - t0;
        if (Math.abs(t - 2) < 1 / 120) vAt2s = ac.ias;
        if (Math.abs(t - 5) < 1 / 120) vAt5s = ac.ias;
        maxAoA = Math.max(maxAoA, ac.aoa);
        if (tracing && i % 15 === 0) {
          console.log(
            `${t.toFixed(2)} ${(ac.ias * MS2KT).toFixed(0)} ${(ac.pos.x - x0).toFixed(0)} ` +
              `${ac.pos.y.toFixed(2)} ${pitchDegOf(ac).toFixed(1)} ${ac.aoa.toFixed(1)} ` +
              `${ac.grounded} ${ac.vsi.toFixed(2)} ${ac.throttle.toFixed(2)}`,
          );
        }
        // Fine trace: TRACEWIN=lo,hi logs every step in the window.
        if (tracing && traceWin && t >= traceWin[0] && t <= traceWin[1]) {
          console.log(
            `  ${t.toFixed(3)} kt=${(ac.ias * MS2KT).toFixed(1)} x=${(ac.pos.x - x0).toFixed(2)} ` +
              `y=${ac.pos.y.toFixed(3)} pitch=${pitchDegOf(ac).toFixed(2)} AoA=${ac.aoa.toFixed(2)} ` +
              `gnd=${ac.grounded} vsi=${ac.vsi.toFixed(3)} vy=${ac.vel.y.toFixed(3)} ` +
              `pre=(${velProbe!.pre.x.toFixed(2)},${velProbe!.pre.y.toFixed(2)}) ` +
              `postW=(${velProbe!.postWorld.x.toFixed(2)},${velProbe!.postWorld.y.toFixed(2)}) ` +
              `postS=(${velProbe!.postStep.x.toFixed(2)},${velProbe!.postStep.y.toFixed(2)})`,
          );
        }
        if (!ac.alive) {
          crashed = crashOf(ac);
          if (tracing) {
            console.log(
              `CRASH ${crashed} at t=${t.toFixed(2)} kt=${(ac.ias * MS2KT).toFixed(0)} ` +
                `x=${(ac.pos.x - x0).toFixed(0)} y=${ac.pos.y.toFixed(2)} pitch=${pitchDegOf(ac).toFixed(1)} AoA=${ac.aoa.toFixed(1)}`,
            );
          }
          break;
        }
        // Lift-off: clearly off the ground, climbing, gear height
        // cleared, and at a real flying speed (the ias > 40 guard keeps
        // a tail-strike bounce from counting as a lift-off).
        if (
          ac.pos.y > heightAt(ac.pos.x, ac.pos.z) + 4 &&
          ac.vsi > 0 &&
          !ac.grounded &&
          ac.ias > 40
        ) {
          liftOff = {
            key,
            flaps,
            crashed: null,
            liftOffSpeedMs: ac.ias,
            liftOffKt: ac.ias * MS2KT,
            liftOffTimeS: t,
            liftOffDistM: ac.pos.x - x0,
            maxAoA,
            accel05s: vAt5s / 5,
            fuelAtLiftOff: ac.fuel,
            gOnGround: gGround,
          };
          break;
        }
        // Runway end: 650 m strip + 100 m overrun from the centre start.
        if (ac.pos.x - x0 > 750) break;
      }
      if (liftOff) {
        results.push(liftOff);
        if (tracing && contactLog) {
          console.log('--- contacts during traced run ---');
          console.log(contactLog.join('\n') || '(none)');
        }
        check(
          `${key} flaps ${flaps}: lifts off inside the runway`,
          liftOff.liftOffDistM! < 650 &&
            liftOff.liftOffSpeedMs! < 100 &&
            liftOff.maxAoA < 30,
          `${liftOff.liftOffSpeedMs!.toFixed(1)} m/s (${liftOff.liftOffKt!.toFixed(0)} kt) in ${liftOff.liftOffDistM!.toFixed(0)} m / ${liftOff.liftOffTimeS!.toFixed(1)} s, max AoA ${liftOff.maxAoA.toFixed(1)} deg, 0-5 s accel ${liftOff.accel05s.toFixed(2)} m/s^2, fuel ${(liftOff.fuelAtLiftOff * 100).toFixed(1)}%`,
        );
      } else {
        failures++;
        console.log(
          `FAIL  ${key} flaps ${flaps}: NO LIFT-OFF — crashed=${crashed} final x=${(ac.pos.x - x0).toFixed(0)} m speed=${(ac.ias * MS2KT).toFixed(0)} kt AoA=${ac.aoa.toFixed(1)} deg alive=${ac.alive}`,
        );
      }
      ac.dispose();
      owners.clear();
      owners.set(terrainCollider.handle, 'terrain');
    }
  }

  // ---- C. F-35A hover at vector 90 ----
  // At combat mass the honest hover throttle is ~94% (supplement §2:
  // 0.94*191 kN = 179.5 kN vs weight 179.2 kN — a 0.2% margin, so at
  // 94% the jet just sits weightless on its gear; at 100% it climbs).
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60); // settle
    ac.vector = 90;
    ac.thrTarget = 1.0;
    for (let i = 0; i < 360; i++) {
      substep(ac, 1 / 60);
      if (!ac.alive) break;
    }
    check(
      'F-35A climbs vertically at 100% throttle, vector 90',
      ac.alive && ac.pos.y >= alpha.elev + 5 && ac.vsi > 0,
      `alive=${ac.alive} alt=${ac.pos.y.toFixed(1)} m vsi=${ac.vsi.toFixed(2)} m/s`,
    );
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  }
  {
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
    ac.vector = 90;
    ac.thrTarget = 0.94;
    let maxAlt = 0;
    for (let i = 0; i < 300; i++) {
      substep(ac, 1 / 60);
      maxAlt = Math.max(maxAlt, ac.pos.y);
      if (!ac.alive) break;
    }
    check(
      'F-35A balances at ~94% throttle, vector 90 (weightless on gear)',
      ac.alive && ac.pos.y - alpha.elev < 3.5 && Math.abs(ac.vsi) < 0.5,
      `alive=${ac.alive} alt=${ac.pos.y.toFixed(2)} m (max ${maxAlt.toFixed(2)}) vsi=${ac.vsi.toFixed(2)} m/s — thrust ~ weight, no sink, no climb`,
    );
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
  }
  {
    // 74% (the old light-mass hover throttle) must NOT lift at combat mass.
    const ac = makeAircraft('f35a');
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    for (let i = 0; i < 120; i++) substep(ac, 1 / 60);
    ac.vector = 90;
    ac.thrTarget = 0.74;
    for (let i = 0; i < 300; i++) substep(ac, 1 / 60);
    check(
      'F-35A does NOT lift at 74% throttle, vector 90 (combat mass)',
      ac.alive && ac.pos.y - alpha.elev < 3,
      `alt=${ac.pos.y.toFixed(1)} m (stays on the deck, as the supplement predicts)`,
    );
    ac.dispose();
  }

  // ---- D. Slow-frame case: dt = 0.05 substeps (the timestep fix) ----
  {
    const ac = makeAircraft('fa18');
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    for (let i = 0; i < 40; i++) substep(ac, 0.05); // 2 s at 20 fps
    const settled = ac.alive && Math.abs(ac.pos.y - (alpha.elev + 2.2)) < 0.3;
    ac.flaps = 30;
    ac.thrTarget = 1;
    ac.pitchIn = -1;
    const x0 = ac.pos.x;
    let lifted = false;
    for (let i = 0; i < 20 * 30; i++) {
      substep(ac, 0.05);
      if (!ac.alive) break;
      if (ac.pos.y > heightAt(ac.pos.x, ac.pos.z) + 4 && ac.vsi > 0 && !ac.grounded) {
        lifted = true;
        break;
      }
      if (ac.pos.x - x0 > 750) break;
    }
    check(
      'F-18 flaps 30 lifts off at dt=0.05 substeps (slow frame, timestep fix)',
      settled && lifted,
      `settled=${settled} lifted=${lifted} crash=${crashOf(ac)}`,
    );
    ac.dispose();
  }

  // ---- E. Helicarrier deck spawn (stage c) ----
  // Mirrors main.ts startRound's deck spawn for each of the 4 jets:
  // 120 m aft of the deck centre, facing the bow at the carrier's
  // heading, inheriting the carrier's cruise velocity. Checks: the jet
  // rides the deck for 3 s with no input (no sliding off the stern),
  // then takes off from the deck (full throttle + full pull, flaps 0)
  // and gets airborne without crashing.
  for (const key of ['fa18', 'f35a', 'f22', 'f15e'] as const) {
    const ac = makeAircraft(key);
    const fwd = new THREE.Vector3(
      Math.sin(carrier.yaw),
      0,
      Math.cos(carrier.yaw),
    );
    const sx = carrier.pos.x - fwd.x * 120;
    const sz = carrier.pos.z - fwd.z * 120;
    ac.resetAt(sx, carrier.deckTopY + 2.2, sz, (carrier.yaw * 180) / Math.PI);
    const dv = fwd.clone().multiplyScalar(carrier.speed);
    ac.body.setLinvel({ x: dv.x, y: 0, z: dv.z }, true);
    ac.vel.copy(dv);
    const deckY = carrier.deckTopY;
    // 3 s, no input: must stay on the deck and ride with it.
    for (let i = 0; i < 180; i++) substepDeck(ac, 1 / 60);
    const ridesDeck =
      ac.alive &&
      ac.onDeck &&
      Math.abs(ac.pos.y - (deckY + 2.2)) < 0.6 &&
      Math.abs(ac.vel.x - carrier.vel.x) < 2 &&
      Math.abs(ac.vel.z - carrier.vel.z) < 2;
    check(
      `${key} deck spawn: rides the deck 3 s, no input`,
      ridesDeck,
      `alive=${ac.alive} onDeck=${ac.onDeck} y=${ac.pos.y.toFixed(2)} (deck ${deckY.toFixed(2)}+2.2) vel=(${ac.vel.x.toFixed(1)},${ac.vel.z.toFixed(1)}) carrier=(${carrier.vel.x.toFixed(1)},${carrier.vel.z.toFixed(1)})`,
    );
    // Take off from the deck.
    ac.flaps = 0;
    ac.thrTarget = 1;
    ac.pitchIn = -1;
    let airborne = false;
    let airTime = 0;
    for (let i = 0; i < 60 * 12; i++) {
      substepDeck(ac, 1 / 60);
      if (!ac.alive) break;
      if (!ac.onDeck && ac.pos.y > deckY + 2 && ac.ias > 40) {
        airborne = true;
        airTime = i / 60;
        break;
      }
    }
    check(
      `${key} deck spawn: takes off from the deck`,
      ac.alive && airborne,
      `alive=${ac.alive} airborne=${airborne} in ${airTime.toFixed(1)} s at ${(ac.ias * MS2KT).toFixed(0)} kt, y=${ac.pos.y.toFixed(0)} m crash=${crashOf(ac)}`,
    );
    ac.dispose();
    owners.clear();
    owners.set(terrainCollider.handle, 'terrain');
    if (dc) owners.set(dc.handle, 'deck');
    carrier.reset(
      CARRIER_START_POS.clone(),
      Math.atan2(
        CARRIER_WAYPOINTS[0]!.x - CARRIER_START_POS.x,
        CARRIER_WAYPOINTS[0]!.z - CARRIER_START_POS.z,
      ),
    );
  }

  // ---- F. Sustained turn (BET §0 calibration) ----
  // F-18 at 400 kt, sea level, coordinated turn: bank to 60 deg and
  // hold it (bank-error feedback) with the pitch holding the AoA for
  // the turn (AoA-error feedback, target ~4 deg = the CL for
  // n = 1/cos(60 deg) = 2 g at 400 kt). The coordinated-turn rate is
  // omega = g*tan(phi)/V = 4.7 deg/s at 60 deg / 400 kt; measure the
  // heading change over 10 s and the speed bleed.
  // The §0 max-performance turn (~19 deg/s at ~82 deg bank, 7 g) is
  // re-tested in F2 below (it departed before the weathercock fix).
  {
    const ac = makeAircraft('fa18');
    ac.resetAt(alpha.x, 200, alpha.z, alpha.hdg);
    ac.setAirspeed(400 / 1.94384);
    ac.thrTarget = 1;
    const hdg0 = Math.atan2(ac.fwd().x, ac.fwd().z);
    const spd0 = ac.ias;
    const bankDes = (60 * Math.PI) / 180;
    const aoaTarget = 4;
    let minAlt = Infinity;
    let maxAoA = -Infinity;
    for (let i = 0; i < 600; i++) {
      const bankNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
      ac.rollIn = Math.max(-1, Math.min(1, (bankDes - bankNow) * 2));
      // AoA hold: pull (nose up, pitchIn < 0) while BELOW the target
      // AoA, ease off / push (nose down, pitchIn > 0) above it.
      ac.pitchIn = Math.max(
        -1,
        Math.min(1, (ac.aoaDeg - aoaTarget) * 0.1),
      );
      ac.yawIn = 0;
      substep(ac, 1 / 60);
      if (!ac.alive) break;
      minAlt = Math.min(minAlt, ac.pos.y);
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
    }
    const hdg1 = Math.atan2(ac.fwd().x, ac.fwd().z);
    let dHdg = hdg1 - hdg0;
    while (dHdg > Math.PI) dHdg -= 2 * Math.PI;
    while (dHdg < -Math.PI) dHdg += 2 * Math.PI;
    const turnRate = Math.abs((dHdg * 180) / Math.PI / 10);
    const bleed = spd0 - ac.ias;
    check(
      'F-18 coordinated turn at 400 kt SL, 60 deg bank: ~4.7 deg/s (g*tan(phi)/V)',
      ac.alive && turnRate > 4 && turnRate < 7,
      `rate=${turnRate.toFixed(1)} deg/s over 10 s, alive=${ac.alive} minAlt=${minAlt.toFixed(0)} m maxAoA=${maxAoA.toFixed(1)} (bank-hold overshoots a few deg, so the rate runs a little high)`,
    );
    check(
      'F-18 coordinated turn: speed holds (no collapse to a stall)',
      ac.alive && ac.ias > 180 && bleed < 30,
      `400 kt -> ${(ac.ias * 1.94384).toFixed(0)} kt (bleed ${bleed.toFixed(1)} m/s)`,
    );
    ac.dispose();
  }

  // ---- F2. Max-performance turn (BET §0 calibration, re-tested after
  // the weathercock sign fix) ----
  // F-18 at 400 kt SL, 80 deg bank, AoA held at ~12 deg (the CL for
  // n = 1/cos(80 deg) = 5.8 g at 400 kt). The coordinated rate is
  // g*tan(80 deg)/V = 15.5 deg/s; the §0 calibration (bank ~82 deg,
  // 7 g, drag ~205 kN vs thrust 196 kN) is ~19 deg/s. Before the
  // weathercock fix this DEPARTED (AoA ~180 deg, speed 400 -> 22 kt)
  // with both a constant full pull AND an AoA-hold — the inverted
  // weathercock was positive feedback. Re-test now.
  // The AoA hold uses a STRONG gain (0.3): a weak gain (0.1) could
  // not pull the 5.8 g needed to hold altitude in an 80 deg bank, so
  // the jet sank and the speed collapsed at t~9 s. A max-performance
  // turn bleeds speed (high induced drag at 4-6 g), so the speed
  // check allows a large bleed — the point is the turn rate and that
  // the jet does not depart or stall, not that it holds 400 kt.
  {
    const ac = makeAircraft('fa18');
    ac.resetAt(alpha.x, 200, alpha.z, alpha.hdg);
    ac.setAirspeed(400 / 1.94384);
    ac.thrTarget = 1;
    const hdg0 = Math.atan2(ac.fwd().x, ac.fwd().z);
    const spd0 = ac.ias;
    const bankDes = (80 * Math.PI) / 180;
    const aoaTarget = 12;
    let minAlt = Infinity;
    let maxAoA = -Infinity;
    for (let i = 0; i < 600; i++) {
      const bankNow = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ').z;
      ac.rollIn = Math.max(-1, Math.min(1, (bankDes - bankNow) * 2));
      ac.pitchIn = Math.max(
        -1,
        Math.min(1, (ac.aoaDeg - aoaTarget) * 0.3),
      );
      ac.yawIn = 0;
      substep(ac, 1 / 60);
      if (!ac.alive) break;
      minAlt = Math.min(minAlt, ac.pos.y);
      maxAoA = Math.max(maxAoA, ac.aoaDeg);
    }
    const hdg1 = Math.atan2(ac.fwd().x, ac.fwd().z);
    let dHdg = hdg1 - hdg0;
    while (dHdg > Math.PI) dHdg -= 2 * Math.PI;
    while (dHdg < -Math.PI) dHdg += 2 * Math.PI;
    const turnRate = Math.abs((dHdg * 180) / Math.PI / 10);
    const bleed = spd0 - ac.ias;
    check(
      'F-18 max-performance turn at 400 kt SL, 80 deg bank: ~15-19 deg/s (g*tan(phi)/V, §0 calibration)',
      ac.alive && turnRate > 12 && turnRate < 25 && maxAoA < 30,
      `rate=${turnRate.toFixed(1)} deg/s over 10 s, alive=${ac.alive} minAlt=${minAlt.toFixed(0)} m maxAoA=${maxAoA.toFixed(1)}`,
    );
    check(
      'F-18 max-performance turn: no stall/collapse (bleeds, does not fall out of the sky)',
      ac.alive && ac.ias > 140 && bleed < 70,
      `400 kt -> ${(ac.ias * 1.94384).toFixed(0)} kt (bleed ${bleed.toFixed(1)} m/s)`,
    );
    ac.dispose();
  }

  // ---- Summary table ----
  console.log('\nTakeoff summary (combat masses, per-model supplement §7):');
  console.log('jet    flaps  Vr m/s (kt)  dist m  time s  maxAoA  0-5s m/s^2');
  for (const r of results) {
    console.log(
      `${r.key.padEnd(6)} ${String(r.flaps).padEnd(5)}  ` +
        `${r.liftOffSpeedMs!.toFixed(1)} (${r.liftOffKt!.toFixed(0)})`.padEnd(12) +
        `${r.liftOffDistM!.toFixed(0).padEnd(7)} ${r.liftOffTimeS!.toFixed(1).padEnd(6)}  ` +
        `${r.maxAoA.toFixed(1).padEnd(6)}  ${r.accel05s.toFixed(2)}`,
    );
  }

  console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('DRIVER ERROR', e);
  process.exit(2);
});
