// probe-pullup-warning.ts — pass 14, worker D (item 2: PULL UP
// warning too late).
//
// v4.8 live finding: the F-18's "PULL UP — BELOW MIN RELEASE ALT"
// warning only appeared at 123 ft in a −42,131 fpm (−214 m/s) dive.
// Root cause, measured on the flown dive below: the CCIP flag fires
// only when h < dH (altitude lost in a 4 g pull-out started NOW, no
// margin) and computeCcip returns null below 30 m AGL, so in a dive
// entered low the warning gives ~1 s of notice and then vanishes in
// the last 98 ft. The fix (FlightHUD.pullUpTimeBased, wired into the
// #warn chain): in a genuine fast dive (V >= 60 m/s, dive angle
// >= 20 deg — approach, flare, runway bounce and VTOL hover all fall
// outside it) warn while the ballistic time to the ground is under
// max(5 s, pull-out time + 2 s reaction margin).
//
// This probe flies the SAME steep dive twice through the live path
// (real fa18, real GLB, variable frame dt, live 3 m/s wind, origin
// shift, Rapier stepping) and reads BOTH flags off it: the old CCIP
// flag (WeaponSystem.computeCcip, the pre-fix code) and the new HUD
// warning (real FlightHUD.update against a stub DOM, the post-fix
// code). Before the fix the HUD shows PULL UP late (or never, below
// 30 m); after the fix it shows it seconds earlier. The false-
// positive legs (level flight, takeoff/climb, VTOL hover descent,
// 3-deg approach) must stay silent.
//
// Usage: node_modules/.bin/esbuild scripts/probe-pullup-warning.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-pullup-warning.mjs
//   && node /tmp/probe-pullup-warning.mjs

if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type: string; lengthComputable = false; loaded = 0; total = 0;
    constructor(type: string, init?: { lengthComputable?: boolean; loaded?: number; total?: number }) {
      this.type = type; this.lengthComputable = init?.lengthComputable ?? false;
      this.loaded = init?.loaded ?? 0; this.total = init?.total ?? 0;
    }
  };
}
if (typeof (globalThis as unknown as { self?: unknown }).self === 'undefined') {
  (globalThis as unknown as { self?: unknown }).self = globalThis;
}
if (typeof (globalThis as unknown as { Image?: unknown }).Image === 'undefined') {
  (globalThis as unknown as { Image?: unknown }).Image = class {
    private listeners: Record<string, Array<() => void>> = {};
    width = 1; height = 1; onload: (() => void) | null = null; onerror: (() => void) | null = null;
    addEventListener(type: string, fn: () => void): void { (this.listeners[type] ??= []).push(fn); }
    removeEventListener(): void {}
    set src(_v: string) { queueMicrotask(() => { for (const fn of this.listeners['load'] ?? []) fn(); this.onload?.(); }); }
  };
}
if (typeof (globalThis as unknown as { window?: unknown }).window === 'undefined') {
  (globalThis as unknown as { window?: unknown }).window = { innerWidth: 1280, innerHeight: 720 };
}
type StubEl = {
  textContent: string;
  style: Record<string, string>;
  dataset: Record<string, string>;
  classList: { add: (c: string) => void; remove: (c: string) => void; toggle: (c: string, f?: boolean) => void; contains: (c: string) => boolean };
  querySelector: (_s: string) => StubEl;
  querySelectorAll: (_s: string) => StubEl[];
  addEventListener: () => void;
  getContext: (_t: string) => Record<string, unknown>;
};
function makeStubEl(): StubEl {
  const classes = new Set<string>();
  const el: StubEl = {
    textContent: '',
    style: {},
    dataset: {},
    classList: {
      add: (c: string) => { classes.add(c); },
      remove: (c: string) => { classes.delete(c); },
      toggle: (c: string, f?: boolean) => { if (f === undefined) { if (classes.has(c)) classes.delete(c); else classes.add(c); } else if (f) classes.add(c); else classes.delete(c); },
      contains: (c: string) => classes.has(c),
    },
    querySelector: (_s: string) => makeStubEl(),
    querySelectorAll: (_s: string) => [],
    addEventListener: () => {},
    getContext: (_t: string) => ({}),
  };
  return el;
}
const domEls = new Map<string, StubEl>();
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    getElementById: (id: string) => {
      if (!domEls.has(id)) domEls.set(id, makeStubEl());
      return domEls.get(id)!;
    },
    createElement: (tag: string) => {
      if (tag === 'canvas') {
        return { width: 128, height: 128, getContext: () => ({ createRadialGradient: () => ({ addColorStop: () => {} }), fillRect: () => {}, beginPath: () => {}, moveTo: () => {}, lineTo: () => {}, closePath: () => {}, fill: () => {}, arc: () => {}, set fillStyle(_v: unknown) {} }) };
      }
      return makeStubEl();
    },
    createElementNS: (_ns: string, tag: string) => {
      if (tag !== 'img') return {};
      const listeners: Record<string, Array<() => void>> = {};
      const img: Record<string, unknown> = {
        addEventListener: (type: string, fn: () => void) => { (listeners[type] ??= []).push(fn); },
        removeEventListener: () => {},
      };
      Object.defineProperty(img, 'src', { set: () => { queueMicrotask(() => { for (const fn of listeners['load'] ?? []) fn(); }); } });
      return img;
    },
  };
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import type { DeckProvider } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { generateHeights, heightfieldHeightAt, WORLD_SIZE, TERRAIN_SEGMENTS } from '../src/world/PandoraTerrainWorker';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import { FlightHUD, pullUpTimeBased } from '../src/ui/FlightHUD';
import { WeaponSystem } from '../src/combat/WeaponSystem';
import { chaseCameraPosition } from '../src/camera/OrbitCamera';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const M2FT = 3.28084;
const WIND = new THREE.Vector3(3, 0, 0);
// Live frame driver (probe-crash-vfx): raw frame times at browser FPS
// 4-14, split into substeps of at most 0.05 s; sim time runs slower
// than real time on slow frames, exactly like main.ts animate().
const RAW_DT_PATTERN = [0.25, 0.07, 0.16, 0.09, 0.2, 0.12];

function ballisticTImpact(aglM: number, vy: number): number {
  const g = 9.81;
  if (aglM <= 0) return 0;
  return (vy + Math.sqrt(vy * vy + 2 * g * aglM)) / g;
}

async function main(): Promise<void> {
  await RAPIER.init();
  const heights = generateHeights({ seed: 1337, chunkX: 0, chunkZ: 0, size: WORLD_SIZE, resolution: TERRAIN_SEGMENTS + 1 });
  const res = TERRAIN_SEGMENTS + 1;
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) for (let ix = 0; ix < res; ix++) hfHeights[ix * res + iz] = heights[iz * res + ix]!;
  world.createCollider(
    RAPIER.ColliderDesc.heightfield(TERRAIN_SEGMENTS, TERRAIN_SEGMENTS, hfHeights, new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE))
      .setFriction(0).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min).setRestitution(0)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
    terrainBody,
  );
  const scene = new THREE.Scene();
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x + origin.cumulativeShift.x, z + origin.cumulativeShift.z);
  const deck: DeckProvider = { alive: false, deckHeightAt: () => -Infinity, velocity: () => new THREE.Vector3(), velocityAt: () => new THREE.Vector3(), yawRate: () => 0, yawRateAt: () => 0, local: () => null };

  // Flattest site for the dive (scan like probe-crash-vfx findSites).
  let flat = { x: 0, z: 0, g: Infinity };
  for (let x = -5000; x <= 5000; x += 120) {
    for (let z = -5000; z <= 5000; z += 120) {
      const gx = (heightAt(x + 60, z) - heightAt(x - 60, z)) / 120;
      const gz = (heightAt(x, z + 60) - heightAt(x, z - 60)) / 120;
      const g = Math.hypot(gx, gz);
      if (g < flat.g) flat = { x, z, g };
    }
  }
  const baseX = flat.x, baseZ = flat.z;
  const baseG = heightAt(baseX, baseZ);
  console.log(`flat dive site: (${baseX}, ${baseZ}) grad=${flat.g.toFixed(3)} ground=${baseG.toFixed(0)}m`);

  const hud = new FlightHUD();
  const warnEl = domEls.get('warn')!;
  const weapons = new WeaponSystem(scene, world, { groundHeight: heightAt });
  const camera = new THREE.PerspectiveCamera(60, 1280 / 720, 0.1, 20000);

  function makePlayer(key: 'fa18' | 'f35a' | 'f22'): Aircraft {
    const ac = new Aircraft({ scene, world, spec: SPECS[key]!, isPlayer: true, modelKey: SPECS[key]!.modelKey, deck, events: {} });
    ac.terrainHeightAt = heightAt;
    origin.registerBody(ac.body);
    return ac;
  }
  function drainFor(ac: Aircraft): () => void {
    const owners = new Map<number, Aircraft>();
    for (const h of ac.colliderHandles()) owners.set(h, ac);
    return () => {
      eventQueue.drainCollisionEvents((h1, h2, started) => {
        if (!started) return;
        const hit = owners.get(h1) ?? owners.get(h2);
        if (!hit || !hit.alive) return;
        hit.notifyContact(hit.vel.y, false);
      });
    };
  }
  // One live frame: substeps (Rapier + player + VFX-free), then the
  // 10 Hz HUD block exactly as main.ts does it (CCIP -> hud.update),
  // then the chase camera follows (computeCcip needs the camera).
  function liveFrame(ac: Aircraft, drain: () => void, sim: { t: number; rawIdx: number }, sample: (simT: number) => void): void {
    const rawDt = RAW_DT_PATTERN[sim.rawIdx++ % RAW_DT_PATTERN.length]!;
    const sub = Math.max(1, Math.ceil(Math.min(rawDt, 0.1) / 0.05));
    const dt = Math.min(rawDt, 0.1) / sub;
    for (let s = 0; s < sub; s++) {
      sim.t += dt;
      world.timestep = dt;
      world.step(eventQueue);
      drain();
      ac.step(dt, WIND, null);
    }
    camera.position.copy(chaseCameraPosition(ac.pos, ac.fwd(), ac.up(), 30, heightAt, null));
    camera.lookAt(ac.pos.clone());
    const ccip = weapons.computeCcip(ac, heightAt(ac.pos.x, ac.pos.z), camera);
    hud.update(ac, { score: 0, kills: 0, traps: 0, elapsed: sim.t, lockTarget: null, belowMinReleaseAlt: ccip?.belowMinAlt ?? false });
    sample(sim.t);
  }

  // ---------- Flown steep dive (the live case) ----------
  {
    const player = makePlayer('fa18');
    const drain = drainFor(player);
    await player.attachModel();
    // Origin shift before the flight (live path: the jet flies in the
    // shifted frame, exactly like probe-crash-vfx scenario C).
    origin.applyShift(new THREE.Vector3(2500, 0, -1200));
    const g2 = heightAt(baseX, baseZ);
    const hdg = 90;
    // Start LOW (1200 m AGL): the model auto-recovers from steep dives
    // (the v4.8 "dives still auto-recover sometimes" finding — the
    // pull-out takes ~10 s), so a dive staged from 2500 m never
    // reaches the ground. From 1200 m at 200 m/s the jet impacts
    // while still diving.
    player.resetAt(baseX, g2 + 1200, baseZ, hdg);
    // Stage the dive: nose down 65 deg, 200 m/s along the nose.
    // Euler X POSITIVE is nose-down here (v2 convention: pitchIn
    // negative pulls up; probe-crash-vfx scenario B stages +8 deg
    // for an 8-deg nose-down impact).
    const q = new THREE.Quaternion().setFromEuler(new THREE.Euler((65 * Math.PI) / 180, (hdg * Math.PI) / 180, 0, 'YXZ'));
    player.body.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
    const f = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
    player.body.setLinvel({ x: f.x * 200, y: f.y * 200, z: f.z * 200 }, true);
    player.vel.set(f.x * 200, f.y * 200, f.z * 200);
    player.gear = false;
    player.grounded = false;
    player.thrTarget = 0.4;
    player.throttle = 0.4;
    player.pitchIn = 1.0; // full nose-down hold
    const sim = { t: 0, rawIdx: 0 };
    let firstOld: { simT: number; aglFt: number; tImpact: number; vsiFpm: number } | null = null;
    let firstNew: { simT: number; aglFt: number; tImpact: number; vsiFpm: number; text: string } | null = null;
    let sink30k: { simT: number; aglFt: number; warned: boolean } | null = null;
    let minAgl = Infinity;
    let minAglSimT = -1;
    let vyAtMinAgl = 0;
    let maxVsi = 0;
    let frames = 0;
    const trace: string[] = [];
    while (sim.t < 40 && frames < 400) {
      frames++;
      liveFrame(player, drain, sim, (simT) => {
        const agl = player.pos.y - heightAt(player.pos.x, player.pos.z);
        const vsi = player.vel.y;
        if (vsi < maxVsi) maxVsi = vsi;
        if (vsi < 0 && agl < minAgl) { minAgl = agl; minAglSimT = simT; vyAtMinAgl = vsi; }
        if (Math.abs(simT % 1) < 0.05 && trace.length < 40) {
          trace.push(`t=${simT.toFixed(0)} agl=${agl.toFixed(0)}m vy=${vsi.toFixed(0)} ias=${(player.ias * 1.944).toFixed(0)}kt aoa=${player.aoaDeg.toFixed(1)} pitch=${(new THREE.Euler().setFromQuaternion(player.quat, 'YXZ').x * 180 / Math.PI).toFixed(1)} thr=${player.throttle.toFixed(2)} stalled=${player.stalled} alive=${player.alive}`);
        }
        const ccip = weapons.computeCcip(player, heightAt(player.pos.x, player.pos.z), camera);
        if (!firstOld && ccip?.belowMinAlt) {
          firstOld = { simT, aglFt: agl * M2FT, tImpact: ballisticTImpact(agl, vsi), vsiFpm: vsi * M2FT * 60 };
        }
        const warned = warnEl.classList.contains('show') && warnEl.textContent.includes('PULL UP');
        if (!firstNew && warned) {
          firstNew = { simT, aglFt: agl * M2FT, tImpact: ballisticTImpact(agl, vsi), vsiFpm: vsi * M2FT * 60, text: warnEl.textContent };
        }
        if (!sink30k && vsi * M2FT * 60 <= -30000) {
          // Steep live-like sink rate: is the warning already on?
          sink30k = { simT, aglFt: agl * M2FT, warned };
        }
      });
      // Stop once the jet has pulled out and is climbing away above
      // 1500 m AGL — the dive is over (the model auto-recovers from
      // steep dives: the v4.8 "dives still auto-recover sometimes"
      // finding, another worker's item — so no impact is expected).
      if (sim.t > 5 && player.vel.y > 0 && player.pos.y - heightAt(player.pos.x, player.pos.z) > 1500) break;
    }
    console.log(`\ndive: max sink ${(maxVsi * M2FT * 60).toFixed(0)} fpm, closest approach ${minAgl.toFixed(0)}m AGL at t=${minAglSimT.toFixed(2)}s (vy=${(vyAtMinAgl * M2FT * 60).toFixed(0)} fpm there)`);
    console.log('dive trace:\n' + trace.join('\n'));
    console.log(`OLD (CCIP) first warning: ${firstOld ? `t=${firstOld.simT.toFixed(2)}s agl=${firstOld.aglFt.toFixed(0)}ft tImpact=${firstOld.tImpact.toFixed(2)}s vsi=${firstOld.vsiFpm.toFixed(0)}fpm` : 'NEVER'}`);
    console.log(`NEW (HUD) first warning: ${firstNew ? `t=${firstNew.simT.toFixed(2)}s agl=${firstNew.aglFt.toFixed(0)}ft tImpact=${firstNew.tImpact.toFixed(2)}s vsi=${firstNew.vsiFpm.toFixed(0)}fpm "${firstNew.text}"` : 'NEVER'}`);
    if (sink30k) console.log(`at -30,000 fpm sink: agl=${sink30k.aglFt.toFixed(0)}ft, warning already on=${sink30k.warned}`);
    check('flown dive reaches a live-like sink rate (<= -30,000 fpm)', maxVsi * M2FT * 60 <= -30000, `max=${(maxVsi * M2FT * 60).toFixed(0)} fpm`);
    check('flown dive gets dangerously low while still descending (closest approach <= 600 m AGL, vy < -50 m/s there)', minAgl <= 600 && vyAtMinAgl < -50, `minAgl=${minAgl.toFixed(0)}m vy=${(vyAtMinAgl * M2FT * 60).toFixed(0)} fpm`);
    check(
      'NEW warning fires with >= 4 s of time-to-impact left (the 5 s floor, less frame sampling)',
      !!firstNew && firstNew.tImpact >= 4.0,
      firstNew ? `tImpact=${firstNew.tImpact.toFixed(2)}s agl=${firstNew.aglFt.toFixed(0)}ft` : 'no warning',
    );
    check(
      'NEW warning fires high (AGL >= 2500 ft at first appearance) — not in the last 100 ft like the live 123 ft',
      !!firstNew && firstNew.aglFt >= 2500,
      firstNew ? `agl=${firstNew.aglFt.toFixed(0)}ft` : 'no warning',
    );
    check(
      'NEW warning fires strictly earlier than the OLD CCIP flag (the defect the time-based warning fixes) — new warns with more time-to-impact left',
      !!firstNew && (!firstOld || firstNew.tImpact > firstOld.tImpact + 0.5),
      firstOld
        ? `new tImpact=${firstNew?.tImpact.toFixed(2)}s vs old tImpact=${firstOld.tImpact.toFixed(2)}s (old warned at agl=${firstOld.aglFt.toFixed(0)}ft)`
        : 'old never warned',
    );
    check(
      'NEW warning is on at least 2 s before the closest approach',
      !!firstNew && firstNew.simT <= minAglSimT - 2.0,
      firstNew ? `warn t=${firstNew.simT.toFixed(2)}s vs closest t=${minAglSimT.toFixed(2)}s (lead ${(minAglSimT - firstNew.simT).toFixed(2)}s)` : 'no warning',
    );
    check(
      'at a -30,000 fpm sink the NEW warning is already on',
      !!sink30k && sink30k.warned,
      sink30k ? `agl=${sink30k.aglFt.toFixed(0)}ft warned=${sink30k.warned}` : 'never reached -30,000 fpm',
    );
    origin.unregisterBody(player.body);
    player.dispose();
  }

  // ---------- Staged-state checks on the real helper ----------
  {
    const ac = makePlayer('f22');
    const g = heightAt(baseX, baseZ);
    ac.resetAt(baseX, g + 500, baseZ, 90);
    ac.grounded = false;
    // Steep fast dive: 190 m/s at 70 deg down.
    ac.vel.set(60, -190, 0);
    ac.pos.set(baseX, g + 500, baseZ);
    let s = pullUpTimeBased(ac);
    check('staged steep fast dive warns (V=199 m/s, dive 72 deg, 500 m AGL)', s.warn, `tImpact=${s.tImpact.toFixed(2)}s tPull=${s.tPull.toFixed(2)}s theta=${s.thetaDeg.toFixed(0)}deg`);
    // VTOL hover descent: 8 m/s straight down.
    ac.vel.set(0, -8, 0);
    s = pullUpTimeBased(ac);
    check('staged VTOL hover descent does NOT warn (V=8 m/s)', !s.warn, `V=8 theta=${s.thetaDeg.toFixed(0)}deg`);
    // Slow vertical sink: 40 m/s straight down.
    ac.vel.set(0, -40, 0);
    s = pullUpTimeBased(ac);
    check('staged slow vertical sink does NOT warn (V=40 m/s, below the 60 m/s gate)', !s.warn, `V=40`);
    // 3-deg approach at 70 m/s.
    ac.vel.set(69.9, -3.7, 0);
    s = pullUpTimeBased(ac);
    check('staged 3-deg approach does NOT warn (dive angle 3 deg)', !s.warn, `theta=${s.thetaDeg.toFixed(1)}deg`);
    // Level flight at 100 m/s.
    ac.vel.set(100, 0, 0);
    s = pullUpTimeBased(ac);
    check('staged level flight does NOT warn', !s.warn, `theta=${s.thetaDeg.toFixed(1)}deg`);
    // Parked on the runway (grounded, bouncing vsi).
    ac.vel.set(80, -5, 0);
    ac.grounded = true;
    s = pullUpTimeBased(ac);
    check('staged runway bounce does NOT warn (grounded gate)', !s.warn, `grounded=true`);
    origin.unregisterBody(ac.body);
    ac.dispose();
  }

  // ---------- Flown false-positive legs (real HUD, live frames) ----------
  async function flyNoWarn(tag: string, key: 'fa18' | 'f35a', setup: (ac: Aircraft) => void, seconds: number): Promise<void> {
    const ac = makePlayer(key);
    const drain = drainFor(ac);
    await ac.attachModel();
    setup(ac);
    const sim = { t: 0, rawIdx: 0 };
    let warned = false;
    let frames = 0;
    while (sim.t < seconds && frames < 300) {
      frames++;
      liveFrame(ac, drain, sim, () => {
        if (warnEl.classList.contains('show') && warnEl.textContent.includes('PULL UP')) warned = true;
      });
      if (!ac.alive) break;
    }
    check(`${tag}: no PULL UP warning`, !warned, `warned=${warned}`);
    origin.unregisterBody(ac.body);
    ac.dispose();
  }
  const g0 = heightAt(baseX, baseZ);
  await flyNoWarn('level flight 10 s at 100 m/s', 'fa18', (ac) => {
    ac.resetAt(baseX, g0 + 800, baseZ, 90);
    ac.setAirspeed(100);
    ac.gear = false;
    ac.grounded = false;
    ac.thrTarget = 0.7;
    ac.throttle = 0.7;
    ac.pitchIn = 0;
  }, 10);
  await flyNoWarn('takeoff + climb from Alpha', 'fa18', (ac) => {
    ac.resetAt(baseX, g0 + 2.25, baseZ, 90);
    ac.thrTarget = 1;
    ac.throttle = 1;
    ac.pitchIn = 0;
  }, 8);

  console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
