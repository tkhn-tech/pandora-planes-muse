// probe-wreck-watch.ts — pass 14, worker D (item 3: respawn dialog
// dismissible; item 4: debris field + fire glow).
//
// ITEM 3 — v4.8 live finding: after a crash the respawn picker
// ("Respawn at" — centre of the screen at top 38%) cannot be
// dismissed to keep watching the wreck; its Cancel button is hidden
// whenever the jet is dead (showRespawnPicker shows it only for the
// stuck/alive case), so the picker covers the wreck view until the
// player respawns. The fix: the Cancel button is always shown —
// labelled "Watch the wreck" when dead — and clicking it hides the
// picker; nothing re-opens it (the respawn timer already fired, and
// the only other showRespawnPicker callers are R / the Respawn
// button via skipSpectate). The spectate banner itself ("WENT DOWN
// — … · watching the wreck") also gets an ✕ dismiss button that
// hides the banner while spectating continues. The picker/banner
// wiring is DOM (main.ts + index.html), so this probe verifies it
// structurally (same pattern as probe-spectate-orbit) plus the
// state-machine invariant in source; the rendered result still
// belongs to the parent's live check.
//
// ITEM 4 — v4.8 live finding: at +75 s after the crash the wreck
// reads as largely intact, just blackened and tilted — no clearly
// scattered debris field, and the fire glow had burned out.
// Measured root cause: the debris and attached-fire lifetimes are
// 30 SIM seconds (DEBRIS_LIFE / ATTACHED_PLUME_LIFE), and at the
// live 4-9 fps the sim runs at 0.4-0.9x real time, so 30 sim-s
// expires at ~33-75 REAL seconds — exactly when the player looks.
// The fix (CrashVFXManager): debris and attached wreck fire live
// 90 sim-s (as long as the scorch mark, SCORCH_LIFE 90), and an
// attached fire plume holds full brightness until its last 15 s
// instead of thinning linearly from birth. The crash below runs
// the live path (real fa18 + GLB, staged fast impact, variable
// frame dt with the main.ts 0.10 s sim cap, live 3 m/s wind, origin
// shift, handleDeaths replica) and samples the field at +30 sim-s
// (≈ +75 real s at 4 fps, the live observation point) and at
// +89/+91 sim-s.
//
// Usage: node_modules/.bin/esbuild scripts/probe-wreck-watch.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-wreck-watch.mjs
//   && node /tmp/probe-wreck-watch.mjs

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
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    createElement: (tag: string) => {
      if (tag === 'canvas') {
        return { width: 128, height: 128, getContext: () => ({ createRadialGradient: () => ({ addColorStop: () => {} }), fillRect: () => {}, beginPath: () => {}, moveTo: () => {}, lineTo: () => {}, closePath: () => {}, fill: () => {}, arc: () => {}, set fillStyle(_v: unknown) {} }) };
      }
      return {};
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
import * as fs from 'node:fs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import type { DeckProvider } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { generateHeights, heightfieldHeightAt, WORLD_SIZE, TERRAIN_SEGMENTS } from '../src/world/PandoraTerrainWorker';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import { CrashVFXManager } from '../src/combat/CrashVFXManager';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const WIND = new THREE.Vector3(3, 0, 0);
const RAW_DT_PATTERN = [0.25, 0.07, 0.16, 0.09, 0.2, 0.12];
const SAMPLE_TIMES = [0, 1.5, 10, 15, 30, 60, 75, 89, 91];

type BoomRec = { sprite: THREE.Sprite; t: number; scale: number };
type PlumeRec = { group: THREE.Group; life: number; maxLife: number; parent: THREE.Object3D | null; baseOpacity: number };
type DebrisRec = { mesh: THREE.Object3D; body: RAPIER.RigidBody; life: number; age: number };
type ScorchRec = { mesh: THREE.Mesh; age: number };

interface FieldSample {
  t: number;
  debris: number;
  torn: number;
  resting: number;
  belowGround: number;
  maxSpreadM: number;
  scattered: number; // pieces > 5 m from the wreck
  /** Sorted distance of each piece from the wreck (m). */
  dists: number[];
  worstPiece: string;
  fireOpacity: number; // max sprite opacity over attached plumes
  attachedPlumes: number;
  scorch: number;
}

async function main(): Promise<void> {
  // ---------- ITEM 3: dialog dismiss (structural checks) ----------
  const html = fs.readFileSync('index.html', 'utf8');
  const mainSrc = fs.readFileSync('src/main.ts', 'utf8');
  const spectateDiv = html.slice(html.indexOf('<div id="spectate">'), html.indexOf('<div id="respawnPicker">'));
  check('spectate banner has a dismiss (✕) button', spectateDiv.includes('id="spectateDismiss"'), spectateDiv.trim().slice(0, 200));
  check('respawn picker has a Cancel button', html.includes('id="pickerCancel"'), '');
  const showPicker = mainSrc.slice(mainSrc.indexOf('function showRespawnPicker'), mainSrc.indexOf('function hideRespawnPicker'));
  check(
    'showRespawnPicker always shows Cancel (not only for the stuck case)',
    !showPicker.includes("cancel.style.display = forStuck ? '' : 'none'") && showPicker.includes("cancel.style.display = ''"),
    '',
  );
  check(
    'showRespawnPicker labels Cancel "Watch the wreck" when the jet is dead',
    showPicker.includes('Watch the wreck'),
    '',
  );
  check(
    'picker Cancel click hides the picker (dismiss path)',
    mainSrc.includes("$('pickerCancel')?.addEventListener('click', hideRespawnPicker)"),
    '',
  );
  check(
    'spectate ✕ click hides the banner only (spectating continues)',
    mainSrc.includes("$('spectateDismiss')?.addEventListener('click', () => setSpectateBanner(false))"),
    '',
  );
  // No per-frame re-show: the picker is opened only by the (one-shot)
  // respawn timer and by the player's own R / Respawn-button action.
  const animateBody = mainSrc.slice(mainSrc.indexOf('function animate()'));
  check('animate() never re-opens the picker on its own', !animateBody.includes('showRespawnPicker'), '');
  const showCalls = (mainSrc.match(/showRespawnPicker\(/g) ?? []).length;
  check('only the expected showRespawnPicker sites exist (definition + timer, skipSpectate, stuck-R, dying-R)', showCalls === 5, `matches=${showCalls}`);

  // ---------- ITEM 4: debris field + fire glow (live crash path) ----------
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

  // Flattest site (same scan as the other probes).
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
  console.log(`\nflat crash site: (${baseX}, ${baseZ}) grad=${flat.g.toFixed(3)} ground=${heightAt(baseX, baseZ).toFixed(0)}m`);

  const vfx = new CrashVFXManager(scene, { groundHeight: heightAt, origin });
  const player = new Aircraft({
    scene, world, spec: SPECS['fa18']!, isPlayer: true, modelKey: SPECS['fa18']!.modelKey, deck,
    events: {
      onDetachSegment: (_a, segment, pos, vel) => {
        if (segment) vfx.spawnDebris(segment, world, pos, vel);
      },
    },
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
  await player.attachModel();
  console.log(`fa18 modelStatus=${player.modelStatus}`);
  // Origin shift before the flight (live path).
  origin.applyShift(new THREE.Vector3(2500, 0, -1200));
  const g2 = heightAt(baseX, baseZ);
  const fhdg = 90;

  // Live frame driver (main.ts animate(): rawDt capped at 0.25 s, sim
  // time capped at 0.10 s/frame into <= 0.05 s substeps).
  let rawIdx = 0;
  function liveFrame(stepSub: (dt: number) => void): void {
    const rawDt = RAW_DT_PATTERN[rawIdx++ % RAW_DT_PATTERN.length]!;
    const simDt = Math.min(rawDt, 0.1);
    const sub = Math.max(1, Math.ceil(simDt / 0.05));
    const dt = simDt / sub;
    for (let s = 0; s < sub; s++) stepSub(dt);
  }

  // handleDeaths replica (main.ts, verbatim calls/order): explosion,
  // scorch, attached fire + smoke for an instant crash, 4 debris.
  const prevAlive = new Map<Aircraft, boolean>();
  const attachedPlumes: THREE.Group[] = [];
  let impactPos: THREE.Vector3 | null = null;
  function handleDeaths(all: Aircraft[]): void {
    for (const a of all) {
      const was = prevAlive.get(a) ?? true;
      if (was && !a.alive) {
        impactPos = a.pos.clone();
        vfx.spawnExplosion(a.pos.clone(), 1 + a.fuel);
        vfx.scorchDecal(a.pos.clone());
        if (!a.crashedFromDying) {
          attachedPlumes.push(vfx.spawnSmoke(a.pos.clone(), 0xff7722, a.group));
          attachedPlumes.push(vfx.spawnSmoke(a.pos.clone(), 0x1a1a1a, a.group));
        }
        for (let i = 0; i < 4; i++) {
          const ang = Math.random() * Math.PI * 2;
          const sp = 10 + Math.random() * 20;
          vfx.spawnDebris(null, world, a.pos.clone(), new THREE.Vector3(Math.cos(ang) * sp, 5 + Math.random() * 10, Math.sin(ang) * sp));
        }
      }
      prevAlive.set(a, a.alive);
    }
  }

  // Staged fast impact (probe-crash-vfx scenario B): 10 m above flat
  // ground, nose-down 8 deg, 85 m/s forward, -16 m/s vertical, gear down.
  const by = heightAt(baseX, baseZ) + 10;
  player.resetAt(baseX, by, baseZ, fhdg);
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler((8 * Math.PI) / 180, (fhdg * Math.PI) / 180, 0, 'YXZ'));
  player.body.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
  const f = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
  player.body.setLinvel({ x: f.x * 85, y: -16, z: f.z * 85 }, true);
  player.vel.set(f.x * 85, -16, f.z * 85);
  player.gear = true;
  player.grounded = false;
  player.thrTarget = 0.6;
  player.throttle = 0.6;
  player.pitchIn = 0;
  let impactedAt = -1;
  let simT = 0;
  while (simT < 10 && impactedAt < 0) {
    liveFrame((dt) => {
      simT += dt;
      world.timestep = dt;
      world.step(eventQueue);
      drain();
      player.step(dt, WIND, null);
      vfx.update(dt);
      handleDeaths([player]);
      if (!player.alive && impactedAt < 0) impactedAt = simT;
    });
  }
  check('staged fast impact crashes through the live path', impactedAt >= 0 && (player.lastCrashReason === 'hard landing' || player.lastCrashReason === 'terrain'), `impact at sim t=${impactedAt.toFixed(2)}s reason=${player.lastCrashReason} vs=${player.lastCrashVs.toFixed(2)}m/s`);
  console.log(`wreck at (${impactPos?.x.toFixed(0)}, ${impactPos?.y.toFixed(0)}, ${impactPos?.z.toFixed(0)}) wingDetached=${player.wingDetached} tailDetached=${player.tailDetached}`);

  function sample(t: number): FieldSample {
    const V = vfx as unknown as { booms: BoomRec[]; plumes: PlumeRec[]; debris: DebrisRec[]; scorches: ScorchRec[] };
    let fireOpacity = 0;
    let attached = 0;
    for (const p of V.plumes) {
      if (!p.parent) continue;
      attached++;
      for (const child of p.group.children) {
        const spr = child as THREE.Sprite;
        fireOpacity = Math.max(fireOpacity, (spr.material as THREE.SpriteMaterial).opacity);
      }
    }
    let torn = 0;
    let resting = 0;
    let belowGround = 0;
    let maxSpread = 0;
    let scattered = 0;
    const dists: number[] = [];
    let worst = '';
    let worstH = -Infinity;
    for (const d of V.debris) {
      let isTorn = false;
      d.mesh.traverse((o) => { if (o.name === 'torn_wing' || o.name === 'torn_tail') isTorn = true; });
      if (isTorn) torn++;
      const p = d.body.translation();
      const lv = d.body.linvel();
      const av = d.body.angvel();
      const gh = heightAt(p.x, p.z);
      if (p.y < gh - 0.5) belowGround++;
      // Pass 15 coordinator: a piece counts as resting when it is
      // SETTLED (no linear/angular motion) within 3 m of the
      // ground. The old bar (centre < 1 m above ground) assumed
      // the pass-13 flat boxes; the pass-15 folded torn panels
      // legitimately rest ON the wreck (centre 1–2.4 m up, bottom
      // on the fuselage/wing) and a folded panel's centre sits
      // ~0.8 m up even on flat ground. A floating piece cannot
      // satisfy this — with no contact, gravity makes |lv| grow,
      // so settled + low is a real rest, on the ground or on the
      // wreck. (4/4 runs pre-fix: rest 2–5/6, worst settled at
      // h=1.04–2.37 m, vy=spd=ang=0.)
      const spd = Math.hypot(lv.x, lv.y, lv.z);
      const ang = Math.hypot(av.x, av.y, av.z);
      if (p.y - gh < 3 && Math.abs(lv.y) < 2 && spd < 1 && ang < 1) resting++;
      const h = p.y - gh;
      if (h > worstH) {
        worstH = h;
        worst = `h=${h.toFixed(2)}m vy=${lv.y.toFixed(2)} spd=${Math.hypot(lv.x, lv.y, lv.z).toFixed(2)} ang=${Math.hypot(av.x, av.y, av.z).toFixed(2)}`;
      }
      const dist = impactPos ? Math.hypot(p.x - impactPos.x, p.z - impactPos.z) : 0;
      dists.push(dist);
      maxSpread = Math.max(maxSpread, dist);
      if (dist > 5) scattered++;
    }
    dists.sort((a, b) => a - b);
    return {
      t, debris: V.debris.length, torn, resting, belowGround, maxSpreadM: maxSpread, scattered, dists, worstPiece: worst,
      fireOpacity, attachedPlumes: attached, scorch: V.scorches.length,
    };
  }
  const samples = new Map<number, FieldSample>();
  samples.set(0, sample(0));
  let next = 1;
  let postT = 0;
  while (postT < 91) {
    liveFrame((dt) => {
      postT += dt;
      world.timestep = dt;
      world.step(eventQueue);
      vfx.update(dt);
      if (next < SAMPLE_TIMES.length && postT >= SAMPLE_TIMES[next]! - 1e-9) {
        samples.set(SAMPLE_TIMES[next]!, sample(SAMPLE_TIMES[next]!));
        next++;
      }
    });
  }
  const at = (t: number): FieldSample => samples.get(t)!;
  console.log('field samples: ' + [0, 1.5, 10, 15, 30, 60, 75, 89, 91].map((t) => {
    const s = at(t);
    return `t+${t}s debris=${s.debris}(torn${s.torn}) rest=${s.resting}/${s.debris} below=${s.belowGround} spread=${s.maxSpreadM.toFixed(0)}m scat>${5}m:${s.scattered} worst[${s.worstPiece}] fireOp=${s.fireOpacity.toFixed(2)} att=${s.attachedPlumes} scorch=${s.scorch}`;
  }).join(' | '));
  console.log(`piece distances from the wreck at +30 s: [${at(30).dists.map((d) => d.toFixed(0)).join(', ')}] m`);
  console.log(`piece distances from the wreck at +75 s: [${at(75).dists.map((d) => d.toFixed(0)).join(', ')}] m`);
  check('debris field present at +1.5 s (4 boxes + torn wing/tail)', at(1.5).debris >= 5, `debris=${at(1.5).debris} torn=${at(1.5).torn}`);
  check('no debris piece fell through the terrain at +10 s', at(10).belowGround === 0, `below=${at(10).belowGround}`);
  check('debris settled at the crash site at +30 s (on the ground or on the wreck — pass-15 folded panels rest on the wreck)', at(30).resting === at(30).debris && at(30).debris > 0, `resting=${at(30).resting}/${at(30).debris} worst[${at(30).worstPiece}]`);
  check('debris field is scattered, not clumped under the wreck (max spread >= 5 m, >= 2 pieces beyond 5 m)', at(10).maxSpreadM >= 5 && at(10).scattered >= 2, `spread=${at(10).maxSpreadM.toFixed(1)}m scattered=${at(10).scattered}`);
  check('debris field STILL present at +30 sim-s — the live +75 s real observation point at 4 fps (old 30 s life: gone)', at(30).debris >= 4, `debris=${at(30).debris}`);
  check('debris field STILL present at +75 sim-s', at(75).debris >= 4, `debris=${at(75).debris}`);
  check('debris field STILL present at +89 sim-s (90 s life)', at(89).debris >= 1, `debris=${at(89).debris}`);
  check('debris field expired by +91 sim-s (it does not linger forever)', at(91).debris === 0, `debris=${at(91).debris}`);
  check('fire still visibly burning at +30 sim-s (old 30 s plumes: gone)', at(30).attachedPlumes >= 1 && at(30).fireOpacity >= 0.4, `attached=${at(30).attachedPlumes} op=${at(30).fireOpacity.toFixed(2)}`);
  check('fire still visibly burning at +75 sim-s (old: long gone)', at(75).attachedPlumes >= 1 && at(75).fireOpacity >= 0.4, `attached=${at(75).attachedPlumes} op=${at(75).fireOpacity.toFixed(2)}`);
  check('scorch mark present at +30 sim-s', at(30).scorch >= 1, `scorch=${at(30).scorch}`);

  console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
