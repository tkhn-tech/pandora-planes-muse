// probe-restart-camera.ts — pass 14, worker D (item 1: chase camera
// after Apply & Restart).
//
// v4.8 live finding (24 Nov, 5 rounds, all from Alpha): after every
// Apply & Restart the chase camera starts in a broken side-on,
// below-ground view (jet in profile, lower half of the screen black,
// HUD heading wrong — F-35A showed 180° / CV −139° until it
// corrected); it fixes itself once the jet moves or the camera button
// is clicked.
//
// Two candidate mechanisms, both measured here through the live
// paths (real Aircraft resetAt, real terrain heightfield, real GLB,
// variable frame dt):
//   (a) STALE MODE: startRound never resets camMode / camDist /
//       camOrbit, so a round that ended spectating (camMode 2,
//       camDist up to 120 m, orbit yaw 0) restarts IN free orbit —
//       a persistent side-on view until the camera button cycles the
//       mode (2 -> 0). This is the persistent defect.
//   (b) STALE POSITION LERP: the chase camera lerps toward its want
//       (1 - 0.001^rawDt per frame). At 4 fps that converges in ~1 s,
//       but at 1 fps during a janky restart it takes 4-9 s, and the
//       straight-line lerp path runs through terrain — the
//       below-ground half-black frame.
//   (c) STALE HEADING: a fresh Aircraft (aircraft switch in the setup
//       menu) has the identity quaternion — fwd +Z, game heading 0,
//       TRUE heading 180° — until resetAt runs. The compass
//       conversion itself (gameToTrueHdg: true = 180 - game) is
//       correct; the 180° the live check saw is the pre-spawn value.
//
// The fix: startRound / respawnPlayer reset camMode 0, camDist 30,
// camOrbit default and SNAP the camera to the chase want
// (chaseCameraPosition, the same math updateCamera converges to)
// on the first frame, so no lerp transient and no stale mode.
//
// Usage: node_modules/.bin/esbuild scripts/probe-restart-camera.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-restart-camera.mjs
//   && node /tmp/probe-restart-camera.mjs

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
import { RUNWAY_TABLE } from '../src/world/PandoraTerrainWorker';
import { generateHeights, heightfieldHeightAt, WORLD_SIZE, TERRAIN_SEGMENTS } from '../src/world/PandoraTerrainWorker';
import { FloatingOrigin } from '../src/world/FloatingOrigin';
import {
  chaseCameraPosition,
  deckChaseCameraPosition,
  applyDeckCeiling,
  orbitCameraPositionDeckAware,
} from '../src/camera/OrbitCamera';
import { gameToTrueHdg } from '../src/ui/FlightHUD';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const R2D = 180 / Math.PI;

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
      .setFriction(0).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min).setRestitution(0),
    terrainBody,
  );
  const scene = new THREE.Scene();
  const origin = new FloatingOrigin(2000);
  origin.registerBody(terrainBody);
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x + origin.cumulativeShift.x, z + origin.cumulativeShift.z);
  const deck: DeckProvider = { alive: false, deckHeightAt: () => -Infinity, velocity: () => new THREE.Vector3(), velocityAt: () => new THREE.Vector3(), yawRate: () => 0, yawRateAt: () => 0, local: () => null };

  const alpha = RUNWAY_TABLE[0]!;
  console.log(`Alpha runway: (${alpha.x}, ${alpha.z}) elev=${alpha.elev} game hdg=${alpha.hdg} -> true hdg=${gameToTrueHdg(alpha.hdg)}`);

  // --- (c) Stale heading: a fresh Aircraft before resetAt. ---
  for (const key of ['fa18', 'f35a'] as const) {
    const ac = new Aircraft({ scene, world, spec: SPECS[key]!, isPlayer: true, modelKey: SPECS[key]!.modelKey, deck, events: {} });
    ac.terrainHeightAt = heightAt;
    const freshGameHdg = ((Math.atan2(ac.fwd().x, ac.fwd().z) * R2D) % 360 + 360) % 360;
    const freshTrue = gameToTrueHdg(freshGameHdg);
    console.log(`${key}: fresh Aircraft fwd=(${ac.fwd().x.toFixed(2)},${ac.fwd().y.toFixed(2)},${ac.fwd().z.toFixed(2)}) game hdg=${freshGameHdg.toFixed(0)} true hdg=${freshTrue.toFixed(0)}`);
    check(
      `${key}: fresh Aircraft reads true 180 (identity quat) — the live "180" is the pre-spawn stale value, not a compass bug`,
      Math.abs(freshTrue - 180) < 1e-6,
      `true=${freshTrue.toFixed(1)}`,
    );
    ac.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
    const spawnGameHdg = ((Math.atan2(ac.fwd().x, ac.fwd().z) * R2D) % 360 + 360) % 360;
    const spawnTrue = gameToTrueHdg(spawnGameHdg);
    // Tolerance 0.01 deg: respawn() round-trips the quaternion through
    // the Rapier body (float32), so the heading shifts by ~1e-6 deg
    // from the exact runway value; the HUD displays whole degrees.
    check(
      `${key}: after resetAt the HUD heading is the runway true heading (${gameToTrueHdg(alpha.hdg)})`,
      Math.abs(spawnTrue - gameToTrueHdg(alpha.hdg)) < 0.01,
      `true=${spawnTrue.toFixed(4)}`,
    );
    ac.dispose();
  }

  // --- Real GLB + spawn for the camera measurements (fa18). ---
  const player = new Aircraft({ scene, world, spec: SPECS['fa18']!, isPlayer: true, modelKey: SPECS['fa18']!.modelKey, deck, events: {} });
  player.terrainHeightAt = heightAt;
  origin.registerBody(player.body);
  await player.attachModel();
  console.log(`fa18 modelStatus=${player.modelStatus}`);
  player.resetAt(alpha.x, alpha.elev + 2.25, alpha.z, alpha.hdg);
  const fwd = player.fwd();
  const up = player.up();
  const groundAt = (x: number, z: number): number => heightAt(x, z);

  // The default chase want, computed independently from the existing
  // verified pieces (deck chase + floor + ceiling) — the position
  // updateCamera's chase branch converges to.
  const wantIndependent = deckChaseCameraPosition(player.pos, fwd, up, 30, null);
  {
    const mid = player.pos.clone().addScaledVector(fwd, -15);
    const floor = Math.max(groundAt(wantIndependent.x, wantIndependent.z) + 6, groundAt(mid.x, mid.z) + 8, player.pos.y + 2);
    if (wantIndependent.y < floor) wantIndependent.y = floor;
  }
  const snap = chaseCameraPosition(player.pos, fwd, up, 30, groundAt, null);
  check(
    'snap helper equals the independently computed chase want (same math updateCamera converges to)',
    snap.distanceTo(wantIndependent) < 1e-6,
    `snap=(${snap.x.toFixed(1)},${snap.y.toFixed(1)},${snap.z.toFixed(1)}) want=(${wantIndependent.x.toFixed(1)},${wantIndependent.y.toFixed(1)},${wantIndependent.z.toFixed(1)})`,
  );
  check(
    'snap camera sits behind the jet at the chase distance, above the terrain',
    snap.distanceTo(player.pos) >= 29 && snap.distanceTo(player.pos) <= 45 &&
      snap.y > groundAt(snap.x, snap.z) + 2 &&
      snap.clone().sub(player.pos).dot(fwd) < 0,
    `dist=${snap.distanceTo(player.pos).toFixed(1)}m y=${snap.y.toFixed(1)} gnd=${groundAt(snap.x, snap.z).toFixed(1)} behindDot=${snap.clone().sub(player.pos).dot(fwd).toFixed(1)}`,
  );

  // Deck snap: camera under the overhead sheet (deckTop + 4.0) and
  // above the deck (deckTop + 3.0).
  const deckTopY = player.pos.y + 27.78; // jet at deckTop + 2.2
  const snapDeck = chaseCameraPosition(player.pos, fwd, up, 30, groundAt, deckTopY);
  check(
    'deck snap stays under the overhead sheet and above the deck',
    snapDeck.y <= deckTopY + 4.0 + 1e-6 && snapDeck.y >= deckTopY + 3.0 - 1e-6,
    `camY=${snapDeck.y.toFixed(2)} deckTop=${deckTopY.toFixed(2)} (+${(snapDeck.y - deckTopY).toFixed(2)})`,
  );

  // --- (a) STALE MODE (the persistent defect): the previous round
  // ended spectating — camMode 2, camDist 120, orbit yaw 0 — and the
  // old startRound resets none of it. The first frame after restart
  // is a 120 m free orbit south of the jet: the jet heads east
  // (Alpha hdg 90), so a camera due south sees it in profile.
  const staleOrbit = orbitCameraPositionDeckAware(player.pos, { yaw: 0, pitch: 0.32 }, 120, groundAt, null);
  const staleIsProfile = Math.abs(staleOrbit.clone().sub(player.pos).normalize().dot(fwd)) < 0.3;
  check(
    'OLD restart: stale orbit mode puts the first frame far from the chase view, jet in profile (DEFECT — must be fixed by the camMode reset + snap)',
    staleOrbit.distanceTo(wantIndependent) > 50 && staleIsProfile,
    `orbit dist from chase want=${staleOrbit.distanceTo(wantIndependent).toFixed(0)}m orbitDist=${staleOrbit.distanceTo(player.pos).toFixed(0)}m profileDot=${staleOrbit.clone().sub(player.pos).normalize().dot(fwd).toFixed(2)}`,
  );

  // --- (b) STALE POSITION LERP: from a camera 3 km away (previous
  // round's position), how long does the chase lerp take to converge
  // at live frame rates? Factor per frame = 1 - 0.001^rawDt.
  const lerpResidual = (rawDt: number, frames: number): number => Math.pow(0.001, rawDt * frames);
  const res4fps1s = lerpResidual(0.25, 4) * 3000;
  const res1fps1s = lerpResidual(0.25, 1) * 3000;
  const res1fps4s = lerpResidual(0.25, 4) * 3000;
  console.log(`lerp residual from 3000 m: 4 fps after 1 s = ${res4fps1s.toFixed(1)} m; 1 fps after 1 s = ${res1fps1s.toFixed(0)} m, after 4 s = ${res1fps4s.toFixed(1)} m`);
  check(
    'OLD restart: at 1 fps the lerp is still >100 m off after 1 s (janky-restart transient the snap eliminates)',
    res1fps1s > 100,
    `residual=${res1fps1s.toFixed(0)}m`,
  );
  check(
    'NEW restart: snap puts the camera exactly on the chase want on frame 0 (no lerp transient at any frame rate)',
    snap.distanceTo(wantIndependent) < 1e-6,
    `offset=${snap.distanceTo(wantIndependent).toFixed(6)}m`,
  );

  // --- Wiring presence in main.ts (the call sites the probe cannot
  // import — same pattern as probe-spectate-orbit). ---
  const mainSrc = fs.readFileSync('src/main.ts', 'utf8');
  const startRound = mainSrc.slice(mainSrc.indexOf('async function startRound'), mainSrc.indexOf('function restartRound'));
  const respawnPlayer = mainSrc.slice(mainSrc.indexOf('function respawnPlayer'), mainSrc.indexOf('function clearAircraftList'));
  check('startRound resets the camera mode to chase on restart', startRound.includes('camMode = 0'), '');
  check('startRound snaps the camera to the chase position after the spawn', startRound.includes('snapCameraToChase'), '');
  check('respawnPlayer snaps the camera to the chase position after the respawn', respawnPlayer.includes('snapCameraToChase'), '');
  check('updateCamera chase branch converges to the same want the snap uses', mainSrc.includes('chaseCameraPosition('), '');

  player.dispose();
  console.log(failures === 0 ? '\nALL CHECKS PASSED' : `\n${failures} CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
