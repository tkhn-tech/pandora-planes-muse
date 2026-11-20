// probe-spectate-orbit.ts — pass 11 (wreck free orbit).
//
// Nicholas: "when watching the wreck, the camera should be the free
// orbit." The old code returned the fixed 42 m velocity-chase from
// the spectating branch BEFORE checking camMode, so camMode 2 never
// applied while spectating despite the comment claiming it did. The
// fix: spectate engages with camMode = 2 (framed at the 30 m /
// 0.32 pitch default), updateCamera's spectating branch runs the
// free-orbit math around the wreck when camMode === 2, and
// skipSpectate restores the mode the player had before the wreck.
// R / the Respawn button still skip (unchanged path).
//
// This probe drives the pure orbit helpers (src/camera/OrbitCamera)
// that updateCamera calls: orbit input (yaw) moves the camera around
// the wreck, zoom input (dist) changes the distance, the camera
// stays focused on the wreck, the terrain floor keeps the wreck
// visible, and the main.ts wiring (spectate branch routes camMode 2
// through the orbit helpers; engage sets camMode 2; skip restores)
// is present. A live visual check (parent's browser check) still
// owns the rendered result.
//
// Usage: node_modules/.bin/esbuild scripts/probe-spectate-orbit.ts
//   --bundle --platform=node --format=esm --outfile=/tmp/probe-spectate-orbit.mjs
//   && node /tmp/probe-spectate-orbit.mjs

import * as THREE from 'three';
import * as fs from 'node:fs';
import {
  applyCameraFloor,
  orbitCameraPosition,
} from '../src/camera/OrbitCamera';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

const wreck = new THREE.Vector3(100, 350, -200);
const flat = () => 0;

function main(): void {
  // --- Orbit position: yaw 0 = south of the wreck (+Z), yaw +90 deg
  // = east (+X), yaw 180 deg = north (-Z); distance == dist.
  const p0 = orbitCameraPosition(wreck, { yaw: 0, pitch: 0.32 }, 30);
  check(
    'orbit yaw 0 sits south of the wreck at the orbit distance',
    Math.abs(p0.x - wreck.x) < 1e-6 &&
      Math.abs(p0.z - (wreck.z + Math.cos(0.32) * 30)) < 1e-6 &&
      Math.abs(p0.distanceTo(wreck) - 30) < 1e-6,
    `pos=(${p0.x.toFixed(2)}, ${p0.y.toFixed(2)}, ${p0.z.toFixed(2)})`,
  );
  const pE = orbitCameraPosition(wreck, { yaw: Math.PI / 2, pitch: 0.32 }, 30);
  check(
    'orbit yaw +90 deg sits east of the wreck',
    pE.x > wreck.x + 20 && Math.abs(pE.z - wreck.z) < 1e-6,
    `pos=(${pE.x.toFixed(2)}, ${pE.y.toFixed(2)}, ${pE.z.toFixed(2)})`,
  );
  const pN = orbitCameraPosition(wreck, { yaw: Math.PI, pitch: 0.32 }, 30);
  check(
    'orbit yaw 180 deg sits north of the wreck',
    pN.z < wreck.z - 20 && Math.abs(pN.x - wreck.x) < 1e-6,
    `pos=(${pN.x.toFixed(2)}, ${pN.y.toFixed(2)}, ${pN.z.toFixed(2)})`,
  );

  // --- Drag changes the angle: yaw +0.5 rad swings the camera's
  // azimuth around the wreck by the same 0.5 rad.
  const az = (p: THREE.Vector3): number =>
    Math.atan2(p.x - wreck.x, p.z - wreck.z);
  const dAz = az(orbitCameraPosition(wreck, { yaw: 0.5, pitch: 0.32 }, 30)) - az(p0);
  check(
    'orbit drag (yaw +0.5) swings the camera azimuth by +0.5 rad',
    Math.abs(dAz - 0.5) < 1e-6,
    `dAz=${dAz.toFixed(4)}`,
  );
  // --- Pitch changes the elevation: higher pitch = higher camera.
  const pHi = orbitCameraPosition(wreck, { yaw: 0, pitch: 0.8 }, 30);
  check(
    'orbit pitch up raises the camera',
    pHi.y > p0.y + 5,
    `y ${p0.y.toFixed(2)} -> ${pHi.y.toFixed(2)}`,
  );

  // --- Zoom changes the distance: 30 -> 60 doubles it.
  const pFar = orbitCameraPosition(wreck, { yaw: 0, pitch: 0.32 }, 60);
  check(
    'orbit zoom (dist 30 -> 60) doubles the camera distance',
    Math.abs(pFar.distanceTo(wreck) - 60) < 1e-6,
    `dist=${pFar.distanceTo(wreck).toFixed(2)}`,
  );

  // --- Focus: the camera looks at the wreck — the wreck is on the
  // lookAt axis by construction (offset length == dist, direction
  // from camera to wreck is the view axis).
  const toWreck = wreck.clone().sub(p0).normalize();
  check(
    'camera offset points from the wreck to the camera (lookAt centres the wreck)',
    Math.abs(p0.clone().sub(wreck).length() - 30) < 1e-6 && toWreck.length() > 0.99,
    `offsetLen=${p0.clone().sub(wreck).length().toFixed(2)}`,
  );

  // --- Terrain floor: a ridge under the camera raises it; flat
  // ground leaves it alone; never below wreck + 2 m.
  const ridge = (x: number, z: number): number =>
    Math.abs(x - p0.x) < 5 && Math.abs(z - p0.z) < 5 ? 500 : 0;
  const raised = applyCameraFloor(p0.clone(), wreck, ridge);
  check(
    'orbit floor raises the camera above a ridge under it',
    raised.y >= 503,
    `y ${p0.y.toFixed(2)} -> ${raised.y.toFixed(2)}`,
  );
  const kept = applyCameraFloor(p0.clone(), wreck, flat);
  check(
    'orbit floor leaves the camera alone over flat ground',
    Math.abs(kept.y - p0.y) < 1e-6,
    `y=${kept.y.toFixed(2)}`,
  );
  const low = orbitCameraPosition(wreck, { yaw: 0, pitch: -0.1 }, 30);
  const floored = applyCameraFloor(low.clone(), wreck, flat);
  check(
    'orbit floor never drops the camera below wreck + 2 m',
    floored.y >= wreck.y + 2 - 1e-6,
    `y=${floored.y.toFixed(2)} wreck+2=${wreck.y + 2}`,
  );

  // --- Wiring in main.ts: the spectating branch routes camMode 2
  // through the orbit helpers, spectate engages with camMode 2, and
  // skipSpectate restores the pre-spectate mode.
  let src = '';
  for (const p of ['src/main.ts', '/home/hatch/workspace/pandora-planes-vite/src/main.ts']) {
    try { src = fs.readFileSync(p, 'utf8'); break; } catch { /* try next */ }
  }
  check('main.ts readable for the wiring check', src.length > 0, `${src.length} bytes`);
  const spectateBlock = src.slice(src.indexOf('if (spectating) {'), src.indexOf('if (camMode === 0) {'));
  check(
    'spectating branch runs the free orbit when camMode === 2',
    spectateBlock.includes('camMode === 2') &&
      spectateBlock.includes('orbitCameraPosition') &&
      spectateBlock.includes('applyCameraFloor'),
    'spectate block routes camMode 2 through the orbit helpers',
  );
  check(
    'spectate engages with camMode = 2 (wreck view IS the free orbit)',
    (src.match(/preSpectateCamMode = camMode;\s*\n\s*camMode = 2;/g) ?? []).length >= 2,
    'both engage sites (dying + instant crash) set camMode 2',
  );
  check(
    'skipSpectate restores the pre-spectate camera mode',
    src.includes('camMode = preSpectateCamMode;'),
    'restore present',
  );

  console.log(failures === 0 ? '\nALL SPECTATE-ORBIT CHECKS PASSED' : `\n${failures} SPECTATE-ORBIT CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main();
