// probe-cockpit.ts — v4.11 (cockpit REMOVAL). Nicholas, 2026-11-25:
// "for ALL planes, remove the ugly extra additions on the nose.
// remove cockpit view".
//
// The "extra additions on the nose" are the procedural cockpit kit
// CockpitBuilder.buildCockpit() mounted on every player jet
// (instrument panel slab, dials, MFD screens, HUD combiner glass,
// seat, headrest, helmet, coaming) — lumps that sit forward/on top
// of the nose-canopy area and read as ugly additions from the
// external/chase view. The cockpit camera mode goes with it.
//
// End state this probe proves, per player jet (fa18, f22, f15e,
// f35a) through the real attachModel path (real GLBs, Image stub),
// plus one enemy jet (mig29, never had a cockpit — must be
// untouched):
//  - the model loads (modelStatus ok);
//  - NO object named cockpit_* exists anywhere in the rig group
//    (the procedural kit is gone; the GLBs' OWN canopy nodes —
//    e.g. the F-18's `Canopy` — stay, they are part of the model);
//  - NO HUD reticle anywhere (the 2026-11-20 "Remove this from ALL
//    JETS" removal still holds);
//  - the canopy glass fix SURVIVES (fixCanopyMaterials still runs
//    after attachModel for player, enemies and traffic — it touches
//    the GLB's OWN canopy meshes, making them transparent glass
//    instead of black metal; removing it would regress the external
//    look. It now lives in src/aircraft/CanopyMaterials.ts);
//  - gear is untouched (rig bellyY in model units, nose leg
//    jet-scale — the gear is NOT a nose addition);
//  - no cockpit part protrudes at spine height (side rays).
// And at source level:
//  - src/aircraft/CockpitBuilder.ts is gone (no buildCockpit, no
//    COCKPIT_EYE, no cockpitEyeFor, no cockpitCameraQuaternion,
//    no COCKPIT_PITCH_DOWN_DEG anywhere in src/ or index.html);
//  - camMode is 0 chase / 1 free-orbit only: cycleCam wraps % 2,
//    no 'camMode === 2' / 'camMode = 2' remains in main.ts,
//    spectate engages with camMode = 1 (both sites);
//  - PCFlightInput / FlightHUD / index.html / OrbitCamera /
//    TerrainStreamer / AircraftRigBindings carry no cockpit
//    reference; cameraToast names are CHASE CAM / FREE ORBIT.
// And the surviving cameras still work numerically:
//  - chaseCameraPosition puts the camera behind the jet, above the
//    ground, ~30 m out; orbitCameraPositionDeckAware puts it ~30 m
//    from the target, above target + 2 m.
//
// Usage: node_modules/.bin/esbuild scripts/probe-cockpit.ts --bundle
//   --platform=node --format=esm --outfile=/tmp/probe-cockpit.mjs
//   && node /tmp/probe-cockpit.mjs

if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type: string;
    lengthComputable = false;
    loaded = 0;
    total = 0;
    constructor(type: string, init?: { lengthComputable?: boolean; loaded?: number; total?: number }) {
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
import * as fs from 'node:fs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { fixCanopyMaterials } from '../src/aircraft/CanopyMaterials';
import {
  chaseCameraPosition,
  orbitCameraPositionDeckAware,
} from '../src/camera/OrbitCamera';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

// Reticle names that must NOT exist anywhere in the aircraft group
// (2026-11-20, Nicholas: "Remove this from ALL JETS" — the green
// crosshair circle on the canopy in the external/chase view).
const RETICLE_NAMES = [
  'cockpit_hud_reticle',
  'cockpit_hud_bar_h',
  'cockpit_hud_bar_v',
  'cockpit_hud_crosshair',
  'cockpit_ccip',
  'cockpit_pipper',
  'cockpit_hud_pipper',
];

async function main(): Promise<void> {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const scene = new THREE.Scene();
  const wind = new THREE.Vector3(0, 0, 0);

  // --- Player jets: attachModel + the remaining main.ts post-attach
  // step (fixCanopyMaterials only — buildCockpit is gone in v4.11).
  for (const key of ['fa18', 'f22', 'f15e', 'f35a']) {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    ac.terrainHeightAt = () => 0;
    ac.resetAt(0, 600, 0, 0);
    await ac.attachModel();
    check(
      `${key} real GLB loads (modelStatus ok)`,
      ac.modelStatus === 'ok',
      `modelStatus=${ac.modelStatus}`,
    );
    if (ac.modelStatus !== 'ok') {
      ac.dispose();
      continue;
    }
    world.timestep = 1 / 60;
    world.step(new RAPIER.EventQueue(true));
    ac.step(1 / 60, wind, null);
    scene.updateMatrixWorld(true);
    const fixed = fixCanopyMaterials(ac.group);
    if (ac.rig) {
      ac.rig.gearDeploy = 0;
      ac.rig.update(
        0,
        { pitch: 0, roll: 0, yaw: 0, throttle: null },
        { aoaDeg: 0, gear: false, hook: false, vectorDeg: 0, rotorRpm: 0 },
      );
    }
    scene.updateMatrixWorld(true);

    // --- NO procedural cockpit parts anywhere in the rig group.
    // The GLBs' own canopy nodes (e.g. F-18 `Canopy`) are NOT
    // named cockpit_* and stay — only the added kit is gone.
    const cockpitParts: string[] = [];
    ac.group.traverse((o) => {
      if (/cockpit/i.test(o.name)) cockpitParts.push(o.name);
    });
    check(
      `${key} NO procedural cockpit parts on the jet (panel/dials/screens/HUD/seat/headrest gone)`,
      cockpitParts.length === 0,
      cockpitParts.length ? `still present: ${[...new Set(cockpitParts)].join(', ')}` : 'no cockpit_* objects',
    );

    // --- No HUD reticle anywhere (2026-11-20 removal still holds).
    const reticleFound: string[] = [];
    ac.group.traverse((o) => {
      if (RETICLE_NAMES.includes(o.name)) reticleFound.push(o.name);
    });
    check(
      `${key} NO HUD reticle present`,
      reticleFound.length === 0,
      reticleFound.length ? `found: ${reticleFound.join(', ')}` : 'no reticle objects',
    );

    // --- Rig belly (gear untouched by the removal): bellyY must be
    // in MODEL units, and the nose gear leg jet-scale.
    {
      const bellyY = ac.rig?.bellyY;
      check(
        `${key} rig bellyY is in model units (gear untouched)`,
        bellyY !== undefined && bellyY > -2 && bellyY < 0.5,
        `bellyY=${bellyY?.toFixed(2) ?? 'n/a'}`,
      );
      const leg = ac.group.getObjectByName('gear_leg_gear_front');
      if (leg) {
        const lbb = new THREE.Box3().setFromObject(leg);
        const h = lbb.max.y - lbb.min.y;
        check(
          `${key} nose gear leg is jet-scale, not a sky spike`,
          h < 5,
          `leg height=${h.toFixed(2)}m`,
        );
      }
    }

    // --- Canopy glass survives (the GLB's OWN canopy, made
    // transparent — not an addition, and removing it would bring
    // back the black metal canopy).
    const canopies: THREE.Mesh[] = [];
    ac.group.traverse((o) => {
      if ((o as THREE.Mesh).isMesh && /canop/i.test(o.name)) {
        canopies.push(o as THREE.Mesh);
      }
    });
    if (key === 'f35a') {
      check(
        'f35a has NO canopy mesh in its GLB (unchanged)',
        canopies.length === 0,
        `canopy meshes=${canopies.length}`,
      );
    } else {
      const glass = canopies.every((m) => {
        const mat = m.material as THREE.MeshStandardMaterial;
        return mat.transparent && Math.abs(mat.opacity - 0.28) < 0.01 && mat.metalness === 0;
      });
      check(
        `${key} canopy is still transparent glass (fix preserved)`,
        canopies.length >= 1 && glass,
        `canopies=${canopies.length} fixed=${fixed}`,
      );
    }

    // --- Nothing procedural protrudes at spine height: side rays
    // from outside must hit airframe/canopy or nothing — never a
    // cockpit part (and, with the kit gone, never any added part).
    {
      const allMeshes: THREE.Mesh[] = [];
      ac.group.traverse((o) => {
        if ((o as THREE.Mesh).isMesh && o.visible) allMeshes.push(o as THREE.Mesh);
      });
      const ray = new THREE.Raycaster();
      // Eye height is gone with the cockpit; use the canopy/nose
      // station instead: 1.2 m above the jet centre at the nose
      // (z +5), both sides.
      const shots: Array<[string, THREE.Vector3, THREE.Vector3]> = [
        ['port', new THREE.Vector3(ac.pos.x - 10, ac.pos.y + 1.2, ac.pos.z + 5), new THREE.Vector3(ac.pos.x, ac.pos.y + 1.2, ac.pos.z + 5)],
        ['starboard', new THREE.Vector3(ac.pos.x + 10, ac.pos.y + 1.2, ac.pos.z + 5), new THREE.Vector3(ac.pos.x, ac.pos.y + 1.2, ac.pos.z + 5)],
      ];
      for (const [side, from, to] of shots) {
        const dir = to.clone().sub(from).normalize();
        ray.set(from, dir);
        ray.far = 30;
        const hits = ray.intersectObjects(allMeshes, false);
        const first = hits[0]?.object.name ?? 'NO HIT';
        check(
          `${key} ${side} ray at nose height hits airframe/canopy, not a cockpit part`,
          hits.length === 0 || !/cockpit/i.test(first),
          `first hit: ${first}`,
        );
      }
    }
    ac.dispose();
  }

  // --- Enemy jet (mig29): never had a cockpit; the removal must
  // not touch the enemy path (attachModel + fixCanopyMaterials,
  // the same calls main.ts makes for enemies).
  {
    const spec = SPECS['mig29']!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: false,
      modelKey: spec.modelKey,
    });
    ac.terrainHeightAt = () => 0;
    ac.resetAt(0, 600, 0, 0);
    await ac.attachModel();
    check(
      'mig29 real GLB loads (modelStatus ok)',
      ac.modelStatus === 'ok',
      `modelStatus=${ac.modelStatus}`,
    );
    if (ac.modelStatus === 'ok') {
      world.step(new RAPIER.EventQueue(true));
      ac.step(1 / 60, wind, null);
      scene.updateMatrixWorld(true);
      fixCanopyMaterials(ac.group);
      scene.updateMatrixWorld(true);
      const cockpitParts: string[] = [];
      ac.group.traverse((o) => {
        if (/cockpit/i.test(o.name)) cockpitParts.push(o.name);
      });
      check(
        'mig29 has NO cockpit parts (enemy path untouched)',
        cockpitParts.length === 0,
        cockpitParts.length ? `found: ${[...new Set(cockpitParts)].join(', ')}` : 'no cockpit_* objects',
      );
    }
    ac.dispose();
  }

  // --- Source-level removal checks (run from the project root).
  const read = (p: string): string => {
    try {
      return fs.readFileSync(p, 'utf8');
    } catch {
      return '';
    }
  };
  const mainSrc = read('src/main.ts');
  const pcSrc = read('src/input/PCFlightInput.ts');
  const hudSrc = read('src/ui/FlightHUD.ts');
  const html = read('index.html');
  const orbitSrc = read('src/camera/OrbitCamera.ts');
  const streamSrc = read('src/world/TerrainStreamer.ts');
  const rigSrc = read('src/aircraft/AircraftRigBindings.ts');

  check(
    'src/aircraft/CockpitBuilder.ts is gone',
    !fs.existsSync('src/aircraft/CockpitBuilder.ts'),
    fs.existsSync('src/aircraft/CockpitBuilder.ts') ? 'file still exists' : 'deleted',
  );
  check(
    'main.ts never builds a cockpit (no buildCockpit call)',
    mainSrc.length > 0 && !mainSrc.includes('buildCockpit'),
    mainSrc.length === 0 ? 'src/main.ts unreadable' : 'no buildCockpit',
  );
  check(
    'main.ts has no cockpit camera code (no cockpitEyeFor / cockpitCameraQuaternion / CockpitBuilder)',
    mainSrc.length > 0 &&
      !mainSrc.includes('cockpitEyeFor') &&
      !mainSrc.includes('cockpitCameraQuaternion') &&
      !mainSrc.includes('CockpitBuilder'),
    mainSrc.length === 0 ? 'src/main.ts unreadable' : 'no cockpit camera refs',
  );
  check(
    'camera cycle wraps over 2 modes (chase / free-orbit)',
    mainSrc.includes('camMode = (camMode + 1) % 2;'),
    'cycleCam % 2 present',
  );
  check(
    "no 'camMode === 2' / 'camMode = 2' remains in main.ts (free-orbit is 1)",
    mainSrc.length > 0 &&
      !mainSrc.includes('camMode === 2') &&
      !mainSrc.includes('camMode = 2'),
    'no camMode 2',
  );
  check(
    'spectate engages with camMode = 1 at both sites (wreck view IS the free orbit)',
    (mainSrc.match(/preSpectateCamMode = camMode;\s*\n\s*camMode = 1;/g) ?? []).length >= 2,
    'both engage sites set camMode 1',
  );
  const spectateBlock = mainSrc.slice(
    mainSrc.indexOf('if (spectating) {'),
    mainSrc.indexOf('if (camMode === 0) {'),
  );
  check(
    'spectating branch runs the free orbit when camMode === 1',
    spectateBlock.includes('camMode === 1') &&
      spectateBlock.includes('orbitCameraPositionDeckAware'),
    'spectate block routes camMode 1 through the orbit helper',
  );
  check(
    'PCFlightInput carries no cockpit reference',
    pcSrc.length > 0 && !/cockpit/i.test(pcSrc),
    pcSrc.length === 0 ? 'PCFlightInput unreadable' : 'no cockpit',
  );
  check(
    'PCFlightInput resets the yoke in free-orbit (mode 1)',
    pcSrc.includes('if (mode === 1) {') || pcSrc.includes('this.camMode === 1'),
    'mode-1 yoke handling present',
  );
  check(
    "FlightHUD camera toast names are CHASE CAM / FREE ORBIT only",
    hudSrc.length > 0 &&
      !hudSrc.includes('COCKPIT CAM') &&
      hudSrc.includes('FREE ORBIT'),
    hudSrc.length === 0 ? 'FlightHUD unreadable' : 'no COCKPIT CAM',
  );
  check(
    'index.html help text has no Cockpit camera',
    html.length > 0 && !/cockpit/i.test(html),
    html.length === 0 ? 'index.html unreadable' : 'no cockpit',
  );
  check(
    'OrbitCamera.ts carries no cockpit reference',
    orbitSrc.length > 0 && !/cockpit/i.test(orbitSrc),
    orbitSrc.length === 0 ? 'OrbitCamera unreadable' : 'no cockpit',
  );
  check(
    'TerrainStreamer.ts carries no cockpit reference',
    streamSrc.length > 0 && !/cockpit/i.test(streamSrc),
    streamSrc.length === 0 ? 'TerrainStreamer unreadable' : 'no cockpit',
  );
  check(
    'AircraftRigBindings.ts carries no cockpit reference',
    rigSrc.length > 0 && !/cockpit/i.test(rigSrc),
    rigSrc.length === 0 ? 'AircraftRigBindings unreadable' : 'no cockpit',
  );

  // --- Surviving cameras still work numerically (same helpers
  // updateCamera uses for chase camMode 0 and free-orbit camMode 1).
  {
    const jet = new THREE.Vector3(0, 600, 0);
    const fwd = new THREE.Vector3(0, 0, 1);
    const up = new THREE.Vector3(0, 1, 0);
    const groundAt = () => 50;
    const chase = chaseCameraPosition(jet, fwd, up, 30, groundAt, null);
    const behind = chase.clone().sub(jet).dot(fwd);
    check(
      'chase camera sits behind the jet, above ground, ~30 m out',
      behind < -20 && behind > -45 && chase.y > 50 && chase.distanceTo(jet) > 25 && chase.distanceTo(jet) < 45,
      `offset along fwd=${behind.toFixed(1)}m y=${chase.y.toFixed(1)} dist=${chase.distanceTo(jet).toFixed(1)}m`,
    );
    const orbit = orbitCameraPositionDeckAware(jet, { yaw: 0, pitch: 0.32 }, 30, groundAt, null);
    check(
      'free-orbit camera sits ~30 m from the jet, above target + 2 m',
      orbit.distanceTo(jet) > 28 && orbit.distanceTo(jet) < 35 && orbit.y >= jet.y + 2 - 1e-6,
      `dist=${orbit.distanceTo(jet).toFixed(1)}m y=${orbit.y.toFixed(1)}`,
    );
  }

  console.log(failures === 0 ? '\nALL COCKPIT CHECKS PASSED' : `\n${failures} COCKPIT CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('PROBE ERROR', e);
  process.exit(2);
});
