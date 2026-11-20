// probe-cockpit.ts — pass 11, item 2: cockpit parity. ALL FOUR player
// jets must show a cockpit to the F/A-18F's standard from the cockpit
// camera, and the F-15E's black screen must be gone.
//
// Measured causes (diag-glb + diag-canopy, 2026-11-20):
// - The F-22/F-15E canopies use a "Glass" material, metalness 1,
//   roughness 1, opaque — with no scene environment map a metal
//   renders BLACK, so the F-15E's camera (inside its canopy) sees a
//   black shell. fixCanopyMaterials() makes every canopy transparent
//   glass (opacity 0.28, no metal).
// - The old fixed eye (fwd x5, up x2.2) floats 0.66 m ABOVE the
//   F-22's canopy (y[0.88,1.54] z[3.92,6.97]); the F-35A has NO
//   canopy mesh (body top ~1.64 at the seat fairing). COCKPIT_EYE
//   gives each jet its own eye from the measured canopy bboxes.
// - Only the F-18's GLB has any cockpit interior (a pilot helmet).
//   buildCockpit() adds the same procedural kit to every player jet
//   (panel + screens, HUD glass + reticle, frame rails + top bow +
//   coaming, seat + helmet), positioned from the eye.
//
// Per jet, through the real attachModel path (real GLBs, Image stub):
// the model loads (modelStatus ok); canopies are transparent glass
// (or, for the F-35A, absent — stated, not faked); the kit parts are
// present; the eye sits inside the canopy bbox (F-35A: just above the
// seat fairing); panel/HUD/frame sit in the forward field of view
// from the eye; the panel top stays below the canopy base (inside
// the fuselage, not floating above the nose).
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
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import {
  buildCockpit,
  cockpitEyeFor,
  fixCanopyMaterials,
} from '../src/aircraft/CockpitBuilder';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

/** Measured body-frame canopy bboxes (diag-glb, 2026-11-20). */
const CANOPY_BBOX: Record<string, { y: [number, number]; z: [number, number] }> = {
  fa18: { y: [2.25, 3.08], z: [1.44, 5.99] },
  f22: { y: [0.88, 1.54], z: [3.92, 6.97] },
  f15e: { y: [1.56, 2.42], z: [2.35, 6.41] },
};

const KIT_PARTS = [
  'cockpit_panel',
  'cockpit_hud_glass',
  'cockpit_hud_reticle',
  'cockpit_frame_L',
  'cockpit_frame_R',
  'cockpit_frame_top',
  'cockpit_coaming',
  'cockpit_seat',
  'cockpit_helmet',
];

async function main(): Promise<void> {
  await RAPIER.init();
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const scene = new THREE.Scene();
  const wind = new THREE.Vector3(0, 0, 0);

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
    // One physics step so the group transform follows the body, then
    // the same setup main.ts runs after attachModel.
    world.timestep = 1 / 60;
    world.step(new RAPIER.EventQueue(true));
    ac.step(1 / 60, wind, null);
    scene.updateMatrixWorld(true);
    const fixed = fixCanopyMaterials(ac.group);
    buildCockpit(ac.group, spec.modelKey);
    scene.updateMatrixWorld(true);

    // --- Canopy glass.
    const canopies: THREE.Mesh[] = [];
    ac.group.traverse((o) => {
      if ((o as THREE.Mesh).isMesh && /canop/i.test(o.name)) {
        canopies.push(o as THREE.Mesh);
      }
    });
    if (key === 'f35a') {
      check(
        'f35a has NO canopy mesh in its GLB (stated — frame kit stands in for it)',
        canopies.length === 0,
        `canopy meshes=${canopies.length}`,
      );
    } else {
      const glass = canopies.every((m) => {
        const mat = m.material as THREE.MeshStandardMaterial;
        return mat.transparent && Math.abs(mat.opacity - 0.28) < 0.01 && mat.metalness === 0;
      });
      check(
        `${key} canopy is transparent glass (no more black shell)`,
        canopies.length >= 1 && glass,
        `canopies=${canopies.length} fixed=${fixed}`,
      );
    }

    // --- Kit parts present.
    const missing = KIT_PARTS.filter((n) => !ac.group.getObjectByName(n));
    check(
      `${key} procedural cockpit kit present (panel/HUD/frame/seat/helmet)`,
      missing.length === 0,
      missing.length ? `missing: ${missing.join(', ')}` : `${KIT_PARTS.length} parts`,
    );

    // --- Eye placement (world = body at heading 0, pos (0,600,0)).
    const eye = cockpitEyeFor(spec.modelKey);
    const eyeWorld = new THREE.Vector3(ac.pos.x, ac.pos.y + eye.y, ac.pos.z + eye.z);
    if (key === 'f35a') {
      // Seat fairing = the smaller named GLB mesh (70 tris); the eye
      // must sit just above its top, and the panel top below it.
      // (Named only — the procedural kit and gear are unnamed.)
      let fairingTop = -Infinity;
      ac.group.traverse((o) => {
        if (!(o as THREE.Mesh).isMesh) return;
        const m = o as THREE.Mesh;
        if (!m.name || /cockpit/i.test(m.name)) return;
        const tris = (m.geometry.attributes.position as THREE.BufferAttribute).count / 3;
        if (tris > 200) return;
        const bb = new THREE.Box3().setFromObject(m);
        fairingTop = Math.max(fairingTop, bb.max.y - ac.pos.y);
      });
      check(
        'f35a eye sits just above the seat-fairing top (not inside the fuselage)',
        fairingTop > -Infinity && eye.y > fairingTop - 0.05 && eye.y < fairingTop + 0.5,
        `eye.y=${eye.y} fairingTop=${fairingTop.toFixed(2)}`,
      );
      check(
        'f35a panel top stays below the fairing top (inside the nose)',
        eye.y - 0.55 < fairingTop,
        `panelTop=${(eye.y - 0.55).toFixed(2)} fairingTop=${fairingTop.toFixed(2)}`,
      );
    } else {
      const bb = CANOPY_BBOX[key]!;
      check(
        `${key} eye sits inside the canopy bbox`,
        eye.y > bb.y[0] && eye.y < bb.y[1] && eye.z > bb.z[0] && eye.z < bb.z[1],
        `eye=(${eye.y}, ${eye.z}) canopy y[${bb.y[0]},${bb.y[1]}] z[${bb.z[0]},${bb.z[1]}]`,
      );
      // Eye inside the LIVE canopy mesh bbox too (not just the
      // measured constants).
      const mesh = canopies[0]!;
      const live = new THREE.Box3().setFromObject(mesh);
      const inside =
        eyeWorld.y > live.min.y - 0.1 && eyeWorld.y < live.max.y + 0.1 &&
        eyeWorld.z > live.min.z - 0.1 && eyeWorld.z < live.max.z + 0.1;
      check(
        `${key} eye sits inside the loaded canopy mesh`,
        inside,
        `eye=(${eyeWorld.y.toFixed(2)}, ${eyeWorld.z.toFixed(2)}) live y[${live.min.y.toFixed(2)},${live.max.y.toFixed(2)}] z[${live.min.z.toFixed(2)},${live.max.z.toFixed(2)}]`,
      );
      // Panel top below the canopy base (inside the fuselage).
      const panel = ac.group.getObjectByName('cockpit_panel')!;
      const pbb = new THREE.Box3().setFromObject(panel);
      check(
        `${key} panel top stays below the canopy base`,
        pbb.max.y < live.min.y + 0.15,
        `panelTop=${(pbb.max.y - ac.pos.y).toFixed(2)} canopyBase=${(live.min.y - ac.pos.y).toFixed(2)}`,
      );
    }

    // --- Forward field of view from the eye. The HUD sits at eye
    // level (near 0°); the frame's top bow is above the eye (~28°);
    // the panel is a look-down instrument (~41° below the forward
    // axis — where a fighter's panel belongs).
    const fwd = new THREE.Vector3(0, 0, 1);
    const tmp = new THREE.Vector3();
    const fovLimits: Record<string, number> = {
      cockpit_panel: 50,
      cockpit_hud_glass: 15,
      cockpit_frame_top: 35,
    };
    let fovOk = true;
    let fovDetail = '';
    for (const [n, limit] of Object.entries(fovLimits)) {
      const part = ac.group.getObjectByName(n);
      if (!part) { fovOk = false; fovDetail = `${n} missing`; break; }
      part.getWorldPosition(tmp);
      const d = tmp.clone().sub(eyeWorld);
      const ang = (Math.acos(Math.max(-1, Math.min(1, d.clone().normalize().dot(fwd)))) * 180) / Math.PI;
      if (d.dot(fwd) <= 0 || ang > limit) {
        fovOk = false;
        fovDetail = `${n} ang=${ang.toFixed(1)}° (limit ${limit}°)`;
        break;
      }
      fovDetail += `${n} ${ang.toFixed(1)}°; `;
    }
    check(
      `${key} panel/HUD/frame in the forward field of view from the eye`,
      fovOk,
      fovDetail,
    );
    ac.dispose();
  }

  console.log(failures === 0 ? '\nALL COCKPIT CHECKS PASSED' : `\n${failures} COCKPIT CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('PROBE ERROR', e);
  process.exit(2);
});
