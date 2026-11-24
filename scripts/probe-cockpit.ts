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
//   (panel + screens, HUD glass, coaming, seat + helmet),
//   positioned from the eye. The HUD reticle (ring + crosshair) was
//   REMOVED 2026-11-20 at Nicholas's direction ("Remove this from
//   ALL JETS") — it read as a green crosshair circle on the canopy
//   in the external/chase view.
// - The canopy frame rails + top bow were REMOVED 2026-11-20: a
//   straight rail cannot hug the curved canopy and read as a black
//   box on the fuselage spine (Nicholas's red circle) — the GLB
//   canopy carries its own frame. This probe raycasts from both
//   sides at spine height to prove no cockpit part protrudes outside
//   the airframe / canopy any more.
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
import * as fs from 'node:fs';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import {
  buildCockpit,
  cockpitCameraQuaternion,
  cockpitEyeFor,
  fixCanopyMaterials,
} from '../src/aircraft/CockpitBuilder';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

/** True when the object and every ancestor are visible — what the
 *  renderer draws. Pass 13: THREE's Raycaster ignores `visible`,
 *  so a raycast without this filter hits stowed (hidden) gear legs
 *  and the hidden GLB helmet as if they were on screen. */
function isVisible(o: THREE.Object3D): boolean {
  let p: THREE.Object3D | null = o;
  while (p) {
    if (!p.visible) return false;
    p = p.parent;
  }
  return true;
}

/** Measured body-frame canopy bboxes (diag-glb, 2026-11-20; fa18
 *  shifted down 1.40 m with the 2026-11-20 included-gear belly
 *  alignment, was y[2.25, 3.08]). */
const CANOPY_BBOX: Record<string, { y: [number, number]; z: [number, number] }> = {
  fa18: { y: [0.85, 1.68], z: [1.44, 5.99] },
  f22: { y: [0.88, 1.54], z: [3.92, 6.97] },
  f15e: { y: [1.56, 2.42], z: [2.35, 6.41] },
};

const KIT_PARTS = [
  'cockpit_panel',
  'cockpit_hud_glass',
  'cockpit_coaming',
  'cockpit_seat',
  'cockpit_helmet',
];

// Reticle names that must NOT exist anywhere in the aircraft group
// (2026-11-20, Nicholas: "Remove this from ALL JETS" — the green
// crosshair circle on the canopy in the external/chase view). The
// kit is built into ac.group, which is the rendered model in BOTH
// views — cockpit (camera inside it) and external/chase (camera
// outside it) — so a traverse-absence check covers both.
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
    // Pass 13 (worker C): the live cockpit check was AIRBORNE
    // (776 kt) — gear stowed, legs hidden. Replicate that state
    // before raycasting, or a gear leg standing in the sightline
    // would read as a cockpit occlusion. (The rig's bellyY frame
    // bug used to build ~600 m legs here; with it fixed the legs
    // are jet-scale either way, but the airborne state is the one
    // the live check saw.)
    if (ac.rig) {
      ac.rig.gearDeploy = 0;
      ac.rig.update(
        0,
        { pitch: 0, roll: 0, yaw: 0, throttle: null },
        { aoaDeg: 0, gear: false, hook: false, vectorDeg: 0, rotorRpm: 0 },
      );
    }
    scene.updateMatrixWorld(true);

    // --- Rig belly (pass 13): bellyY must be in MODEL units. It
    // was measured from a world-space bbox while the group sat at
    // the spawn altitude, so it came out ~599 and every procedural
    // leg was built as a ~600 m spike (nose-gear strutTop ~599.5).
    {
      const bellyY = ac.rig?.bellyY;
      check(
        `${key} rig bellyY is in model units (no altitude leak)`,
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

    // --- No HUD reticle anywhere in the aircraft group (2026-11-20,
    // Nicholas: "Remove this from ALL JETS"). The reticle (ring +
    // crosshair bars) read as a green crosshair circle on the canopy
    // in the external/chase view. The kit is the rendered model in
    // both views, so absence here covers cockpit AND external.
    const reticleFound: string[] = [];
    ac.group.traverse((o) => {
      if (RETICLE_NAMES.includes(o.name)) reticleFound.push(o.name);
    });
    check(
      `${key} NO HUD reticle present (cockpit + external view)`,
      reticleFound.length === 0,
      reticleFound.length ? `found: ${reticleFound.join(', ')}` : 'no reticle objects',
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
        'f35a fascia top stays below the fairing top (no slab on the nose spine)',
        (() => {
          const panel = ac.group.getObjectByName('cockpit_panel');
          if (!panel) return false;
          const pbb = new THREE.Box3().setFromObject(panel);
          return pbb.max.y - ac.pos.y < fairingTop;
        })(),
        (() => {
          const panel = ac.group.getObjectByName('cockpit_panel');
          const top = panel
            ? (new THREE.Box3().setFromObject(panel).max.y - ac.pos.y).toFixed(2)
            : 'n/a';
          return `fasciaTop=${top} fairingTop=${fairingTop.toFixed(2)}`;
        })(),
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
      // Panel top under the jet's own skin (pass 13): the old check
      // ("panel top below the canopy BASE") is obsolete — the F-18
      // rail sits inside the canopy bubble, above the base, and the
      // real constraint is that it does not poke THROUGH the jet.
      // Cast DOWN from above the jet at the panel's (x, z): the
      // first hit must be a GLB surface (canopy or airframe spine —
      // on the F-15E the spine rides above the canopy at the panel
      // station), never a cockpit part. (An upward ray cannot be
      // used — the canopy is FrontSide, so a ray from inside hits
      // only back faces and the raycaster culls them.)
      const panel = ac.group.getObjectByName('cockpit_panel')!;
      const pbb = new THREE.Box3().setFromObject(panel);
      const topC = new THREE.Vector3(
        (pbb.min.x + pbb.max.x) / 2,
        pbb.max.y,
        (pbb.min.z + pbb.max.z) / 2,
      );
      const downRay = new THREE.Raycaster(
        new THREE.Vector3(topC.x, ac.pos.y + 4, topC.z),
        new THREE.Vector3(0, -1, 0),
      );
      downRay.far = 8;
      const downMeshes: THREE.Mesh[] = [];
      ac.group.traverse((o) => {
        if ((o as THREE.Mesh).isMesh && isVisible(o)) downMeshes.push(o as THREE.Mesh);
      });
      const downHits = downRay.intersectObjects(downMeshes, false);
      const firstDown = downHits[0]?.object.name ?? 'NONE';
      check(
        `${key} panel top sits under the jet's skin (not poking out of it)`,
        firstDown !== 'NONE' && !/cockpit/i.test(firstDown),
        `panelTop=${(pbb.max.y - ac.pos.y).toFixed(2)} first hit from above: ${firstDown}`,
      );
    }

    // --- Forward field of view from the eye. The HUD sits at eye
    // level (near 0°); the panel is a look-down instrument (~41°
    // below the forward axis — where a fighter's panel belongs).
    const fwd = new THREE.Vector3(0, 0, 1);
    const tmp = new THREE.Vector3();
    const fovLimits: Record<string, number> = {
      cockpit_panel: 50,
      cockpit_hud_glass: 15,
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
      `${key} panel/HUD in the forward field of view from the eye`,
      fovOk,
      fovDetail,
    );

    // --- The cockpit camera looks FORWARD (v4.6 live failure,
    // 2026-11-20): copying the aircraft quaternion alone points a
    // three.js camera AFT (it looks down its local -Z), so the v4.6
    // cockpit view showed the seat / headrest filling the frame on
    // all four jets, with no panel or HUD. cockpitCameraQuaternion
    // flips it 180° about Y. Verify the camera's look direction
    // (-Z under the quaternion) equals the aircraft's forward (+Z
    // body), and that the seat / headrest / helmet sit BEHIND the
    // eye (not in the forward view) while the panel / HUD sit in
    // front of it.
    {
      const camQ = cockpitCameraQuaternion(ac.quat);
      const look = new THREE.Vector3(0, 0, -1).applyQuaternion(camQ);
      const fwdBody = new THREE.Vector3(0, 0, 1).applyQuaternion(ac.quat);
      const align = look.dot(fwdBody);
      // Pass 12: the cockpit camera is pitched down 15° so the panel
      // (with its dials) is visible at the bottom of the frame — the
      // panel sits ~42° below the forward axis and the 62° FOV camera
      // (±31°) cannot see it level. look·fwd = cos(15°) ≈ 0.966.
      check(
        `${key} cockpit camera looks FORWARD and slightly DOWN (not aft at the seat)`,
        align > 0.95 && align < 0.99,
        `look·fwd=${align.toFixed(3)}`,
      );
      let seatOk = true;
      let seatDetail = '';
      for (const n of ['cockpit_seat', 'cockpit_headrest', 'cockpit_helmet']) {
        const part = ac.group.getObjectByName(n);
        if (!part) { seatOk = false; seatDetail = `${n} missing`; break; }
        part.getWorldPosition(tmp);
        const d = tmp.clone().sub(eyeWorld);
        // Behind the eye = negative forward component (body +Z).
        if (d.dot(fwd) >= 0) {
          seatOk = false;
          seatDetail = `${n} is in front of the eye (dz=${d.dot(fwd).toFixed(2)})`;
          break;
        }
        seatDetail += `${n} dz=${d.dot(fwd).toFixed(2)}; `;
      }
      check(
        `${key} seat/headrest/helmet sit BEHIND the eye (not in the forward view)`,
        seatOk,
        seatDetail,
      );
      // The seat back tops out below the eye (it no longer towers
      // over the fuselage as a black box).
      const seat = ac.group.getObjectByName('cockpit_seat')!;
      const sbb = new THREE.Box3().setFromObject(seat);
      check(
        `${key} seat back tops out below the eye line`,
        sbb.max.y < eyeWorld.y + 0.05,
        `seatTop=${(sbb.max.y - ac.pos.y).toFixed(2)} eye.y=${eye.y.toFixed(2)}`,
      );
      // Pass 12: the dials must be inside the cockpit camera's frame
      // (62° FOV, ±31° from the look direction). The camera is pitched
      // down 15° so the panel (~42° below forward) lands ~27° below
      // the look direction — visible at the bottom of the frame.
      {
        let frameOk = true;
        let frameDetail = '';
        for (const n of ['cockpit_dial_0', 'cockpit_dial_1', 'cockpit_dial_2']) {
          const part = ac.group.getObjectByName(n);
          if (!part) { frameOk = false; frameDetail = `${n} missing`; break; }
          part.getWorldPosition(tmp);
          const d = tmp.clone().sub(eyeWorld);
          const ang = (Math.acos(Math.max(-1, Math.min(1, d.clone().normalize().dot(look.clone().normalize())))) * 180) / Math.PI;
          if (d.dot(look) <= 0 || ang > 31) {
            frameOk = false;
            frameDetail = `${n} ang=${ang.toFixed(1)}° (limit 31°)`;
            break;
          }
          frameDetail += `${n} ${ang.toFixed(1)}°; `;
        }
        check(
          `${key} dials inside the cockpit camera frame`,
          frameOk,
          frameDetail,
        );
      }
    }

    // --- Pass 13 (worker C): LIVE-path visibility. The pass-12 probe
    // only checked angles from the eye — it never asked whether
    // anything is BETWEEN the eye and the dials, whether the dials
    // land inside the REAL camera frustum (62° FOV, near 0.5, live
    // aspect — a portrait phone's horizontal half-FOV is only ~15°),
    // or whether the dial faces even point at the pilot. The v4.7
    // live check saw blank panels on all four jets, so these are the
    // checks that must fail before the fix and pass after it.
    {
      // Live camera, exactly as main.ts builds it: PerspectiveCamera
      // 62°, near 0.5, far 20000, cockpit position + quaternion.
      const eyeLive = ac.pos
        .clone()
        .addScaledVector(fwd, eye.z)
        .addScaledVector(new THREE.Vector3(0, 1, 0).applyQuaternion(ac.quat), eye.y);
      const look = new THREE.Vector3(0, 0, -1).applyQuaternion(
        cockpitCameraQuaternion(ac.quat),
      );
      void look;
      const allMeshes: THREE.Mesh[] = [];
      ac.group.traverse((o) => {
        if ((o as THREE.Mesh).isMesh && isVisible(o)) allMeshes.push(o as THREE.Mesh);
      });
      const ray = new THREE.Raycaster();
      // Occlusion: cast from the live eye at each instrument. The
      // first OPAQUE hit must be the target itself — a canopy hit
      // does not count (transparent glass, opacity 0.28, see-through)
      // but the panel slab, a screen, or any GLB airframe mesh does.
      const occlTargets: Array<{ name: string; obj: THREE.Object3D }> = [];
      for (const n of ['cockpit_dial_0', 'cockpit_dial_1', 'cockpit_dial_2', 'cockpit_panel', 'cockpit_screen_L', 'cockpit_screen_R', 'cockpit_screen']) {
        const o = ac.group.getObjectByName(n);
        if (o) occlTargets.push({ name: n, obj: o });
      }
      for (const t of occlTargets) {
        const target = new THREE.Vector3();
        t.obj.getWorldPosition(target);
        const dir = target.clone().sub(eyeLive);
        const dist = dir.length();
        ray.set(eyeLive, dir.normalize());
        ray.far = dist + 0.5;
        const hits = ray.intersectObjects(allMeshes, false);
        let firstOpaque: THREE.Intersection | undefined;
        let canopyHits = 0;
        for (const h of hits) {
          if (/canop/i.test(h.object.name)) { canopyHits++; continue; }
          firstOpaque = h;
          break;
        }
        const hitIsTarget =
          !!firstOpaque &&
          (firstOpaque.object === t.obj ||
            t.obj.getObjectByName(firstOpaque.object.name) !== undefined ||
            firstOpaque.object.name.startsWith(t.name) ||
            (t.obj as THREE.Group).children.includes(firstOpaque.object as never) ||
            firstOpaque.object.parent === t.obj);
        // Surface detail on the target itself (a glyph line drawn
        // 0.002 m proud of a screen face) reads as a "blocker" at
        // 0.01 m before the target — it is part of the target, not
        // an occluder. Allow a first hit that belongs to the panel
        // mount and sits within 0.05 m of the target.
        const inPanelMount = (o: THREE.Object3D): boolean => {
          let p: THREE.Object3D | null = o;
          while (p) {
            if (p.name === 'cockpit_panel_mount') return true;
            p = p.parent;
          }
          return false;
        };
        // For the panel itself the first opaque hit IS the panel.
        const visible =
          !firstOpaque ||
          firstOpaque.object === t.obj ||
          hitIsTarget ||
          (t.name === 'cockpit_panel' && /cockpit_panel/.test(firstOpaque.object.name)) ||
          (inPanelMount(firstOpaque.object) && firstOpaque.distance >= dist - 0.05);
        check(
          `${key} LIVE ray eye→${t.name} is not blocked (first opaque hit is the target itself)`,
          visible,
          firstOpaque
            ? `first opaque: ${firstOpaque.object.name} at ${(firstOpaque.distance).toFixed(2)}m, target at ${dist.toFixed(2)}m${canopyHits ? ` (through ${canopyHits} canopy)` : ''}`
            : `clear to ${dist.toFixed(2)}m`,
        );
      }
      // Dial face orientation: the CircleGeometry face (+Z local,
      // tilted by the dial group's rx) must point AT the pilot —
      // its world normal dotted with (eye - facePos) must be > 0,
      // else the single-sided face is backface-culled from the
      // cockpit camera. (A DoubleSide material also satisfies this
      // check's intent; the check reads the material side too.)
      for (const n of ['cockpit_dial_0', 'cockpit_dial_1', 'cockpit_dial_2']) {
        const d = ac.group.getObjectByName(n);
        if (!d) { check(`${key} ${n} face points at the pilot`, false, 'dial missing'); continue; }
        const face = d.getObjectByName('cockpit_dial_face') as THREE.Mesh | undefined;
        if (!face) { check(`${key} ${n} face points at the pilot`, false, 'face mesh missing'); continue; }
        const facePos = new THREE.Vector3();
        face.getWorldPosition(facePos);
        const normal = new THREE.Vector3(0, 0, 1).applyQuaternion(face.getWorldQuaternion(new THREE.Quaternion()));
        const toEye = eyeLive.clone().sub(facePos).normalize();
        const mat = face.material as THREE.MeshBasicMaterial;
        const facing = normal.dot(toEye);
        check(
          `${key} ${n} face is visible from the eye (normal toward pilot, or DoubleSide)`,
          facing > 0 || mat.side === THREE.DoubleSide,
          `normal·toEye=${facing.toFixed(2)} side=${mat.side === THREE.DoubleSide ? 'DoubleSide' : 'FrontSide'}`,
        );
      }
      // NDC projection through the REAL live camera (62° FOV, near
      // 0.5) at portrait-phone aspect (390x844) and landscape 16:9.
      // A dial counts only if it lands inside the frustum on BOTH.
      for (const aspect of [390 / 844, 16 / 9]) {
        const cam = new THREE.PerspectiveCamera(62, aspect, 0.5, 20000);
        cam.position.copy(eyeLive);
        cam.quaternion.copy(cockpitCameraQuaternion(ac.quat));
        cam.updateMatrixWorld(true);
        let ndcOk = true;
        let ndcDetail = '';
        for (const n of ['cockpit_dial_0', 'cockpit_dial_1', 'cockpit_dial_2']) {
          const part = ac.group.getObjectByName(n);
          if (!part) { ndcOk = false; ndcDetail = `${n} missing`; break; }
          const p = new THREE.Vector3();
          part.getWorldPosition(p);
          p.project(cam);
          const inside = Math.abs(p.x) <= 1 && Math.abs(p.y) <= 1 && p.z > -1 && p.z < 1;
          ndcDetail += `${n} (${p.x.toFixed(2)},${p.y.toFixed(2)}); `;
          if (!inside) ndcOk = false;
        }
        check(
          `${key} dials inside the LIVE camera frustum (aspect ${aspect.toFixed(2)})`,
          ndcOk,
          ndcDetail,
        );
      }
      // NOTE (pass 13): a "deck below the dials" check used to live
      // here — a downward ray at the panel station. It is REMOVED:
      // on the F-15E the first GLB surface above the panel station
      // is the airframe spine (2.13 m), which rides ABOVE the
      // cockpit opening; the panel sits in the tub below the spine
      // and is visible from the eye through the opening. The
      // eye→dial occlusion raycast above already tests the actual
      // sightline, which is the correct statement of "not buried".
    }

    // --- No black box on the spine: the canopy frame rails were the
    // red-circled artifact (they sat outside the curved canopy and
    // read as a dark box from the side). They are removed, so no
    // cockpit part may be the first thing a ray from outside the
    // spine hits — the canopy or airframe must occlude the kit.
    {
      const allMeshes: THREE.Mesh[] = [];
      ac.group.traverse((o) => {
        if ((o as THREE.Mesh).isMesh && isVisible(o)) allMeshes.push(o as THREE.Mesh);
      });
      const frameParts = allMeshes.filter((m) => /cockpit_frame/i.test(m.name));
      check(
        `${key} no canopy-frame parts remain (the black-box artifact is gone)`,
        frameParts.length === 0,
        frameParts.length ? `still present: ${frameParts.map((m) => m.name).join(', ')}` : 'none',
      );
      const ray = new THREE.Raycaster();
      const eyeY = ac.pos.y + eye.y;
      const eyeZ = ac.pos.z + eye.z;
      // Two heights per side: eye level (where the frame rails
      // were) and 0.5 m lower (through the fuselage, so the F-35A —
      // whose eye sits above its fuselage — still gets a hit).
      const sideShots: Array<[string, THREE.Vector3, THREE.Vector3]> = [
        ['port', new THREE.Vector3(ac.pos.x - 10, eyeY, eyeZ), new THREE.Vector3(ac.pos.x, eyeY, eyeZ)],
        ['port-low', new THREE.Vector3(ac.pos.x - 10, eyeY - 0.5, eyeZ), new THREE.Vector3(ac.pos.x, eyeY - 0.5, eyeZ)],
        ['starboard', new THREE.Vector3(ac.pos.x + 10, eyeY, eyeZ), new THREE.Vector3(ac.pos.x, eyeY, eyeZ)],
        ['starboard-low', new THREE.Vector3(ac.pos.x + 10, eyeY - 0.5, eyeZ), new THREE.Vector3(ac.pos.x, eyeY - 0.5, eyeZ)],
      ];
      for (const [side, from, to] of sideShots) {
        const dir = to.clone().sub(from).normalize();
        ray.set(from, dir);
        ray.far = 30;
        const hits = ray.intersectObjects(allMeshes, false);
        const first = hits[0]?.object.name ?? 'NO HIT';
        // A miss is fine (nothing protrudes into the ray); a hit
        // must be airframe / canopy, never a cockpit part.
        check(
          `${key} ${side} ray at spine height hits airframe/canopy, not a cockpit part`,
          hits.length === 0 || !/cockpit/i.test(first),
          `first hit: ${first}`,
        );
      }
    }
    ac.dispose();
  }

  // --- Pass 13 (worker C): the amber ⊕ crosshair was the CCIP DOM
  // pipper (#ccip: gold ring + 44 px cross bars, #ffd166), shown
  // by main.ts whenever a bomb-carrying jet (fa18/f15e) was airborne
  // — which is why it appeared on F-18/F-15E and never on
  // F-22/F-35A (bombs: false). It must be gone from the markup, the
  // styles and the HUD wiring. The computation stays (it feeds the
  // PULL UP warning), but nothing may display it.
  {
    const read = (p: string): string => {
      try {
        return fs.readFileSync(p, 'utf8');
      } catch {
        return '';
      }
    };
    const html = read('index.html');
    const mainSrc = read('src/main.ts');
    const hudSrc = read('src/ui/FlightHUD.ts');
    check(
      'CCIP pipper div is gone from index.html',
      html.length > 0 && !html.includes('id="ccip"'),
      html.length === 0 ? 'index.html unreadable (run from the project root)' : 'no id="ccip"',
    );
    check(
      'CCIP pipper styles are gone from index.html',
      html.length > 0 && !html.includes('#ccip'),
      html.length === 0 ? 'index.html unreadable' : 'no #ccip selector',
    );
    check(
      'main.ts never shows the CCIP pipper',
      mainSrc.length > 0 && !mainSrc.includes('showCcip'),
      mainSrc.length === 0 ? 'src/main.ts unreadable' : 'no showCcip call',
    );
    check(
      'FlightHUD has no CCIP display method',
      hudSrc.length > 0 && !hudSrc.includes('showCcip') && !hudSrc.includes('hideCcip'),
      hudSrc.length === 0 ? 'src/ui/FlightHUD.ts unreadable' : 'no showCcip/hideCcip',
    );
    check(
      'CCIP computation still feeds the PULL UP warning',
      mainSrc.includes('computeCcip') && mainSrc.includes('belowMinReleaseAlt'),
      'computeCcip + belowMinReleaseAlt present in main.ts',
    );
  }

  console.log(failures === 0 ? '\nALL COCKPIT CHECKS PASSED' : `\n${failures} COCKPIT CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error('PROBE ERROR', e);
  process.exit(2);
});
