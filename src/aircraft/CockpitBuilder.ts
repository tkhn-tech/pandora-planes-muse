// CockpitBuilder.ts — pass 11, item 2 (cockpit parity).
//
// The F/A-18F's cockpit view is more detailed than the other three
// jets, and the F-15E's is mostly a black screen. Two measured causes:
//
// 1. CANOPY GLASS: the F-22/F-15E canopies use a "Glass" material
//    with metalness 1, roughness 1, opaque — with no scene environment
//    map a metal renders BLACK, so the F-15E's camera (inside its
//    canopy) sees a black shell, and the F-22's reads as a black hump.
//    The F-18's canopy is opaque white, FrontSide — invisible from
//    inside (backface-culled) but a white blob from outside.
//    fixCanopyMaterials() makes every canopy transparent glass
//    (opacity 0.28, no metal), so the cockpit camera sees through it
//    and it reads as glass from outside too.
//
// 2. CAMERA EYE: updateCamera used ONE fixed eye for all jets
//    (fwd x5, up x2.2). Measured against the GLB canopy bboxes
//    (diag-glb, 2026-11-20): the F-18's eye already sits inside its
//    canopy (y[0.85,1.68] z[1.44,5.99] after the 2026-11-20 belly
//    shift for the included gear; was y[2.25,3.08]); the F-22's eye
//    floats 0.66 m ABOVE its canopy (y[0.88,1.54] z[3.92,6.97]); the
//    F-35A has NO canopy mesh at all (body top ~1.64 at the seat
//    fairing). COCKPIT_EYE gives each jet its own eye from those
//    bboxes.
//
// 3. NO INTERIOR: only the F-18's GLB has any cockpit interior (a
//    pilot helmet, 0.02 m behind the eye). The others are bare shells.
//    buildCockpit() adds the same procedural kit to every player jet
//    — instrument panel (+ screens), HUD combiner + reticle, coaming,
//    seat + helmet — positioned from the eye, so all four read to the
//    F-18's standard. The F-35A's panel is a single wide display (its
//    real layout); the others get two screens + gauge dots. The kit
//    sits inside the fuselage (panel top = eye.y - 0.55, below the
//    canopy base on every jet), so it is occluded from outside by the
//    airframe and only reads from the cockpit (and faintly through the
//    glass from outside). Do NOT reduce the F-18 — it gets the same
//    kit (its GLB helmet is hidden by attachModel's helmet fix; the
//    procedural helmet replaces it, behind the eye where it belongs).
//    The canopy frame rails + top bow were REMOVED 2026-11-20: a
//    straight rail cannot hug the curved canopy and read as a black
//    box on the spine (Nicholas's red circle) — the GLB canopy
//    carries its own frame.

import * as THREE from 'three';

/** Cockpit camera eye in body coords (y up from the CG, z forward),
 *  plus the HUD combiner's z. x is 0 (centreline) on every jet. */
export interface CockpitEye {
  y: number;
  z: number;
  hudZ: number;
}

export const COCKPIT_EYE: Record<string, CockpitEye> = {
  // F/A-18F: canopy y[0.85,1.68] z[1.44,5.99] — eye 0.1 m above the
  // canopy base, inside the bubble. The canopy moved down 1.40 m
  // with the model when the included-gear belly alignment landed
  // (2026-11-20: belly +0.56 -> -0.84 m), so the eye moves with it
  // (was 2.35).
  fa18: { y: 0.95, z: 5.0, hudZ: 5.7 },
  // F-22: canopy y[0.88,1.54] z[3.92,6.97] — eye down inside it.
  f22: { y: 1.3, z: 5.5, hudZ: 6.7 },
  // F-15E: canopy y[1.56,2.42] z[2.35,6.41] — eye inside it.
  f15e: { y: 2.0, z: 4.5, hudZ: 6.1 },
  // F-35A: no canopy mesh; body top ~1.64 at the seat fairing (mesh 2
  // y[-0.29,1.64] z[3.18,6.66]). The eye sits just ABOVE the fairing
  // (1.75) — any lower and the camera is inside the fuselage hump.
  f35a: { y: 1.75, z: 5.0, hudZ: 6.2 },
};

export function cockpitEyeFor(modelKey: string): CockpitEye {
  return COCKPIT_EYE[modelKey] ?? { y: 2.2, z: 5.0, hudZ: 5.7 };
}

/**
 * Cockpit camera orientation for an aircraft quaternion. A three.js
 * camera looks down its local -Z, so copying the aircraft quaternion
 * alone points it AFT (toward the tail) — the v4.6 live cockpit view
 * showed the seat / headrest filling the frame on all four jets,
 * with no panel or HUD, because the camera looked backward at the
 * seat behind the eye instead of forward at the panel. Flip it 180°
 * about Y so the camera looks along the aircraft's forward (+Z body).
 */
export function cockpitCameraQuaternion(
  aircraftQuat: THREE.Quaternion,
): THREE.Quaternion {
  return aircraftQuat
    .clone()
    .multiply(
      new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 1, 0),
        Math.PI,
      ),
    );
}

/** Panel width per jet (F-35A's wide single display; F-15E's tandem
 *  front office is the widest of the conventional panels). */
const PANEL_W: Record<string, number> = {
  fa18: 0.95,
  f22: 0.9,
  f15e: 1.0,
  f35a: 1.1,
};

const MAT = {
  panel: new THREE.MeshStandardMaterial({
    color: 0x0a0f14,
    roughness: 0.6,
    metalness: 0.2,
  }),
  screen: new THREE.MeshBasicMaterial({ color: 0x14324a }),
  screenWide: new THREE.MeshBasicMaterial({ color: 0x1a4a3a }),
  gaugeAmber: new THREE.MeshBasicMaterial({ color: 0xffd166 }),
  gaugeGreen: new THREE.MeshBasicMaterial({ color: 0x7ee787 }),
  hudGlass: new THREE.MeshBasicMaterial({
    color: 0x9fd8ff,
    transparent: true,
    opacity: 0.18,
    side: THREE.DoubleSide,
    depthWrite: false,
  }),
  hudLine: new THREE.MeshBasicMaterial({ color: 0x7ee787 }),
  frame: new THREE.MeshStandardMaterial({
    color: 0x23262b,
    roughness: 0.9,
    metalness: 0.1,
  }),
  seat: new THREE.MeshStandardMaterial({
    color: 0x1c1f24,
    roughness: 0.95,
    metalness: 0,
  }),
  helmet: new THREE.MeshStandardMaterial({
    color: 0x3a3f45,
    roughness: 0.5,
    metalness: 0.3,
  }),
};

function box(
  name: string,
  w: number,
  h: number,
  d: number,
  mat: THREE.Material,
  x: number,
  y: number,
  z: number,
  rx = 0,
): THREE.Mesh {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.name = name;
  m.position.set(x, y, z);
  if (rx) m.rotation.x = rx;
  return m;
}

/**
 * Make every canopy mesh in the group transparent glass. Returns the
 * number of meshes fixed. Call after attachModel (player, enemies and
 * traffic — the black "Glass" canopies read wrong from outside too).
 */
export function fixCanopyMaterials(group: THREE.Group): number {
  let fixed = 0;
  group.traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (!mesh.isMesh || !/canop/i.test(mesh.name)) return;
    const mat = mesh.material as THREE.MeshStandardMaterial | null;
    if (!mat || mat.transparent) return;
    mat.transparent = true;
    mat.opacity = 0.28;
    mat.metalness = 0;
    mat.roughness = 0.08;
    mat.depthWrite = false;
    mat.needsUpdate = true;
    fixed++;
  });
  return fixed;
}

/**
 * Build the procedural cockpit kit for one player jet, in body coords
 * relative to its eye. Returns the group (named `cockpit_procedural`,
 * every part named `cockpit_*` for probes). Idempotent: a second call
 * replaces the first kit.
 */
export function buildCockpit(
  group: THREE.Group,
  modelKey: string,
): THREE.Group {
  const old = group.getObjectByName('cockpit_procedural');
  if (old) group.remove(old);
  const eye = cockpitEyeFor(modelKey);
  const kit = new THREE.Group();
  kit.name = 'cockpit_procedural';
  const panelW = PANEL_W[modelKey] ?? 0.95;
  const panelZ = eye.z + 0.9;
  const panelY = eye.y - 0.78;

  // Instrument panel (top at eye.y - 0.55, below the canopy base).
  kit.add(
    box('cockpit_panel', panelW, 0.45, 0.08, MAT.panel, 0, panelY, panelZ, 0.25),
  );
  // Screens on the panel face: F-35A one wide display, the others two
  // screens + gauge dots.
  const faceZ = panelZ + 0.055;
  const scrY = panelY + 0.05;
  if (modelKey === 'f35a') {
    kit.add(
      box('cockpit_screen', panelW * 0.82, 0.26, 0.02, MAT.screenWide, 0, scrY, faceZ, 0.25),
    );
    kit.add(
      box('cockpit_gauge', 0.3, 0.03, 0.02, MAT.gaugeGreen, 0, scrY - 0.12, faceZ, 0.25),
    );
  } else {
    const sw = panelW * 0.36;
    kit.add(
      box('cockpit_screen_L', sw, 0.26, 0.02, MAT.screen, -panelW * 0.24, scrY, faceZ, 0.25),
    );
    kit.add(
      box('cockpit_screen_R', sw, 0.26, 0.02, MAT.screen, panelW * 0.24, scrY, faceZ, 0.25),
    );
    for (const [gx, gm] of [
      [-0.32, MAT.gaugeAmber],
      [0.32, MAT.gaugeGreen],
    ] as const) {
      const dot = new THREE.Mesh(
        new THREE.CircleGeometry(0.035, 12),
        gm,
      );
      dot.name = 'cockpit_gauge';
      dot.position.set(gx, scrY - 0.13, faceZ + 0.005);
      dot.rotation.x = 0.25;
      kit.add(dot);
    }
  }

  // HUD combiner glass + reticle (ring + crosshair), just ahead of the
  // eye at eye level.
  const hud = new THREE.Mesh(new THREE.PlaneGeometry(0.55, 0.38), MAT.hudGlass);
  hud.name = 'cockpit_hud_glass';
  hud.position.set(0, eye.y + 0.02, eye.hudZ);
  hud.rotation.x = 0.1;
  kit.add(hud);
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.11, 0.006, 8, 32),
    MAT.hudLine,
  );
  ring.name = 'cockpit_hud_reticle';
  ring.position.set(0, eye.y + 0.02, eye.hudZ + 0.01);
  kit.add(ring);
  kit.add(
    box('cockpit_hud_bar_h', 0.3, 0.012, 0.01, MAT.hudLine, 0, eye.y + 0.02, eye.hudZ + 0.01),
  );
  kit.add(
    box('cockpit_hud_bar_v', 0.012, 0.22, 0.01, MAT.hudLine, 0, eye.y + 0.02, eye.hudZ + 0.01),
  );

  // Canopy frame: the coaming (the padded rim below the panel) only.
  // The side rails + top bow are GONE (2026-11-20): they were the
  // black box on the fuselage spine (Nicholas's red circle). A
  // straight vertical rail at a fixed x cannot hug a curved canopy
  // bubble — the F-18's canopy narrows from +-0.39 m at its base
  // (y 0.84) to ~+-0.26 m at mid-height (y 1.10), so even rails set
  // to the canopy's max half-width stuck 0.15+ m outside the glass
  // at mid-height and read as a dark box from the side (raycast-
  // confirmed: the port-side ray at spine height hit
  // cockpit_frame_L before the canopy). The GLB canopy carries its
  // own frame, so no procedural frame is needed — the kit keeps the
  // interior (panel, HUD, seat, helmet) and the coaming, all inside
  // the fuselage / canopy.
  kit.add(
    box('cockpit_coaming', 1.0, 0.08, 0.45, MAT.frame, 0, eye.y - 1.04, eye.z + 0.85),
  );

  // Seat + helmet behind the eye (visible from outside through the
  // glass, and in the rear view — never in the forward sightline).
  // Kept LOW and compact (2026-11-20): the old 0.85 m seat back
  // towered to eye.y + 0.13 with the headrest to eye.y + 0.33, so on
  // the canopy-less F-35A the whole assembly stuck 0.24–0.44 m above
  // the fuselage top as a black box on the spine, and — with the
  // pre-fix camera looking AFT — it filled the cockpit view. The seat
  // back now tops out BELOW the eye (eye.y - 0.12), the headrest and
  // helmet sit at eye level (the pilot's head), and all three sit
  // further back, so the eye is in front of the panel + HUD with
  // nothing tall behind it crowding the view.
  kit.add(
    box('cockpit_seat', 0.62, 0.6, 0.14, MAT.seat, 0, eye.y - 0.42, eye.z - 0.78, -0.2),
  );
  kit.add(
    box('cockpit_headrest', 0.3, 0.16, 0.1, MAT.seat, 0, eye.y + 0.02, eye.z - 0.84, -0.2),
  );
  const helmet = new THREE.Mesh(new THREE.SphereGeometry(0.13, 16, 12), MAT.helmet);
  helmet.name = 'cockpit_helmet';
  helmet.position.set(0, eye.y + 0.02, eye.z - 0.64);
  kit.add(helmet);

  group.add(kit);
  return kit;
}
