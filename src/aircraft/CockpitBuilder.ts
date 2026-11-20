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
//    — instrument panel (+ screens), HUD combiner glass, coaming,
//    seat + helmet — positioned from the eye, so all four read to the
//    F-18's standard. The F-35A's panel is a single wide display (its
//    real layout); the others get two screens. Pass 12: every screen
//    carries lit symbology and a row of three round dials (rims,
//    ticks, needles) sits below the screens — the panels were blank
//    grey/black with no dials in the v4.6.1 live check. The kit
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
 * Pass 12: then pitch the view down so the instrument panel (with
 * its dials) is visible at the bottom of the frame — the panel sits
 * ~42° below the forward axis and the 62° FOV cockpit camera (±31°)
 * cannot see it level. 15° down puts the panel ~27° below the look
 * direction — inside the frame — while the horizon stays in the
 * upper half.
 */
export const COCKPIT_PITCH_DOWN_DEG = 15;

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
    )
    .multiply(
      new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(1, 0, 0),
        -THREE.MathUtils.degToRad(COCKPIT_PITCH_DOWN_DEG),
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
  // Dial + screen-symbology materials (pass 12, worker D): unlit
  // MeshBasicMaterial, so the instruments read in the dark cockpit
  // (the panel is inside the fuselage, away from the sun — standard
  // materials go black there, which is why the live panels read as
  // blank grey/black with no dials).
  dialFace: new THREE.MeshBasicMaterial({ color: 0x0d1a24 }),
  dialRim: new THREE.MeshBasicMaterial({ color: 0xd8e2ea }),
  dialTick: new THREE.MeshBasicMaterial({ color: 0xcfe3ef }),
  dialNeedle: new THREE.MeshBasicMaterial({ color: 0xff5a4a }),
  dialNeedleWhite: new THREE.MeshBasicMaterial({ color: 0xffffff }),
  screenLine: new THREE.MeshBasicMaterial({ color: 0x7ee787 }),
  screenLineCyan: new THREE.MeshBasicMaterial({ color: 0x6ab8ff }),
  screenLineAmber: new THREE.MeshBasicMaterial({ color: 0xffd166 }),
  hudGlass: new THREE.MeshBasicMaterial({
    color: 0x9fd8ff,
    transparent: true,
    opacity: 0.18,
    side: THREE.DoubleSide,
    depthWrite: false,
  }),
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
 * One round standby dial (pass 12, worker D): dark face, white rim,
 * 8 tick marks, a needle on a pivot (rotated to `needleDeg`, 0 = 12
 * o'clock) and a centre hub. The whole dial tilts with the panel
 * (rx 0.25). Parts are named `cockpit_dial_*` for probes; the needle
 * pivot is `cockpit_dial_needle_pivot`, the needle itself
 * `cockpit_dial_needle`.
 */
function dial(
  name: string,
  r: number,
  needleDeg: number,
  needleMat: THREE.Material,
  x: number,
  y: number,
  z: number,
): THREE.Group {
  const g = new THREE.Group();
  g.name = name;
  g.position.set(x, y, z);
  g.rotation.x = 0.25;
  const face = new THREE.Mesh(new THREE.CircleGeometry(r, 24), MAT.dialFace);
  face.name = 'cockpit_dial_face';
  g.add(face);
  const rim = new THREE.Mesh(
    new THREE.TorusGeometry(r, r * 0.1, 8, 32),
    MAT.dialRim,
  );
  rim.name = 'cockpit_dial_rim';
  g.add(rim);
  for (let i = 0; i < 8; i++) {
    const a = (i / 8) * Math.PI * 2;
    const tick = new THREE.Mesh(
      new THREE.BoxGeometry(0.006, r * 0.22, 0.004),
      MAT.dialTick,
    );
    tick.name = 'cockpit_dial_tick';
    tick.position.set(Math.sin(a) * r * 0.86, Math.cos(a) * r * 0.86, 0.002);
    tick.rotation.z = -a;
    g.add(tick);
  }
  const pivot = new THREE.Group();
  pivot.name = 'cockpit_dial_needle_pivot';
  pivot.rotation.z = (-needleDeg * Math.PI) / 180;
  const needle = new THREE.Mesh(
    new THREE.BoxGeometry(0.008, r * 0.8, 0.004),
    needleMat,
  );
  needle.name = 'cockpit_dial_needle';
  needle.position.set(0, r * 0.4, 0.004);
  pivot.add(needle);
  g.add(pivot);
  const hub = new THREE.Mesh(
    new THREE.CircleGeometry(r * 0.12, 12),
    MAT.dialNeedleWhite,
  );
  hub.name = 'cockpit_dial_hub';
  hub.position.z = 0.006;
  g.add(hub);
  return g;
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
  // screens. Pass 12 (worker D): the screens were blank dark
  // rectangles and the only "gauges" were two flat dots — the
  // v4.6.1 live check read the panels as "blank grey/black with no
  // dials". Each screen now carries lit symbology (a horizon bar +
  // data blocks, unlit so they read in the dark cockpit), and a row
  // of three round dials with rims, ticks and needles sits below
  // the screens, on the panel face.
  const faceZ = panelZ + 0.055;
  const scrY = panelY + 0.05;
  const glyph = (cx: number, w: number): void => {
    kit.add(
      box('cockpit_screen_line', w * 0.8, 0.012, 0.004, MAT.screenLine, cx, scrY, faceZ + 0.012, 0.25),
    );
    kit.add(
      box('cockpit_screen_line', 0.05, 0.035, 0.004, MAT.screenLineCyan, cx - w * 0.22, scrY + 0.06, faceZ + 0.012, 0.25),
    );
    kit.add(
      box('cockpit_screen_line', 0.05, 0.02, 0.004, MAT.screenLineAmber, cx + w * 0.22, scrY - 0.07, faceZ + 0.012, 0.25),
    );
  };
  if (modelKey === 'f35a') {
    kit.add(
      box('cockpit_screen', panelW * 0.82, 0.26, 0.02, MAT.screenWide, 0, scrY, faceZ, 0.25),
    );
    glyph(0, panelW * 0.82);
  } else {
    const sw = panelW * 0.36;
    kit.add(
      box('cockpit_screen_L', sw, 0.26, 0.02, MAT.screen, -panelW * 0.24, scrY, faceZ, 0.25),
    );
    kit.add(
      box('cockpit_screen_R', sw, 0.26, 0.02, MAT.screen, panelW * 0.24, scrY, faceZ, 0.25),
    );
    glyph(-panelW * 0.24, sw);
    glyph(panelW * 0.24, sw);
  }
  // Dial row (airspeed / attitude / altitude standbys): white rims,
  // ticks and coloured needles, so they read as instruments. The
  // row sits on the panel face below the screens (dial top =
  // scrY - 0.14 + 0.062, still below the panel top at eye.y - 0.55).
  const dialY = scrY - 0.14;
  const dialR = 0.062;
  const dialAngles = [-40, 15, 65];
  const dialMats = [MAT.dialNeedle, MAT.dialNeedleWhite, MAT.dialNeedle];
  [-0.3, 0, 0.3].forEach((dx, i) => {
    kit.add(
      dial(
        `cockpit_dial_${i}`,
        dialR,
        dialAngles[i]!,
        dialMats[i]!,
        dx,
        dialY,
        faceZ + 0.012,
      ),
    );
  });

  // HUD combiner glass only, just ahead of the eye at eye level.
  // The green reticle (ring + crosshair bars) was REMOVED 2026-11-20
  // at Nicholas's direction ("Remove this from ALL JETS", F-22
  // screenshot 20 Nov ~14:28 SGT, purple circle): the reticle read
  // as a green crosshair circle on the canopy in the external/chase
  // view. The glass stays — it is a separate transparent plane, not
  // part of what he circled.
  const hud = new THREE.Mesh(new THREE.PlaneGeometry(0.55, 0.38), MAT.hudGlass);
  hud.name = 'cockpit_hud_glass';
  hud.position.set(0, eye.y + 0.02, eye.hudZ);
  hud.rotation.x = 0.1;
  kit.add(hud);

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
  // helmet sit 0.30 below eye level (tops 1.55 / 1.60, below the
  // 1.64 fuselage top — the canopy-less F-35A has nothing to hide
  // them), and all three sit further back, so the eye is in front
  // of the panel + HUD with nothing tall behind it crowding the view.
  kit.add(
    box('cockpit_seat', 0.62, 0.6, 0.14, MAT.seat, 0, eye.y - 0.42, eye.z - 0.78, -0.2),
  );
  kit.add(
    box('cockpit_headrest', 0.3, 0.16, 0.1, MAT.seat, 0, eye.y - 0.28, eye.z - 0.84, -0.2),
  );
  const helmet = new THREE.Mesh(new THREE.SphereGeometry(0.13, 16, 12), MAT.helmet);
  helmet.name = 'cockpit_helmet';
  helmet.position.set(0, eye.y - 0.28, eye.z - 0.64);
  kit.add(helmet);

  group.add(kit);
  return kit;
}
