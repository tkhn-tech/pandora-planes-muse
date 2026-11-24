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
//    F-18's standard. Pass 12: every screen carries lit symbology
//    and a row of three round dials (rims, ticks, needles) sits below
//    the screens — the panels were blank grey/black with no dials in
//    the v4.6.1 live check. Pass 13 (worker C): the pass-12 dials
//    STILL did not render live (v4.7 live check: blank panel on all
//    four jets). Two measured causes, both in buildCockpit: (a) the
//    screens/dials were mounted at faceZ = panelZ + 0.055, on the
//    NOSE side of the opaque panel slab, so the live eye→dial ray
//    hit the panel at ~1.15 m before the dials at ~1.30 m; (b) the
//    dial faces' +Z normals pointed at the nose, backface-culling
//    them (normal·toEye -0.86..-0.89). The instruments now mount on
//    the panel's PILOT side, in a panel group whose +Z faces the
//    pilot. And the F-18/F-35A panels move onto the visible
//    dashboard slope as a thin fascia: their GLBs have no open tub
//    at the old station — the F-18's airframe deck rises to y 0.85
//    at z 6.0 under a falling canopy, and the F-35A's seat-fairing
//    hump runs 1.52 @ z 5.0 to 0.95 @ z 6.0 — so a slab there is
//    buried under the airframe itself (eye→panel ray hits the GLB
//    at 0.67 m / 1.04 m). The fasciae sit in the measured open
//    slots (fa18 y 0.80 z 5.60; f35a y 1.22 z 5.85), aimed at the
//    eye. Do NOT reduce the F-18 — it gets the same kit (its GLB
//    helmet is hidden by attachModel's helmet fix; the procedural
//    helmet replaces it, behind the eye where it belongs).
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
  // (was 2.35). Pass 13: y 0.95 -> 1.05. The old eye sat BELOW the
  // airframe deck (~1.01 at z 5.0) — inside the fuselage solid, so
  // even the HUD combiner at eye level was blocked (the v4.7 live
  // check's "flat dark-grey slab" is the deck's interior, and the
  // eye→HUD ray hits the airframe). 1.05 is 0.04 above the deck
  // and 0.23 below the canopy top at the eye's z (1.28), in the
  // open lens between deck and glass; the eye→HUD ray is clear
  // from 1.05 (diag-sweep, 2026-11-20).
  fa18: { y: 1.05, z: 5.0, hudZ: 5.7 },
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

/**
 * Per-jet panel placement (pass 13, worker C). Measured from the
 * live GLBs with the probe's raycasts (diag-glb / diag-ray,
 * 2026-11-20) — the pass-11/12 assumption that the space
 * ahead-below the eye is open tub holds only for the F-22/F-15E:
 *
 * - fa18: the "ground" airframe mesh (the whole GLB airframe) rises
 *   as a dashboard hump from (z 5.51, y 0.51) to (z 6.0, y 0.85)
 *   while the canopy top falls to ~0.90 — at the old panel station
 *   (z 5.9) there is no slot for a 0.45 m panel at all, and the
 *   eye→panel ray hits the airframe at 0.67 m. Worse, the old eye
 *   (y 0.95) sat BELOW the deck (~1.01 at z 5.0), inside the
 *   fuselage solid — even the HUD combiner was blocked. Pass 13
 *   raises the eye to 1.05 (in the open lens between deck and
 *   glass) and puts just the dial row on a 0.12 m rail at
 *   (y 0.96, z 5.55), the widest point of the lens (deck 0.91,
 *   canopy 1.045 — 0.135 m tall; a 0.26 m screen cannot fit, so
 *   the F-18 has dials only, by measurement).
 * - f35a: the seat-fairing hump (body_F35_2) runs 1.52 @ z 5.0 down
 *   to 0.95 @ z 6.0 and the eye→panel ray hits it at 1.04 m; the
 *   old panel (centre y 0.97) sat under it. The fascia goes on the
 *   hump slope at (y 1.22, z 5.85), where the ray clears the hump
 *   by ~0.10 m and the strip top (1.38) stays below the fairing
 *   top (1.64).
 * - f22/f15e: open tubs (deck -0.54 / +0.18 at the panel station),
 *   so the slab stays — but its station moves 0.15 m forward
 *   (dz 0.90 -> 1.05): the pilot-side mount swings the dials
 *   0.15 m closer to the eye, which pushed them to 34° off the
 *   look axis and NDC y -1.01 (outside the frame); dz 1.05
 *   restores the pass-12 framing (dial dz ~0.97, NDC y ~-0.82).
 *
 * `dy`/`dz` are the panel centre relative to the eye; `w` the
 * panel/fascia width; `dialDx` the dial half-spacing — the F-18's
 * fascia sits only ~0.55 m from the eye, so its row narrows to
 * stay inside a portrait phone's ~15.5° horizontal half-FOV
 * (ndc |x| <= 1, probe-cockpit).
 */
interface PanelPlace {
  dy: number;
  dz: number;
  w: number;
  dialDx: number;
  /** Dial radius (m) — the F-18 rail sits ~0.52 m from the eye, so
   *  its dials shrink to keep the row inside a portrait frame. */
  dialR: number;
  fascia: boolean;
  /** Rail mode (fa18 only): the panel is just the dial row on a
   *  0.12 m strip — no screens. The lens between deck and canopy
   *  is 0.135 m tall there; a 0.26 m screen cannot fit. */
  rail?: boolean;
}

const PANEL_PLACE: Record<string, PanelPlace> = {
  fa18: { dy: 0.09, dz: 0.55, w: 0.62, dialDx: 0.13, dialR: 0.055, fascia: true, rail: true },
  f22: { dy: 0.78, dz: 1.05, w: 0.9, dialDx: 0.3, dialR: 0.062, fascia: false },
  f15e: { dy: 0.78, dz: 1.05, w: 1.0, dialDx: 0.3, dialR: 0.062, fascia: false },
  f35a: { dy: 0.53, dz: 0.85, w: 0.9, dialDx: 0.18, dialR: 0.062, fascia: true },
};

const DEFAULT_PLACE: PanelPlace = {
  dy: 0.78,
  dz: 1.05,
  w: 0.95,
  dialDx: 0.3,
  dialR: 0.062,
  fascia: false,
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
  dialFace: new THREE.MeshBasicMaterial({
    color: 0x0d1a24,
    // Pass 13: DoubleSide. The dial face is a single-sided circle
    // whose +Z normal points away from the pilot on the slab jets
    // (the panel group's +Z faces the pilot only after the Y-flip
    // in buildCockpit) — FrontSide would backface-cull it from the
    // cockpit camera even when unoccluded (measured
    // normal·toEye = -0.86..-0.89, probe-cockpit).
    side: THREE.DoubleSide,
  }),
  dialRim: new THREE.MeshBasicMaterial({ color: 0xd8e2ea }),
  dialTick: new THREE.MeshBasicMaterial({ color: 0xcfe3ef }),
  dialNeedle: new THREE.MeshBasicMaterial({ color: 0xff5a4a }),
  dialNeedleWhite: new THREE.MeshBasicMaterial({
    color: 0xffffff,
    // Pass 13: DoubleSide — the hub is a circle too (see dialFace).
    side: THREE.DoubleSide,
  }),
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
 * o'clock) and a centre hub. The dial is built in its parent's
 * frame with its face normal along +Z — the parent (the panel group
 * in buildCockpit) aims +Z at the pilot, so the dial carries NO own
 * tilt any more (pass 13: the old per-dial rx 0.25 tilt is gone; the
 * panel group owns the orientation). Parts are named
 * `cockpit_dial_*` for probes; the needle pivot is
 * `cockpit_dial_needle_pivot`, the needle itself
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
  const place = PANEL_PLACE[modelKey] ?? DEFAULT_PLACE;
  const panelW = place.w;
  const panelY = eye.y - place.dy;
  const panelZ = eye.z + place.dz;

  // Instrument panel, in its own group whose local +Z always faces
  // the pilot (pass 13 — the root cause of the blank panels: the
  // old code mounted the screens and dials at faceZ = panelZ + 0.055,
  // on the NOSE side of the opaque 0.08 m panel slab, so the live
  // eye→dial ray hit cockpit_panel at ~1.15 m before the dials at
  // ~1.30 m on all four jets, and the dial faces' +Z normals
  // pointed at the nose, backface-culling them even unoccluded).
  // The group is positioned at the panel centre and oriented per
  // jet: the slab jets keep the 0.25 rad rake (top forward) with a
  // Y-flip so +Z faces the pilot; the fascia jets (fa18/f35a) are
  // aimed at the eye with a lookAt matrix so the strip lies on the
  // dashboard slope facing the pilot. Every instrument is a child
  // of this group at +Z (pilot side), in panel-local coords
  // (x right, y up the face, z out toward the pilot).
  const panel = new THREE.Group();
  panel.name = 'cockpit_panel_mount';
  panel.position.set(0, panelY, panelZ);
  if (place.fascia) {
    // Matrix4.lookAt(eye, target, up): +Z runs from the target
    // toward the eye — the strip's face normal points at the pilot.
    // All in body coords (no world transform involved).
    const m = new THREE.Matrix4().lookAt(
      new THREE.Vector3(0, eye.y, eye.z),
      new THREE.Vector3(0, panelY, panelZ),
      new THREE.Vector3(0, 1, 0),
    );
    panel.quaternion.setFromRotationMatrix(m);
  } else {
    // Qx(0.25) * Qy(PI): +Z -> (0, +0.247, -0.969), the pilot-side
    // face normal with the old 0.25 rad rake; +Y stays upright.
    panel.quaternion
      .setFromAxisAngle(new THREE.Vector3(1, 0, 0), 0.25)
      .multiply(
        new THREE.Quaternion().setFromAxisAngle(
          new THREE.Vector3(0, 1, 0),
          Math.PI,
        ),
      );
  }
  kit.add(panel);
  // Slab (f22/f15e): 0.45 m tall, 0.08 m thick, in the open tub.
  // Fascia (f35a): a 0.34 m thin strip (0.02 m) lying on the
  // visible dashboard slope — the slope IS the panel there.
  // Rail (fa18): a 0.12 m strip carrying just the dial row — the
  // lens between deck and canopy is 0.135 m tall, so no screen fits.
  const thick = place.fascia ? 0.02 : 0.08;
  const height = place.rail ? 0.12 : place.fascia ? 0.34 : 0.45;
  panel.add(
    box('cockpit_panel', panelW, height, thick, MAT.panel, 0, 0, 0),
  );
  // Screens on the panel face: F-35A one wide display, the others two
  // screens. Pass 12 (worker D): the screens were blank dark
  // rectangles and the only "gauges" were two flat dots — the
  // v4.6.1 live check read the panels as "blank grey/black with no
  // dials". Each screen now carries lit symbology (a horizon bar +
  // data blocks, unlit so they read in the dark cockpit), and a row
  // of three round dials with rims, ticks and needles sits below
  // the screens, on the panel face. Panel-local z: the face sits at
  // +thick/2; screens are centred 0.01 in front of it, glyphs and
  // dials 0.024 in front of it (0.002 clear of the screen face, so
  // nothing z-fights, and the dials sit proud of the screens where
  // the rows overlap).
  const face = thick / 2;
  const scrZ = face + 0.01;
  const scrY = 0.05;
  const glyphZ = face + 0.024;
  const glyph = (cx: number, w: number): void => {
    panel.add(
      box('cockpit_screen_line', w * 0.8, 0.012, 0.004, MAT.screenLine, cx, scrY, glyphZ),
    );
    panel.add(
      box('cockpit_screen_line', 0.05, 0.035, 0.004, MAT.screenLineCyan, cx - w * 0.22, scrY + 0.06, glyphZ),
    );
    panel.add(
      box('cockpit_screen_line', 0.05, 0.02, 0.004, MAT.screenLineAmber, cx + w * 0.22, scrY - 0.07, glyphZ),
    );
  };
  // Rail mode (fa18): no screens — the lens is 0.135 m tall and a
  // 0.26 m screen cannot fit. The dials are the instruments.
  if (!place.rail) {
    if (modelKey === 'f35a') {
      panel.add(
        box('cockpit_screen', panelW * 0.82, 0.26, 0.02, MAT.screenWide, 0, scrY, scrZ),
      );
      glyph(0, panelW * 0.82);
    } else {
      const sw = panelW * 0.36;
      panel.add(
        box('cockpit_screen_L', sw, 0.26, 0.02, MAT.screen, -panelW * 0.24, scrY, scrZ),
      );
      panel.add(
        box('cockpit_screen_R', sw, 0.26, 0.02, MAT.screen, panelW * 0.24, scrY, scrZ),
      );
      glyph(-panelW * 0.24, sw);
      glyph(panelW * 0.24, sw);
    }
  }
  // Dial row (airspeed / attitude / altitude standbys): white rims,
  // ticks and coloured needles, so they read as instruments. The
  // row sits on the panel face below the screens (dial centre
  // scrY - 0.14, top scrY - 0.14 + 0.062) — or centred on the rail.
  const dialY = place.rail ? 0 : scrY - 0.14;
  const dialZ = face + 0.024;
  const dialR = place.dialR;
  const dialAngles = [-40, 15, 65];
  const dialMats = [MAT.dialNeedle, MAT.dialNeedleWhite, MAT.dialNeedle];
  [-place.dialDx, 0, place.dialDx].forEach((dx, i) => {
    panel.add(
      dial(
        `cockpit_dial_${i}`,
        dialR,
        dialAngles[i]!,
        dialMats[i]!,
        dx,
        dialY,
        dialZ,
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
  // Coaming: the padded rim at the panel's base, following the panel
  // (pass 13: it used to sit at a fixed eye offset, which stranded
  // it 0.87 m below the raised F-18/F-35A fascias).
  kit.add(
    box('cockpit_coaming', panelW, 0.08, 0.45, MAT.frame, 0, panelY - 0.26, panelZ - 0.05),
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
