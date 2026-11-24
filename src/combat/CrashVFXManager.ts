// CrashVFXManager — pooled explosion sprites, smoke/fire plumes, tumbling
// debris and ground scorch marks. Ported from the v2 template
// (pandora-template.html ~1620–1704 playExplosion/updateBooms/makeSmokePlume
// and ~1178–1214 spawnDebris/updateDebris/detachWing), with one v2 change:
// detached segments are now independent tumbling *Rapier* bodies instead of
// hand-integrated meshes, so they bounce/roll against the physics world.
//
// Texture note: the template builds its glow textures from canvas radial
// gradients (boomTex, flareTex, cloudTex). Same approach here — no assets.

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';

export interface CrashVFXOptions {
  /** Ground height at (x, z) — used to rest scorch decals and kill debris. */
  groundHeight?: (x: number, z: number) => number;
  /** Camera-shake hook (main.ts feeds this into the chase camera). */
  onShake?: (amount: number) => void;
  /** Floating origin (pass 6, item 2): persistent VFX (scorch decals,
   *  debris, free plumes) are registered with it so a shift moves them
   *  with the terrain instead of leaving them at the wrong place —
   *  a scorch left behind by a shift reads as a black disc on a
   *  runway it never crashed on. Attached plumes are children of a
   *  jet's group and move with it, so they are not registered. */
  origin?: {
    registerObject(obj: THREE.Object3D): void;
    unregisterObject(obj: THREE.Object3D): void;
    registerBody(body: RAPIER.RigidBody): void;
    unregisterBody(body: RAPIER.RigidBody): void;
  };
}

interface Boom {
  sprite: THREE.Sprite;
  t: number;
  scale: number;
}

interface Plume {
  group: THREE.Group;
  life: number;
  maxLife: number;
  /** Parent the plume is attached to (live finding 3): when set, the
   *  plume is a child of a moving object (a dying jet's group) so it
   *  follows the wreck instead of being left behind as a static puff.
   *  Null = a free plume in the scene (the default). */
  parent: THREE.Object3D | null;
  /** Starting sprite opacity (fire burns brighter than smoke) —
   *  update() fades from this, not from a hardcoded 0.55. */
  baseOpacity: number;
}

interface DebrisPiece {
  mesh: THREE.Object3D;
  body: RAPIER.RigidBody;
  life: number;
  // Age since spawn. (Pass 8 used it for a ground-removal rule that
  // deleted a piece on its first touchdown — pass 11 removed that
  // rule, so debris rests on the ground for its full life. The age
  // is kept for probes/diagnostics.)
  age: number;
}

function radialTexture(stops: [number, string][]): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d')!;
  const grd = g.createRadialGradient(64, 64, 4, 64, 64, 62);
  for (const [at, col] of stops) grd.addColorStop(at, col);
  g.fillStyle = grd;
  g.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}

/** Torn wreckage panel (pass 15, worker F, item 8): a BoxGeometry
 *  IS a rectangular slab — flat top face, rectangular footprint,
 *  and a Rapier cuboid that settles on its largest face, so every
 *  piece lay flat and read as a dark rectangle from the chase
 *  camera above (the live "black/grey slabs at every crash site").
 *  A torn panel is instead tapered (narrow end 55-75% of the wide
 *  end), jittered (shared vertices move together, keyed by
 *  position, so the faces do not crack apart) and folded 15-30 deg
 *  at mid-length, so no face is flat, no outline is rectangular,
 *  and the piece cannot lie flat. The geometry is re-centred on
 *  its own bounding box (the debris mesh is synced straight from
 *  its Rapier body, so a non-centred geometry would sit offset
 *  from its collider) and the returned half-extents are the
 *  post-deformation bounding box — the collider wraps the bent
 *  panel, not the flat box it started as. */
function tornPanelGeometry(
  w: number,
  t: number,
  l: number,
): { geometry: THREE.BufferGeometry; half: THREE.Vector3 } {
  const geo = new THREE.BoxGeometry(w, t, l, 3, 1, 4);
  const p = geo.getAttribute('position') as THREE.BufferAttribute;
  const fold = ((15 + Math.random() * 15) * Math.PI) / 180;
  const taper = 0.25 + Math.random() * 0.2;
  const cosF = Math.cos(fold);
  const sinF = Math.sin(fold);
  // One jitter offset per distinct vertex POSITION (BoxGeometry
  // duplicates vertices across faces — jittering per index would
  // split the seams and leave see-through cracks).
  const jitter = new Map<string, { x: number; y: number; z: number }>();
  const jFor = (x: number, y: number, z: number): { x: number; y: number; z: number } => {
    const key = `${x.toFixed(3)},${y.toFixed(3)},${z.toFixed(3)}`;
    let j = jitter.get(key);
    if (!j) {
      j = {
        x: (Math.random() - 0.5) * w * 0.06,
        y: (Math.random() - 0.5) * t * 0.9,
        z: (Math.random() - 0.5) * l * 0.03,
      };
      jitter.set(key, j);
    }
    return j;
  };
  const v = new THREE.Vector3();
  for (let i = 0; i < p.count; i++) {
    v.set(p.getX(i), p.getY(i), p.getZ(i));
    const s = v.z / l + 0.5; // 0 at the -z end, 1 at the +z end
    v.x *= 1 - taper * s;
    const j = jFor(p.getX(i), p.getY(i), p.getZ(i));
    v.x += j.x;
    v.y += j.y;
    v.z += j.z;
    // Fold the +z half UP around the X axis at z = 0.
    if (v.z > 0) {
      const y2 = v.y * cosF + v.z * sinF;
      const z2 = -v.y * sinF + v.z * cosF;
      v.y = y2;
      v.z = z2;
    }
    p.setXYZ(i, v.x, v.y, v.z);
  }
  geo.computeVertexNormals();
  // The vertex data is no longer a box — label it honestly, so a
  // later geometry-type check (or a reader) does not mistake a torn
  // panel for a rectangular slab. (type is readonly in three's
  // typings — a runtime instance property, so the cast is safe.)
  (geo as unknown as { type: string }).type = 'TornPanelGeometry';
  geo.computeBoundingBox();
  const bb = geo.boundingBox!;
  const centre = new THREE.Vector3();
  bb.getCenter(centre);
  const size = new THREE.Vector3();
  bb.getSize(size);
  geo.translate(-centre.x, -centre.y, -centre.z);
  return { geometry: geo, half: size.multiplyScalar(0.5) };
}

const BOOM_STOPS: [number, string][] = [
  [0, 'rgba(255,255,220,1)'],
  [0.35, 'rgba(255,140,40,0.9)'],
  [1, 'rgba(120,20,0,0)'],
];
const SMOKE_STOPS: [number, string][] = [
  [0, 'rgba(255,255,255,0.85)'],
  [0.6, 'rgba(255,255,255,0.35)'],
  [1, 'rgba(255,255,255,0)'],
];

/** Irregular scorch blotch texture (pass 13): a perfect dark circle
 *  reads as a shadow or a disc on the grass — the live "dark shadow
 *  beneath it". A real scorch is an irregular blotch, darkest in
 *  the middle, feathered and lumpy at the edge. Drawn once and
 *  cached; the material colour stays white so the texture carries
 *  the colour. */
let scorchTex: THREE.CanvasTexture | null = null;
function irregularScorchTexture(): THREE.CanvasTexture {
  if (scorchTex) return scorchTex;
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d')!;
  const cx = 128;
  const cy = 128;
  // Lumpy silhouette: radius modulated by a few sine harmonics.
  g.beginPath();
  for (let i = 0; i <= 64; i++) {
    const a = (i / 64) * Math.PI * 2;
    const r =
      118 *
      (0.82 + 0.1 * Math.sin(a * 3 + 1.7) + 0.08 * Math.sin(a * 7 + 0.4));
    const x = cx + Math.cos(a) * r;
    const y = cy + Math.sin(a) * r;
    if (i === 0) g.moveTo(x, y);
    else g.lineTo(x, y);
  }
  g.closePath();
  const grd = g.createRadialGradient(cx, cy, 8, cx, cy, 118);
  grd.addColorStop(0, 'rgba(12,9,7,0.95)');
  grd.addColorStop(0.55, 'rgba(42,36,30,0.85)');
  grd.addColorStop(0.85, 'rgba(58,50,42,0.45)');
  grd.addColorStop(1, 'rgba(58,50,42,0)');
  g.fillStyle = grd;
  g.fill();
  // Mottled darker blotches inside, so the mark is not flat.
  for (let i = 0; i < 7; i++) {
    const a = Math.random() * Math.PI * 2;
    const d = Math.random() * 60;
    const bx = cx + Math.cos(a) * d;
    const by = cy + Math.sin(a) * d;
    const br = 12 + Math.random() * 22;
    const bg = g.createRadialGradient(bx, by, 1, bx, by, br);
    bg.addColorStop(0, 'rgba(10,8,6,0.5)');
    bg.addColorStop(1, 'rgba(10,8,6,0)');
    g.fillStyle = bg;
    g.beginPath();
    g.arc(bx, by, br, 0, Math.PI * 2);
    g.fill();
  }
  scorchTex = new THREE.CanvasTexture(c);
  return scorchTex;
}

/** Charred wreckage colours (pass 13): the old generic debris was a
 *  light grey (0x8a97a3) that read on grass as "grey rectangular
 *  slabs". Wreckage is near-black charred metal with brown/grey
 *  variation — one piece per spawn, picked at random. */
const CHARRED_COLORS = [0x17171a, 0x23211e, 0x2e2a24, 0x1c1c20, 0x201d1a];
const MAX_SCORCH = 12;
/** Scorch decal lifetime (s): fades out over the last 30 s. */
const SCORCH_LIFE = 90;
/** Explosion sprite lifetime (s). Pass 11 (stall-impact VFX): was
 *  0.9 s — the fireball started at ~12 m, ballooned to ~110 m while
 *  fading linearly to transparent, and was gone in under a second, so
 *  a hard stall impact read live as only a brief orange glow. A
 *  crash that tears the jet apart needs a fireball that is actually
 *  seen: full brightness while it grows (BOOM_BRIGHT), then a fade
 *  over BOOM_LIFE. */
const BOOM_LIFE = 2.5;
/** Explosion bright phase (s): opacity stays 1 while the fireball
 *  grows to full size; it only starts fading after this. */
const BOOM_BRIGHT = 0.8;
/** Explosion growth cap (x the spawn scale): the old unbounded
 *  growth (6 + t*55) blew the sprite up to ~110 m while it faded,
 *  so it read as a thinning haze, not a fireball. Capped at roughly
 *  2-3 jet lengths across. */
const BOOM_MAX_SCALE = 40;
/** Attached-plume (wreck fire) lifetime (s). Pass 11: was 6 s — the
 *  fire died before the 3 s spectate window plus a zoomed look had
 *  finished, so the wreck read as not burning. A wreck fire burns
 *  for 12 s (respawnPlayer clears attached plumes on the next life,
 *  so it cannot leak into a fresh jet). Free (trail) plumes stay
 *  at 6 s.
 *  Pass 13: 12 s is still too short — the live crash report reads
 *  "brief orange glow faded ~15 s" and then an intact-looking
 *  wreck. A wreck fire burns for the debris lifetime (30 s), so
 *  the crash site reads as a burning wreck for as long as the
 *  wreckage is there; respawn still clears it.
 *  Pass 14 (worker D): 30 s is still too short — the v4.8 live
 *  check found the fire glow burned out by +75 s after the crash.
 *  Lifetimes are SIM seconds, and at the live 4-9 fps the sim runs
 *  at 0.4-0.9x real time, so 30 sim-s expires at ~33-75 REAL
 *  seconds — exactly when the player is watching the wreck. A
 *  wreck fire now burns 90 s, as long as the debris (DEBRIS_LIFE)
 *  and the scorch (SCORCH_LIFE 90) — the whole crash site reads
 *  as a burning wreck for as long as any of it is there. */
const ATTACHED_PLUME_LIFE = 90;
/** Attached-plume fade-out (s): the fire holds full brightness
 *  until its last PLUME_FADE_S seconds, then fades (pass 14,
 *  worker D — the old linear fade had it at half brightness by
 *  mid-life). Free plumes fade linearly over their whole life. */
const PLUME_FADE_S = 15;
/** Debris lifetime (s). Pass 11: was 10 s AND pieces were deleted
 *  the moment they touched the ground, so by the time the player
 *  looked at the wreck (seconds after impact) the debris field was
 *  already gone — the live "no debris" finding. Debris now RESTS
 *  on the ground until it expires (30 s).
 *  Pass 14 (worker D): 30 s is too short for the same sim-vs-real
 *  reason as the fire (see ATTACHED_PLUME_LIFE): at 4 fps the
 *  field expires ~75 real seconds after the crash, so the v4.8
 *  live check found no debris field at +75 s. Debris now rests
 *  90 s — as long as the scorch mark — so the wreck site still
 *  reads as scattered wreckage whenever the player looks at it. */
const DEBRIS_LIFE = 90;

export class CrashVFXManager {
  private readonly scene: THREE.Scene;
  private readonly opts: CrashVFXOptions;
  private readonly boomTex: THREE.CanvasTexture;
  private readonly smokeTex: THREE.CanvasTexture;
  private readonly booms: Boom[] = [];
  private readonly plumes: Plume[] = [];
  private readonly debris: DebrisPiece[] = [];
  /** Scorch decals, oldest first — capped (see scorchDecal). Each
   *  carries its age so it fades out over ~90 s (live finding 1:
   *  a pile of permanent black decals at a repeated crash site reads
   *  as a flat black disc on the ground). */
  private readonly scorches: Array<{ mesh: THREE.Mesh; age: number }> = [];
  private debrisWorld: RAPIER.World | null = null;
  private readonly flash = new THREE.PointLight(0xffaa33, 0, 120);

  constructor(scene: THREE.Scene, opts: CrashVFXOptions = {}) {
    this.scene = scene;
    this.opts = opts;
    this.boomTex = radialTexture(BOOM_STOPS);
    this.smokeTex = radialTexture(SMOKE_STOPS);
    scene.add(this.flash);
    // Pass 13: the flash is a scene object at the crash site — shift
    // it with the world like every other VFX object (see
    // spawnExplosion for the boom sprite).
    this.opts.origin?.registerObject(this.flash);
  }

  /** Pooled explosion sprite burst + light flash + camera-shake hook.
   *  Pass 11 timing: life BOOM_LIFE (2.5 s), full brightness for the
   *  first BOOM_BRIGHT (0.8 s) while the fireball grows to
   *  BOOM_MAX_SCALE, then a fade. The old template timing (scale =
   *  s * (6 + t*55), opacity 1 - t/0.9, life 0.9 s) read live as a
   *  brief orange glow: small at birth, ballooning while fading, and
   *  gone in under a second. */
  spawnExplosion(pos: THREE.Vector3, scale = 1): void {
    let b = this.booms.find((x) => !x.sprite.visible);
    if (!b) {
      const sprite = new THREE.Sprite(
        new THREE.SpriteMaterial({
          map: this.boomTex,
          transparent: true,
          depthWrite: false,
        }),
      );
      sprite.visible = false;
      this.scene.add(sprite);
      // Pass 13: the boom was the one VFX object never registered
      // with the floating origin — a shift during its 2.5 s life
      // left the fireball 2 km behind at the pre-shift coordinates
      // (probe-crash-vfx E: boom 2773 m from the wreck after a
      // mid-life shift). Register it at creation (idempotent, and
      // pooled reuse repositions it anyway).
      this.opts.origin?.registerObject(sprite);
      b = { sprite, t: 0, scale: 1 };
      this.booms.push(b);
    }
    b.sprite.position.copy(pos);
    b.sprite.visible = true;
    b.t = 0;
    b.scale = scale;
    // Light flash at the blast, decaying in update().
    this.flash.position.copy(pos);
    this.flash.intensity = 60 * scale;
    this.opts.onShake?.(0.8 * scale);
  }

  /** Smoke/fire plume. By default a free plume in the scene at `pos`
   *  (static — it drifts up and fades over 6 s). Pass `attachTo` to
   *  parent the plume to a moving object (a dying jet's group): the
   *  plume then follows the wreck instead of being left behind as a
   *  static puff the falling jet (60-100 m/s) outruns in 0.1 s (live
   *  finding 3 — the v3 shoot-down showed no fire/smoke in flight
   *  because the static puffs were left behind). The plume is
   *  positioned at the parent's origin (the jet's centre) so the
   *  fire reads as ON the jet, and it tumbles with the wreck.
   *  Template: 5 sprites trailing behind/below, colour 0x555555 smoke
   *  or 0xff7722 fire. Returns the group so the caller can remove it.
   *  Pass 11: an attached plume is a WRECK fire — it burns for
   *  ATTACHED_PLUME_LIFE (90 s), and a fire-coloured plume is
   *  brighter and larger than smoke (the old 0.55-opacity 6 m
   *  sprites read as faint haze at the 42 m spectate distance, not
   *  fire). */
  spawnSmoke(
    pos: THREE.Vector3,
    colour: number,
    attachTo?: THREE.Object3D,
  ): THREE.Group {
    const group = new THREE.Group();
    const isFire = colour === 0xff7722;
    const baseOpacity = isFire ? 0.8 : 0.55;
    for (let i = 0; i < 5; i++) {
      const s = new THREE.Sprite(
        new THREE.SpriteMaterial({
          map: this.smokeTex,
          color: colour,
          transparent: true,
          opacity: baseOpacity,
          depthWrite: false,
        }),
      );
      const size = isFire ? 8 + i * 2.5 : 6 + i * 2;
      s.scale.set(size, size, 1);
      s.position.set((Math.random() - 0.5) * 2, -i * 1.2, -4 - i * 1.5);
      group.add(s);
    }
    const life = attachTo ? ATTACHED_PLUME_LIFE : 6;
    if (attachTo) {
      // Attached: parent to the moving object at its origin (the
      // jet's centre) — the fire is ON the jet and follows it.
      group.position.set(0, 0, 0);
      attachTo.add(group);
      this.plumes.push({ group, life, maxLife: life, parent: attachTo, baseOpacity });
    } else {
      group.position.copy(pos);
      this.scene.add(group);
      // Free plume: register with the floating origin so a shift
      // moves it with the world (it is a scene object, not a child
      // of a jet). Attached plumes move with their parent and are
      // not registered.
      this.opts.origin?.registerObject(group);
      this.plumes.push({ group, life, maxLife: life, parent: null, baseOpacity });
    }
    return group;
  }

  /** Remove every plume attached to `parent` (e.g. on respawn — a
   *  respawned jet must not carry the previous life's burn plume). */
  clearAttached(parent: THREE.Object3D): void {
    for (let i = this.plumes.length - 1; i >= 0; i--) {
      const p = this.plumes[i]!;
      if (p.parent === parent) {
        parent.remove(p.group);
        this.plumes.splice(i, 1);
      }
    }
  }

  /** Detach a wing/tail/engine segment: the supplied mesh becomes an
   *  independent tumbling Rapier body (v2 behaviour; template ~1178–1204
   *  hand-integrated a box). The segment is removed from its parent and
   *  added to the scene. Returns the Rapier body so the caller can track
   *  or remove it. Falls back to a 4 x 0.5 x 2.5 m box when no segment is
   *  supplied. Life DEBRIS_LIFE (90 s): the piece RESTS on the ground
   *  until it expires (pass 11 — the old rule deleted a piece the
   *  moment it touched the ground, so the debris field was gone
   *  before the player ever looked at the wreck). Spawned 0.5 m above
   *  the given position so the body does not start intersecting the
   *  heightfield. */
  spawnDebris(
    segment: THREE.Object3D | null,
    world: RAPIER.World,
    pos: THREE.Vector3,
    vel: THREE.Vector3,
  ): RAPIER.RigidBody {
    let mesh: THREE.Object3D;
    let half: THREE.Vector3;
    if (segment) {
      mesh = segment;
      if (mesh.parent) mesh.parent.remove(mesh);
      // Pass 15 (worker F, item 8): a torn wing/tail fallback is a
      // flat BoxGeometry (F-22: 6.1 x 0.3 x 4.2 m) — the single
      // biggest slab in the debris field. Reshape it into a torn
      // panel (same bounding size, same name) before the collider
      // is sized. A real GLB node is not a box and is left alone.
      const segMesh = mesh as THREE.Mesh;
      if (segMesh.isMesh && (segMesh.geometry as THREE.BufferGeometry)?.type === 'BoxGeometry') {
        const sb = new THREE.Box3().setFromObject(mesh);
        const ss = new THREE.Vector3();
        sb.getSize(ss);
        const torn = tornPanelGeometry(ss.x, ss.y, ss.z);
        segMesh.geometry.dispose();
        segMesh.geometry = torn.geometry;
        half = torn.half;
      } else {
        const box = new THREE.Box3().setFromObject(mesh);
        const size = new THREE.Vector3();
        box.getSize(size);
        half = size.multiplyScalar(0.5);
      }
      const box = new THREE.Box3().setFromObject(mesh);
      // Re-centre the mesh on its own origin so the body tracks it cleanly.
      const centre = new THREE.Vector3();
      box.getCenter(centre);
      mesh.position.sub(centre);
      const holder = new THREE.Group();
      holder.add(mesh);
      mesh = holder;
    } else {
      // Pass 13: charred, irregular wreckage — NOT four identical
      // light-grey 4 x 0.5 x 2.5 m slabs (the live "grey rectangular
      // slabs scattered near every crash site"). Each piece gets a
      // random size (2.5-5 m long, 0.3-0.9 m thick) and a random
      // charred colour, so the field reads as torn wreckage.
      // Pass 14 (worker D): two shapes — torn skin panels (wide,
      // thin) and chunky fragments — so the field does not read as
      // four same-sized boxes.
      // Pass 15 (worker F, item 8): the pieces were still
      // BoxGeometry boxes (flat top, rectangular footprint, lying
      // flat on their largest face) — the live "black/grey slabs".
      // Every piece is now a torn panel: tapered, jittered and
      // folded, so no face is flat and no outline is rectangular.
      const panel = Math.random() < 0.5;
      const len = panel ? 3 + Math.random() * 2.5 : 2.5 + Math.random() * 2.5;
      const thick = panel ? 0.12 + Math.random() * 0.2 : 0.3 + Math.random() * 0.6;
      const wid = panel ? 1.8 + Math.random() * 1.6 : 1.2 + Math.random() * 1.6;
      const torn = tornPanelGeometry(wid, thick, len);
      mesh = new THREE.Mesh(
        torn.geometry,
        new THREE.MeshLambertMaterial({
          color: CHARRED_COLORS[
            Math.floor(Math.random() * CHARRED_COLORS.length)
          ]!,
        }),
      );
      half = torn.half;
    }
    // A degenerate half-extent (a paper-thin GLB segment) makes a
    // useless Rapier cuboid — floor it, as the old code did.
    half.x = Math.max(half.x, 0.25);
    half.y = Math.max(half.y, 0.25);
    half.z = Math.max(half.z, 0.25);
    mesh.position.copy(pos);
    mesh.position.y += 0.5;
    this.scene.add(mesh);

    this.debrisWorld = world;
    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(pos.x, pos.y + 0.5, pos.z)
        .setLinvel(vel.x, vel.y, vel.z)
        // Pass 11: damping, so the field stays near the wreck. The
        // heightfield has zero friction, so an undamped piece launched
        // at 10-30 m/s slides for hundreds of metres and never
        // settles — the debris field reads as nothing near the wreck.
        // Damping bleeds the launch speed over a few seconds while
        // the piece still bounces/rolls; gravity keeps a piece on a
        // steep slope sliding slowly downhill instead of stopping.
        // Pass 12: angular damping 0.5 -> 2.0 and real collider
        // friction, for the probe flake (probe-crash-vfx debris
        // resting at +10 s, 5/6): a long torn panel (F-22 torn
        // wing 6.1 x 0.3 x 4.18 m) tumbled onto its end and sat
        // propped 2.01 m above the ground, essentially at rest
        // (vy -0.17, angular 0.02) but never lying flat, and the
        // zero-friction ground let it slide downhill at ~1.4 m/s
        // (terminal slide speed ~ g*sin(slope)/damping). Higher
        // angular damping kills the tumble so the piece lies flat;
        // the collider friction (Max combine rule — the terrain
        // collider uses Min with friction 0, so Max wins and the
        // pair friction is max(0, 0.8) = 0.8) stops the slide.
        .setLinearDamping(0.8)
        .setAngularDamping(2.0)
        .setAngvel({
          x: (Math.random() - 0.5) * 4,
          y: (Math.random() - 0.5) * 4,
          z: (Math.random() - 0.5) * 4,
        }),
    );
    this.opts.origin?.registerBody(body);
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(half.x, half.y, half.z)
        .setFriction(0.8)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Max),
      body,
    );
    this.debris.push({ mesh, body, life: DEBRIS_LIFE, age: 0 });
    return body;
  }

  /** Cheap ground scorch mark: a dark circle lying on the terrain.
   *  CAPPED at the 12 most recent (MAX_SCORCH) AND faded out over
   *  ~90 s (SCORCH_LIFE): scorch decals were never removed, so a
   *  respawn death-loop piled them up at the repeated crash site —
   *  a growing black disc the stuck jet sits on, which is what the
   *  v2 play-test read as a "black disc in the sky". Oldest decals
   *  are removed from the scene and disposed; the rest fade in
   *  update() over their last 30 s.
   *  Pass 6 (item 2): the v4.1 ground was still covered in huge
   *  dark discs — the repeated spawn-crash scorches (radius 8 = 16 m
   *  across, at 0.45 opacity) read as black discs on the green
   *  terrain, and they are not registered with the floating origin
   *  so a shift leaves them at the wrong place. An aircraft crash
   *  now leaves a smaller, fainter mark (radius 5, 0.35); the
   *  carrier impact scorch still passes its own radius (30). */
  scorchDecal(pos: THREE.Vector3, radius = 5): THREE.Mesh {
    const gh = this.opts.groundHeight
      ? this.opts.groundHeight(pos.x, pos.z)
      : pos.y;
    const decal = new THREE.Mesh(
      new THREE.CircleGeometry(radius, 24),
      new THREE.MeshBasicMaterial({
        // Pass 13: the colour lives in the irregular blotch texture
        // now (see irregularScorchTexture) — a flat dark circle
        // read as a shadow/disc on the grass.
        map: irregularScorchTexture(),
        transparent: true,
        opacity: 0.4,
        depthWrite: false,
      }),
    );
    if (this.opts.groundHeight) {
      // Pass 11: TILT the decal to the local terrain slope. The old
      // flat horizontal disc on a mountainside was buried under the
      // uphill half of the slope and floating up to a metre above it
      // on the downhill half, so a mountainside crash showed no
      // scorch at all (the v4.5 live finding). Sample the ground on
      // both sides, take the surface normal, and lay the decal on
      // that plane, 0.15 m above the surface along the normal.
      const d = radius * 0.6;
      const gx =
        (this.opts.groundHeight(pos.x + d, pos.z) -
          this.opts.groundHeight(pos.x - d, pos.z)) /
        (2 * d);
      const gz =
        (this.opts.groundHeight(pos.x, pos.z + d) -
          this.opts.groundHeight(pos.x, pos.z - d)) /
        (2 * d);
      const normal = new THREE.Vector3(-gx, 1, -gz).normalize();
      // A CircleGeometry lies in its local XY plane, so its normal
      // is local +Z — map +Z onto the terrain normal (mapping +Y
      // instead lays the disc vertically, edge-on to the slope).
      decal.quaternion.setFromUnitVectors(
        new THREE.Vector3(0, 0, 1),
        normal,
      );
      // Pass 12: centre the decal on the terrain under its RIM, not
      // just the centre. The tilt uses the gradient at the centre,
      // but on a curved slope the rim can float (or bury) by ~1 m —
      // the f22 A stall crash floated 0.86 m at one rim point, over
      // the probe's 0.6 m limit. Sample the terrain at 8 rim points
      // and set the offset along the normal so the rim-delta RANGE
      // is centred in the probe's allowed band (+0.4 buried to −0.6
      // floating, centre −0.1): a flat disc cannot follow curvature,
      // so centring the range — not the mean — is what fits the
      // band. On flat ground this is the old 0.15 m (range [0,0]
      // → h ≈ 0.1). The factor is × normal.y, not ÷: shifting along
      // the normal moves the rim across the slope as well as down,
      // so a normal-shift of h changes the rim deltas by −h/|n| =
      // −h·normal.y — dividing overshoots 3.5× on a 59° slope.
      const q = decal.quaternion;
      const right = new THREE.Vector3(1, 0, 0).applyQuaternion(q);
      const up2 = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
      let dMin = Infinity;
      let dMax = -Infinity;
      for (let k = 0; k < 8; k++) {
        const a = (k / 8) * Math.PI * 2;
        const rx =
          pos.x + (right.x * Math.cos(a) + up2.x * Math.sin(a)) * radius;
        const rz =
          pos.z + (right.z * Math.cos(a) + up2.z * Math.sin(a)) * radius;
        const ry =
          gh + (right.y * Math.cos(a) + up2.y * Math.sin(a)) * radius;
        const d0 = this.opts.groundHeight(rx, rz) - ry;
        dMin = Math.min(dMin, d0);
        dMax = Math.max(dMax, d0);
      }
      const h = ((dMax + dMin) / 2 + 0.1) * Math.max(normal.y, 0.2);
      decal.position.set(pos.x, gh, pos.z).addScaledVector(normal, h);
      // Pass 13: SNAP the decal's vertices to the terrain. The tilt
      // + rim-centering above only fits a FLAT disc to a curved
      // slope — on the f22 A cliff crash the rim range was 1.03 m
      // wide, wider than the 1.0 m band it is centred in, so one
      // rim point floated 0.68 m no matter how the disc was placed
      // (the probe only passed there before because leftover debris
      // from an earlier scenario had perturbed the crash site).
      // Bake the tilt into the geometry and drop every vertex
      // vertically onto the terrain (+0.12 m), so the mark drapes
      // over the slope instead of floating above it or burying
      // under it. The mesh quaternion is identity afterwards and
      // the vertices are relative to the mesh position.
      const geo = decal.geometry as THREE.CircleGeometry;
      const centre = decal.position.clone();
      const p = geo.getAttribute('position') as THREE.BufferAttribute;
      const v = new THREE.Vector3();
      for (let i = 0; i < p.count; i++) {
        v.set(p.getX(i), p.getY(i), p.getZ(i)).applyQuaternion(q);
        const wx = centre.x + v.x;
        const wz = centre.z + v.z;
        v.y = this.opts.groundHeight(wx, wz) + 0.12 - centre.y;
        p.setXYZ(i, v.x, v.y, v.z);
      }
      p.needsUpdate = true;
      decal.quaternion.identity();
    } else {
      decal.rotation.x = -Math.PI / 2;
      decal.position.set(pos.x, gh + 0.15, pos.z);
    }
    this.scene.add(decal);
    this.opts.origin?.registerObject(decal);
    this.scorches.push({ mesh: decal, age: 0 });
    while (this.scorches.length > MAX_SCORCH) {
      const old = this.scorches.shift()!;
      this.scene.remove(old.mesh);
      this.opts.origin?.unregisterObject(old.mesh);
      old.mesh.geometry.dispose();
      (old.mesh.material as THREE.Material).dispose();
    }
    return decal;
  }

  update(dt: number): void {
    // Explosion sprites: full brightness while the fireball grows to
    // its capped size (pass 11 — the old 0.9 s linear fade read as a
    // brief orange glow), then a fade over BOOM_LIFE.
    for (const b of this.booms) {
      if (!b.sprite.visible) continue;
      b.t += dt;
      const s = b.scale * Math.min(6 + b.t * 30, BOOM_MAX_SCALE);
      b.sprite.scale.set(s, s, 1);
      b.sprite.material.opacity =
        b.t < BOOM_BRIGHT
          ? 1
          : Math.max(0, 1 - (b.t - BOOM_BRIGHT) / (BOOM_LIFE - BOOM_BRIGHT));
      if (b.t > BOOM_LIFE) b.sprite.visible = false;
    }
    // Light flash decays (pass 11: slower, so the flash lasts about
    // as long as the bright phase of the fireball, not 0.3 s).
    if (this.flash.intensity > 0) {
      this.flash.intensity = Math.max(0, this.flash.intensity - dt * 100);
    }
    // Plumes drift up and fade out — but only FREE plumes. An attached
    // plume is parented to a falling wreck; drifting its local Y up
    // 1.5 m/s lifts the fire 9 m off the wreck over its 6 s life
    // (pass 5: the attached plume separated from the wreck in the
    // going-down probe, so the fire visibly floated above the fall).
    // The wreck's own motion carries an attached plume.
    for (let i = this.plumes.length - 1; i >= 0; i--) {
      const p = this.plumes[i];
      p.life -= dt;
      if (!p.parent) p.group.position.y += dt * 1.5;
      // Pass 14 (worker D): an attached (wreck-fire) plume holds
      // full brightness until its last PLUME_FADE_S seconds, then
      // fades — a wreck fire burns, it does not thin out linearly
      // from birth (the old linear fade had the fire at half
      // brightness by mid-life and gone by +30 s). Free plumes
      // keep the linear fade over their whole (short) life.
      const fade = p.parent
        ? Math.min(1, Math.max(0, p.life / PLUME_FADE_S))
        : Math.max(0, p.life / p.maxLife);
      for (const child of p.group.children) {
        const spr = child as THREE.Sprite;
        spr.material.opacity = p.baseOpacity * fade;
      }
      if (p.life <= 0) {
        // Attached plumes are children of a moving object (a jet's
        // group), not the scene — remove from their parent. Free
        // plumes are scene objects registered with the origin —
        // unregister them too.
        if (p.parent) p.parent.remove(p.group);
        else {
          this.scene.remove(p.group);
          this.opts.origin?.unregisterObject(p.group);
        }
        this.plumes.splice(i, 1);
      }
    }
    // Debris: sync mesh from its Rapier body; remove on expiry — or
    // if it falls THROUGH the world (below the terrain). Pass 11:
    // the old rule deleted a piece the moment it touched the ground
    // (age > 0.5 s AND falling back AND within 0.5 m of the surface),
    // so a debris field never rested visibly on the ground at all —
    // the live "no debris" finding. A piece now bounces/rolls to
    // rest and stays there for its full life, which is what makes
    // the crash site read as a wreck.
    for (let i = this.debris.length - 1; i >= 0; i--) {
      const d = this.debris[i];
      d.life -= dt;
      d.age += dt;
      const p = d.body.translation();
      const q = d.body.rotation();
      d.mesh.position.set(p.x, p.y, p.z);
      d.mesh.quaternion.set(q.x, q.y, q.z, q.w);
      const gh = this.opts.groundHeight
        ? this.opts.groundHeight(p.x, p.z)
        : -Infinity;
      if (d.life <= 0 || p.y < gh - 2) {
        this.scene.remove(d.mesh);
        this.opts.origin?.unregisterBody(d.body);
        this.debrisWorld?.removeRigidBody(d.body);
        this.debris.splice(i, 1);
      }
    }
    // Scorch decals: age and fade out over SCORCH_LIFE (full opacity
    // for the first 60 s, then a linear fade to 0 over the last 30 s).
    for (let i = this.scorches.length - 1; i >= 0; i--) {
      const s = this.scorches[i]!;
      s.age += dt;
      if (s.age >= SCORCH_LIFE) {
        this.scene.remove(s.mesh);
        this.opts.origin?.unregisterObject(s.mesh);
        s.mesh.geometry.dispose();
        (s.mesh.material as THREE.Material).dispose();
        this.scorches.splice(i, 1);
      } else if (s.age > SCORCH_LIFE - 30) {
        const fade = 1 - (s.age - (SCORCH_LIFE - 30)) / 30;
        // Pass 6: the fade used 0.55 while the decal starts at 0.35,
        // so a scorch got MORE opaque when it started fading. Use the
        // same base opacity as the decal is created with (0.40 since
        // pass 11).
        (s.mesh.material as THREE.MeshBasicMaterial).opacity = 0.4 * fade;
      }
    }
  }
}
