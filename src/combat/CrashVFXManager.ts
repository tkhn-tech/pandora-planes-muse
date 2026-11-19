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
}

interface DebrisPiece {
  mesh: THREE.Object3D;
  body: RAPIER.RigidBody;
  life: number;
  // Pass 8 (item 5): age since spawn. Debris is spawned AT the
  // surface-clamped crash position with an upward velocity (5-15
  // m/s) — in the first 0.05 s step a slow piece rises only 0.25 m,
  // so the old `p.y < gh + 0.5` ground-removal test fired on frame
  // one and the debris vanished before it was ever seen (the live
  // "no debris" finding). Ground removal only applies once the piece
  // has had time to rise AND is actually falling back.
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

/** Max ground scorch decals kept (oldest removed first — see
 *  scorchDecal). */
const MAX_SCORCH = 12;
/** Scorch decal lifetime (s): fades out over the last 30 s. */
const SCORCH_LIFE = 90;

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
  }

  /** Pooled explosion sprite burst + light flash + camera-shake hook.
   *  Template timing: scale = s * (6 + t*55), opacity 1 - t/0.9, life 0.9 s. */
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
   *  or 0xff7722 fire. Returns the group so the caller can remove it. */
  spawnSmoke(
    pos: THREE.Vector3,
    colour: number,
    attachTo?: THREE.Object3D,
  ): THREE.Group {
    const group = new THREE.Group();
    for (let i = 0; i < 5; i++) {
      const s = new THREE.Sprite(
        new THREE.SpriteMaterial({
          map: this.smokeTex,
          color: colour,
          transparent: true,
          opacity: 0.55,
          depthWrite: false,
        }),
      );
      s.scale.set(6 + i * 2, 6 + i * 2, 1);
      s.position.set((Math.random() - 0.5) * 2, -i * 1.2, -4 - i * 1.5);
      group.add(s);
    }
    if (attachTo) {
      // Attached: parent to the moving object at its origin (the
      // jet's centre) — the fire is ON the jet and follows it.
      group.position.set(0, 0, 0);
      attachTo.add(group);
      this.plumes.push({ group, life: 6, maxLife: 6, parent: attachTo });
    } else {
      group.position.copy(pos);
      this.scene.add(group);
      // Free plume: register with the floating origin so a shift
      // moves it with the world (it is a scene object, not a child
      // of a jet). Attached plumes move with their parent and are
      // not registered.
      this.opts.origin?.registerObject(group);
      this.plumes.push({ group, life: 6, maxLife: 6, parent: null });
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
   *  supplied. Life 10 s, removed on ground contact or expiry. */
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
      const box = new THREE.Box3().setFromObject(mesh);
      const size = new THREE.Vector3();
      box.getSize(size);
      half = size.multiplyScalar(0.5);
      half.x = Math.max(half.x, 0.25);
      half.y = Math.max(half.y, 0.25);
      half.z = Math.max(half.z, 0.25);
      // Re-centre the mesh on its own origin so the body tracks it cleanly.
      const centre = new THREE.Vector3();
      box.getCenter(centre);
      mesh.position.sub(centre);
      const holder = new THREE.Group();
      holder.add(mesh);
      mesh = holder;
    } else {
      mesh = new THREE.Mesh(
        new THREE.BoxGeometry(4, 0.5, 2.5),
        new THREE.MeshLambertMaterial({ color: 0x8a97a3 }),
      );
      half = new THREE.Vector3(2, 0.25, 1.25);
    }
    mesh.position.copy(pos);
    this.scene.add(mesh);

    this.debrisWorld = world;
    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(pos.x, pos.y, pos.z)
        .setLinvel(vel.x, vel.y, vel.z)
        .setAngvel({
          x: (Math.random() - 0.5) * 4,
          y: (Math.random() - 0.5) * 4,
          z: (Math.random() - 0.5) * 4,
        }),
    );
    this.opts.origin?.registerBody(body);
    world.createCollider(RAPIER.ColliderDesc.cuboid(half.x, half.y, half.z), body);
    this.debris.push({ mesh, body, life: 10, age: 0 });
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
        // Pass 5 (item 2): was 0x0a0a0a at 0.55 — a pure-black disc
        // on the ground, which read as one of the "black discs" (a
        // 430+ px black ellipse straddling a runway in the v4
        // play-test, left by the sink crashes). A scorch is a darkened
        // patch, not a black hole: dark brown-grey, less opaque.
        // Pass 6: 0.45 was still reading black from a chase cam, so
        // 0.35 now, and the radius is smaller (see above).
        color: 0x2a241e,
        transparent: true,
        opacity: 0.35,
        depthWrite: false,
      }),
    );
    decal.rotation.x = -Math.PI / 2;
    decal.position.set(pos.x, gh + 0.15, pos.z);
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
    // Explosion sprites: grow + fade over 0.9 s (template updateBooms).
    for (const b of this.booms) {
      if (!b.sprite.visible) continue;
      b.t += dt;
      const s = b.scale * (6 + b.t * 55);
      b.sprite.scale.set(s, s, 1);
      b.sprite.material.opacity = Math.max(0, 1 - b.t / 0.9);
      if (b.t > 0.9) b.sprite.visible = false;
    }
    // Light flash decays fast.
    if (this.flash.intensity > 0) {
      this.flash.intensity = Math.max(0, this.flash.intensity - dt * 220);
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
      const fade = Math.max(0, p.life / p.maxLife);
      for (const child of p.group.children) {
        const spr = child as THREE.Sprite;
        spr.material.opacity = 0.55 * fade;
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
    // Debris: sync mesh from its Rapier body; remove on ground/expiry.
    // Pass 8 (item 5): a piece is only "on the ground" once it has
    // risen and is falling back (age > 0.5 s AND vertical velocity
    // below zero) — a fresh piece at the surface with an upward
    // velocity is still in its launch arc, not resting on the ground.
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
      const fallingBack = d.age > 0.5 && d.body.linvel().y < 0;
      if (d.life <= 0 || (fallingBack && p.y < gh + 0.5)) {
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
        // same base opacity as the decal is created with.
        (s.mesh.material as THREE.MeshBasicMaterial).opacity = 0.35 * fade;
      }
    }
  }
}
