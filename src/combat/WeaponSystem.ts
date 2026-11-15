// WeaponSystem — gun (hitscan + pooled tracers), missiles (proportional
// navigation), flares and gravity bombs. Ported from the v2 template
// (pandora-template.html ~1432–1620), keeping every numeric constant:
//   gun: 1200 m range, 9 m hit radius, 12 dmg, 800 rpm (0.075 s cooldown),
//        tracer drawn at most 400 m (v1.1 fix), muzzle velocity 900 m/s
//   missile: speed 280 m/s, life 12 s, proximity fuse 8 m, 40 dmg,
//        PN max turn 3.2 * speed rad/s, flare break chance 2.2/s
//   bomb: blast radius 70 m, up to 60 dmg, pod damage 40 within 45 m
//   lock: 15 deg cone, < 2500 m, 1.2 s to full lock
// v2 changes: missiles and bombs are Rapier dynamic bodies (gravity scale 0
// for missiles, 1 for bombs) instead of hand-integrated meshes, so they
// collide with the physics world. Proximity fusing is still a distance
// check (aircraft colliders are owned by worker B's FlightPhysicsController).

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import type { Aircraft } from '../physics/FlightPhysicsController';
import { CrashVFXManager } from './CrashVFXManager';

export const GUN_RANGE = 1200;
export const GUN_HIT_RADIUS = 9;
export const GUN_DAMAGE = 12;
export const GUN_COOLDOWN = 0.075; // 800 rpm
export const MUZZLE_VELOCITY = 900; // m/s — also the pipper lead math
export const TRACER_MAX_DRAW = 400; // v1.1: draw at most 400 m of tracer
export const MISSILE_SPEED = 280;
export const MISSILE_LIFE = 12;
export const MISSILE_PROXIMITY = 8;
export const MISSILE_DAMAGE = 40;
export const MISSILE_MAX_TURN = 3.2; // * speed, per second (PN)
export const LOCK_CONE_DEG = 15;
export const LOCK_RANGE = 2500;
export const LOCK_TIME = 1.2;
export const BOMB_BLAST_RADIUS = 70;
export const BOMB_MAX_DAMAGE = 60;
export const BOMB_POD_RADIUS = 45;
export const BOMB_POD_DAMAGE = 40;

export interface WeaponHooks {
  sfx?: (name: 'gun' | 'gunFar' | 'boom' | 'msl' | 'flare' | 'hit') => void;
  /** Lock-tone hook: on = target held in the cone, rate 0..1 to full lock. */
  lockTone?: (on: boolean, rate: number) => void;
  /** Explosion visual/audio — main wires this to CrashVFXManager + sfx. */
  onExplosion?: (pos: THREE.Vector3, scale: number) => void;
  /** Gun/bomb hits on a helicarrier rotor pod (template carrierPodDamage). */
  onCarrierPodHit?: (podIndex: number, damage: number) => void;
  /** Carrier pod world positions, for gun ray + bomb splash checks. */
  carrierPodPositions?: () => THREE.Vector3[];
  groundHeight?: (x: number, z: number) => number;
  deckHeight?: (x: number, z: number) => number;
}

interface Missile {
  mesh: THREE.Mesh;
  body: RAPIER.RigidBody;
  target: Aircraft | null;
  shooter: Aircraft;
  life: number;
}

interface Bomb {
  mesh: THREE.Group;
  body: RAPIER.RigidBody;
  shooter: Aircraft;
  life: number;
}

interface Flare {
  sprite: THREE.Sprite;
  vel: THREE.Vector3;
  life: number;
}

const _v1 = new THREE.Vector3();
const _v2 = new THREE.Vector3();
const _v3 = new THREE.Vector3();

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

export class WeaponSystem {
  private readonly scene: THREE.Scene;
  private readonly world: RAPIER.World;
  private readonly hooks: WeaponHooks;
  private readonly vfx: CrashVFXManager;
  private readonly tracers: THREE.Line[] = [];
  private readonly missiles: Missile[] = [];
  private readonly bombs: Bomb[] = [];
  private readonly flares: Flare[] = [];
  private readonly missileGeo = new THREE.ConeGeometry(0.5, 3.2, 6);
  private readonly missileMat = new THREE.MeshLambertMaterial({ color: 0xd8d8d8 });
  private readonly flareTex: THREE.CanvasTexture;

  constructor(
    scene: THREE.Scene,
    world: RAPIER.World,
    hooks: WeaponHooks = {},
  ) {
    this.scene = scene;
    this.world = world;
    this.hooks = hooks;
    this.vfx = new CrashVFXManager(scene, {
      groundHeight: hooks.groundHeight,
    });
    // Pooled tracer lines (template: 10 lines, shared geometry layout).
    for (let i = 0; i < 10; i++) {
      const g = new THREE.BufferGeometry();
      g.setAttribute(
        'position',
        new THREE.BufferAttribute(new Float32Array(6), 3),
      );
      const line = new THREE.Line(
        g,
        new THREE.LineBasicMaterial({
          color: 0xffd166,
          transparent: true,
          opacity: 0.9,
        }),
      );
      line.visible = false;
      line.userData.life = 0;
      scene.add(line);
      this.tracers.push(line);
    }
    // Flare glow texture (template flareTex canvas gradient).
    const c = document.createElement('canvas');
    c.width = c.height = 64;
    const g2 = c.getContext('2d')!;
    const grd = g2.createRadialGradient(32, 32, 2, 32, 32, 30);
    grd.addColorStop(0, 'rgba(255,255,255,1)');
    grd.addColorStop(0.4, 'rgba(255,170,50,0.9)');
    grd.addColorStop(1, 'rgba(255,120,0,0)');
    g2.fillStyle = grd;
    g2.fillRect(0, 0, 64, 64);
    this.flareTex = new THREE.CanvasTexture(c);
  }

  // ---- gun (hitscan) ------------------------------------------------

  /** Fire one hitscan round at a single target (or dry-fire downrange when
   *  target is null). Caller enforces shooter.gunCd; this method sets it to
   *  GUN_COOLDOWN (0.075 s, 800 rpm). Damage goes through
   *  target.applyDamage(amount, zone, byPlayer) with byPlayer =
   *  shooter.isPlayer. */
  fireGun(shooter: Aircraft, target: Aircraft | null, dt: number): void {
    void dt;
    if (!shooter.alive || shooter.gunCd > 0) return;
    shooter.gunCd = GUN_COOLDOWN;
    const from = shooter.pos
      .clone()
      .addScaledVector(shooter.fwd(), 12);
    const dir = shooter.fwd();
    let end: THREE.Vector3;
    let hit: Aircraft | null = null;
    if (target && target.alive && target !== shooter) {
      const to = _v1.copy(target.pos).sub(from);
      const along = to.dot(dir);
      if (along > 0 && along < GUN_RANGE) {
        const perp = _v2
          .copy(to)
          .addScaledVector(dir, -along)
          .length();
        if (perp < GUN_HIT_RADIUS) {
          hit = target;
          end = from.clone().addScaledVector(dir, along);
        } else {
          end = from.clone().addScaledVector(dir, GUN_RANGE);
        }
      } else {
        end = from.clone().addScaledVector(dir, GUN_RANGE);
      }
    } else {
      end = from.clone().addScaledVector(dir, GUN_RANGE);
    }
    this.showTracer(from, end);
    if (hit) {
      const r = Math.random();
      const zone = r < 0.25 ? 'wing' : r < 0.55 ? 'engine' : 'fuselage';
      hit.applyDamage(GUN_DAMAGE, zone, shooter.isPlayer);
      if (shooter.isPlayer) this.hooks.sfx?.('hit');
    }
    this.checkPodHits(from, dir, shooter);
    this.hooks.sfx?.(shooter.isPlayer ? 'gun' : 'gunFar');
  }

  /** Gun against a list of candidates: nearest centre within 9 m of the
   *  ray, under 1200 m (template hitscanShot). Used by the player (enemies
   *  + traffic) and by AI gun passes (single-target list). */
  fireGunAtTargets(
    shooter: Aircraft,
    targets: Aircraft[],
    dt: number,
  ): void {
    if (!shooter.alive || shooter.gunCd > 0) return;
    const from = shooter.pos.clone().addScaledVector(shooter.fwd(), 12);
    const dir = shooter.fwd();
    let best: Aircraft | null = null;
    let bd = GUN_RANGE;
    for (const t of targets) {
      if (!t.alive || t === shooter) continue;
      const to = _v1.copy(t.pos).sub(from);
      const along = to.dot(dir);
      if (along < 0 || along > GUN_RANGE) continue;
      const perp = _v2.copy(to).addScaledVector(dir, -along).length();
      if (perp < GUN_HIT_RADIUS && along < bd) {
        bd = along;
        best = t;
      }
    }
    // Reuse fireGun's tracer/damage path for the chosen target.
    shooter.gunCd = 0; // fireGun re-checks + sets the cooldown
    this.fireGun(shooter, best, dt);
    if (!best) {
      // fireGun already drew the full-range tracer; pod check ran inside it.
    }
  }

  /** AI gun pass with a skill error cone (template aiGunShot): aim at the
   *  lead point (target pos + vel * range/900), perturb by `err`, fire. */
  aiGunShot(shooter: Aircraft, target: Aircraft, err: number): void {
    if (!shooter.alive || shooter.gunCd > 0 || !target.alive) return;
    const from = shooter.pos.clone().addScaledVector(shooter.fwd(), 12);
    const range = shooter.pos.distanceTo(target.pos);
    const lead = target.pos
      .clone()
      .addScaledVector(target.vel, range / MUZZLE_VELOCITY);
    const dir = lead.sub(from).normalize();
    dir.x += (Math.random() - 0.5) * err;
    dir.y += (Math.random() - 0.5) * err;
    dir.z += (Math.random() - 0.5) * err;
    dir.normalize();
    // Aim the shooter's nose-independent ray: temporarily reuse fireGun by
    // firing at the target with the perturbed direction folded in through a
    // direct hitscan here (fireGun uses shooter.fwd(), so do it inline).
    shooter.gunCd = GUN_COOLDOWN;
    let best: Aircraft | null = null;
    let bd = GUN_RANGE;
    const to = _v1.copy(target.pos).sub(from);
    const along = to.dot(dir);
    if (along > 0 && along < GUN_RANGE) {
      const perp = _v2.copy(to).addScaledVector(dir, -along).length();
      if (perp < GUN_HIT_RADIUS) {
        best = target;
        bd = along;
      }
    }
    const end = from.clone().addScaledVector(dir, best ? bd : GUN_RANGE);
    this.showTracer(from, end);
    if (best) {
      const r = Math.random();
      const zone = r < 0.25 ? 'wing' : r < 0.55 ? 'engine' : 'fuselage';
      best.applyDamage(GUN_DAMAGE, zone, shooter.isPlayer);
      if (shooter.isPlayer) this.hooks.sfx?.('hit');
    }
    this.hooks.sfx?.('gunFar');
  }

  private checkPodHits(
    from: THREE.Vector3,
    dir: THREE.Vector3,
    shooter: Aircraft,
  ): void {
    const pods = this.hooks.carrierPodPositions?.();
    if (!pods) return;
    for (let i = 0; i < pods.length; i++) {
      const pp = pods[i];
      const to = _v3.copy(pp).sub(from);
      const along = to.dot(dir);
      if (along > 0 && along < GUN_RANGE) {
        const perp = to.clone().addScaledVector(dir, -along).length();
        if (perp < 12) {
          this.hooks.onCarrierPodHit?.(i, GUN_DAMAGE);
          this.showTracer(from, pp);
          void shooter;
        }
      }
    }
  }

  private showTracer(a: THREE.Vector3, b: THREE.Vector3): void {
    const t =
      this.tracers.find((x) => !x.visible) ?? this.tracers[0];
    // v1.1: draw at most 400 m of tracer.
    const ab = b.clone().sub(a);
    let end = b;
    if (ab.length() > TRACER_MAX_DRAW) {
      end = a.clone().addScaledVector(ab.normalize(), TRACER_MAX_DRAW);
    }
    const p = t.geometry.attributes.position as THREE.BufferAttribute;
    p.setXYZ(0, a.x, a.y, a.z);
    p.setXYZ(1, end.x, end.y, end.z);
    p.needsUpdate = true;
    t.visible = true;
    t.userData.life = 0.09;
    (t.material as THREE.LineBasicMaterial).opacity = 0.9;
  }

  // ---- lead pipper ---------------------------------------------------

  /** Dynamic lead-target pipper (template updateHud, G6): project the
   *  predicted intercept point — target pos + vel * (range / 900 m/s) —
   *  onto the screen. Returns viewport pixel coords, or null when the
   *  point is behind the camera / there is no target. FlightHUD.showPipper
   *  places the ring at the returned coords. */
  computeLeadPipper(
    shooter: Aircraft,
    target: Aircraft | null,
    camera: THREE.Camera,
  ): { x: number; y: number } | null {
    if (!target || !target.alive) return null;
    const tof = shooter.pos.distanceTo(target.pos) / MUZZLE_VELOCITY;
    const lead = target.pos.clone().addScaledVector(target.vel, tof);
    const v = lead.clone().project(camera);
    if (v.z >= 1) return null;
    return {
      x: (v.x * 0.5 + 0.5) * window.innerWidth,
      y: (-v.y * 0.5 + 0.5) * window.innerHeight,
    };
  }

  // ---- missiles ------------------------------------------------------

  /** Launch a missile at `target`: Rapier dynamic body (gravity scale 0),
   *  initial velocity = shooter vel + fwd * 90, speed 280 m/s, life 12 s.
   *  Proportional-navigation guidance runs in update(). Requires a lock
   *  (15 deg cone, < 2500 m, 1.2 s) — main.ts tracks the lock and only
   *  calls this when lockT > LOCK_TIME. Consumes one shooter.msl. */
  launchMissile(shooter: Aircraft, target: Aircraft): void {
    if (!shooter.alive || shooter.msl <= 0 || !target.alive) return;
    shooter.msl--;
    const start = shooter.pos
      .clone()
      .addScaledVector(shooter.fwd(), 14);
    const mesh = new THREE.Mesh(this.missileGeo, this.missileMat);
    mesh.position.copy(start);
    this.scene.add(mesh);
    const vel = shooter.vel
      .clone()
      .addScaledVector(shooter.fwd(), 90);
    const body = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(start.x, start.y, start.z)
        .setLinvel(vel.x, vel.y, vel.z)
        .setGravityScale(0),
    );
    this.world.createCollider(
      RAPIER.ColliderDesc.ball(0.6).setSensor(true),
      body,
    );
    this.missiles.push({ mesh, body, target, shooter, life: MISSILE_LIFE });
    this.hooks.sfx?.('msl');
  }

  /** Lock-tone hook passthrough (main.ts calls this from its lock scan). */
  setLockTone(on: boolean, rate: number): void {
    this.hooks.lockTone?.(on, clamp(rate, 0, 1));
  }

  // ---- flares --------------------------------------------------------

  /** Decoy flares (template Aircraft.fireFlares): 6 sprites, 1.5 s
   *  cooldown, consumes one flare. While flareCd > 0.4 a missile tracking
   *  this aircraft has a 2.2/s chance per second to lose its lock (see
   *  update()). */
  fireFlares(aircraft: Aircraft): void {
    if (!aircraft.alive || aircraft.flares <= 0 || aircraft.flareCd > 0)
      return;
    aircraft.flares--;
    aircraft.flareCd = 1.5;
    for (let i = 0; i < 6; i++) this.spawnFlare(aircraft.pos, aircraft.vel);
    if (aircraft.isPlayer) this.hooks.sfx?.('flare');
  }

  /**
   * Visuals-only flare burst (main wires Aircraft onFlares here — the
   * AI's Aircraft.fireFlares() already did the bookkeeping, so this must
   * not decrement anything).
   */
  showFlares(aircraft: Aircraft): void {
    for (let i = 0; i < 6; i++) this.spawnFlare(aircraft.pos, aircraft.vel);
  }

  private spawnFlare(pos: THREE.Vector3, vel: THREE.Vector3): void {
    let f = this.flares.find((x) => x.life <= 0);
    if (!f) {
      const sprite = new THREE.Sprite(
        new THREE.SpriteMaterial({
          map: this.flareTex,
          color: 0xffaa33,
          transparent: true,
          depthWrite: false,
        }),
      );
      sprite.scale.set(4, 4, 1);
      this.scene.add(sprite);
      f = { sprite, vel: new THREE.Vector3(), life: 0 };
      this.flares.push(f);
    }
    f.sprite.position.copy(pos);
    f.sprite.visible = true;
    f.life = 3;
    f.vel
      .copy(vel)
      .multiplyScalar(0.3)
      .add(
        new THREE.Vector3(
          (Math.random() - 0.5) * 20,
          -15,
          (Math.random() - 0.5) * 20,
        ),
      );
  }

  // ---- bombs ---------------------------------------------------------

  /** Drop a procedural gravity bomb (template dropBomb): F-18 / F-15E only
   *  (spec.bombs), 4 carried. Rapier dynamic body, initial velocity =
   *  shooter vel + fwd * 8, tumbling. Explodes on ground/deck impact:
   *  blast radius 70 m, up to 60 dmg falloff, pods 40 dmg within 45 m. */
  dropBomb(shooter: Aircraft): void {
    if (!shooter.alive) return;
    if (!shooter.spec.bombs) return; // main.ts toasts the reason
    if (shooter.bombs <= 0) return; // main.ts toasts "BOMBS EMPTY"
    shooter.bombs--;
    const group = new THREE.Group();
    const bmat = new THREE.MeshLambertMaterial({ color: 0x2e3438 });
    const bodyMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.35, 0.35, 2.2, 8),
      bmat,
    );
    bodyMesh.rotation.x = Math.PI / 2;
    group.add(bodyMesh);
    const nose = new THREE.Mesh(
      new THREE.ConeGeometry(0.35, 0.8, 8),
      bmat,
    );
    nose.rotation.x = Math.PI / 2;
    nose.position.z = 1.5;
    group.add(nose);
    for (let i = 0; i < 4; i++) {
      const fin = new THREE.Mesh(
        new THREE.BoxGeometry(0.08, 0.7, 0.7),
        new THREE.MeshLambertMaterial({ color: 0x555555 }),
      );
      fin.position.z = -1.2;
      fin.rotation.z = (i * Math.PI) / 2;
      group.add(fin);
    }
    const start = shooter.pos
      .clone()
      .addScaledVector(shooter.fwd(), 6)
      .addScaledVector(shooter.up(), -2);
    group.position.copy(start);
    this.scene.add(group);
    const vel = shooter.vel.clone().addScaledVector(shooter.fwd(), 8);
    const body = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(start.x, start.y, start.z)
        .setLinvel(vel.x, vel.y, vel.z)
        .setAngvel({ x: 2, y: 0, z: 0 }),
    );
    this.world.createCollider(
      RAPIER.ColliderDesc.ball(0.8).setSensor(true),
      body,
    );
    this.bombs.push({ mesh: group, body, shooter, life: 20 });
    this.hooks.sfx?.('msl');
  }

  // ---- per-frame -----------------------------------------------------

  /** Step missiles, bombs, flares and tracer fades. `targets` is the
   *  splash-damage candidate list for bomb blasts (enemies; the player is
   *  never splash-damaged by their own bomb in the template). */
  update(dt: number, targets: Aircraft[] = []): void {
    // Missiles — proportional navigation toward the predicted intercept.
    for (let i = this.missiles.length - 1; i >= 0; i--) {
      const m = this.missiles[i];
      m.life -= dt;
      let dead = m.life <= 0;
      const pos = m.body.translation();
      const meshPos = _v1.set(pos.x, pos.y, pos.z);
      if (!dead && m.target && m.target.alive) {
        const speed = MISSILE_SPEED;
        const tGo = Math.max(
          0.1,
          meshPos.distanceTo(m.target.pos) / speed,
        );
        const aim = m.target.pos
          .clone()
          .addScaledVector(m.target.vel, tGo * 0.8);
        const want = aim.sub(meshPos).normalize().multiplyScalar(speed);
        const lv = m.body.linvel();
        const cur = _v2.set(lv.x, lv.y, lv.z).normalize().multiplyScalar(speed);
        const steer = want.sub(cur);
        const maxTurn = MISSILE_MAX_TURN * speed * dt;
        if (steer.length() > maxTurn) steer.setLength(maxTurn);
        const next = cur.add(steer);
        m.body.setLinvel({ x: next.x, y: next.y, z: next.z }, true);
        // Flares break the lock (G6).
        if (m.target.flareCd > 0.4 && Math.random() < dt * 2.2) {
          m.target = null;
        }
        if (m.target && meshPos.distanceTo(m.target.pos) < MISSILE_PROXIMITY) {
          m.target.applyDamage(MISSILE_DAMAGE, 'fuselage', m.shooter.isPlayer);
          this.hooks.onExplosion?.(m.target.pos.clone(), 1.4);
          dead = true;
        }
      }
      if (
        !dead &&
        this.hooks.groundHeight &&
        meshPos.y < this.hooks.groundHeight(meshPos.x, meshPos.z)
      ) {
        this.hooks.onExplosion?.(meshPos.clone(), 1);
        dead = true;
      }
      if (dead) {
        this.removeMissile(i);
        continue;
      }
      m.mesh.position.copy(meshPos);
      const lv = m.body.linvel();
      if (lv.x !== 0 || lv.y !== 0 || lv.z !== 0) {
        m.mesh.quaternion.setFromUnitVectors(
          new THREE.Vector3(0, 1, 0),
          _v3.set(lv.x, lv.y, lv.z).normalize(),
        );
      }
    }

    // Bombs — Rapier gravity does the falling; explode on ground/deck.
    for (let i = this.bombs.length - 1; i >= 0; i--) {
      const b = this.bombs[i];
      b.life -= dt;
      const p = b.body.translation();
      b.mesh.position.set(p.x, p.y, p.z);
      const q = b.body.rotation();
      b.mesh.quaternion.set(q.x, q.y, q.z, q.w);
      const gh = this.hooks.groundHeight
        ? this.hooks.groundHeight(p.x, p.z)
        : -Infinity;
      const dy = this.hooks.deckHeight
        ? this.hooks.deckHeight(p.x, p.z)
        : -Infinity;
      if (b.life <= 0 || p.y <= Math.max(gh, dy) + 0.5) {
        const blastPos = new THREE.Vector3(p.x, p.y, p.z);
        this.hooks.onExplosion?.(blastPos, 2);
        for (const e of targets) {
          if (!e.alive) continue;
          const d = e.pos.distanceTo(blastPos);
          if (d < BOMB_BLAST_RADIUS) {
            e.applyDamage(
              BOMB_MAX_DAMAGE * (1 - d / BOMB_BLAST_RADIUS),
              'fuselage',
              b.shooter.isPlayer,
            );
          }
        }
        const pods = this.hooks.carrierPodPositions?.();
        if (pods) {
          for (let k = 0; k < pods.length; k++) {
            if (pods[k].distanceTo(blastPos) < BOMB_POD_RADIUS) {
              this.hooks.onCarrierPodHit?.(k, BOMB_POD_DAMAGE);
            }
          }
        }
        this.removeBomb(i);
      }
    }

    // Flares drift down and fade over 3 s.
    for (const f of this.flares) {
      if (f.life <= 0) {
        f.sprite.visible = false;
        continue;
      }
      f.life -= dt;
      f.sprite.position.addScaledVector(f.vel, dt);
      (f.sprite.material as THREE.SpriteMaterial).opacity = Math.max(
        0,
        f.life / 3,
      );
      if (f.life <= 0) f.sprite.visible = false;
    }

    // Tracers fade over 0.09 s.
    for (const t of this.tracers) {
      if (!t.visible) continue;
      t.userData.life -= dt;
      (t.material as THREE.LineBasicMaterial).opacity =
        Math.max(0, t.userData.life / 0.09) * 0.9;
      if (t.userData.life <= 0) t.visible = false;
    }

    this.vfx.update(dt);
  }

  private removeMissile(i: number): void {
    const m = this.missiles[i];
    this.scene.remove(m.mesh);
    this.world.removeRigidBody(m.body);
    this.missiles.splice(i, 1);
  }

  private removeBomb(i: number): void {
    const b = this.bombs[i];
    this.scene.remove(b.mesh);
    this.world.removeRigidBody(b.body);
    this.bombs.splice(i, 1);
  }

  /** Expose the shared explosion VFX (missiles/bombs call onExplosion,
   *  which main wires to vfx.spawnExplosion; kept here for direct use). */
  get vfxManager(): CrashVFXManager {
    return this.vfx;
  }
}
