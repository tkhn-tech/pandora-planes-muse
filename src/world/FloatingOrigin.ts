// FloatingOrigin.ts — G4 floating origin (v2 template lines ~717-748).
//
// Past ~2 km from the origin, the world is shifted back toward the origin so
// positions stay small and float jitter never builds up at this 6 km scale.
// v2 shifts EVERYTHING by the same vector — terrain, islands, runways,
// carrier, AI, FX — and the player's own position too, so there is no
// visible jump: relative geometry is unchanged, only the absolute frame
// moves. The cumulative shift is kept (v2 `originShift`) so scene coords can
// be converted back to original-frame coords for groundH / minimap / nav.
//
// Usage (main.ts, every frame):
//   const shift = origin.shiftIfNeeded(player.pos);
//   if (shift) origin.applyShift(shift);
// Register everything that lives in the world: THREE objects via
// registerObject, Rapier bodies via registerBody, and bare logical positions
// (player.pos, AI navTargets, carrier waypoints) via registerVector — v2
// shifts those too. The world generator registers itself with
// registerWithFloatingOrigin() so its groundHeight() stays correct.

import * as THREE from 'three';
import type { RigidBody } from '@dimforge/rapier3d-compat';

export class FloatingOrigin {
  /** v2 threshold: shift once |player xz| passes 2000 m. */
  readonly threshold: number;
  /** Cumulative world shift (v2 `originShift`). Scene coords + this shift =
   *  original-frame coords. */
  readonly cumulativeShift = new THREE.Vector3();

  private readonly objects: THREE.Object3D[] = [];
  private readonly bodies: RigidBody[] = [];
  private readonly vectors: THREE.Vector3[] = [];
  private readonly callbacks: Array<(shift: THREE.Vector3) => void> = [];

  constructor(threshold = 2000) {
    this.threshold = threshold;
  }

  registerObject(obj: THREE.Object3D): void {
    this.objects.push(obj);
  }

  registerBody(body: RigidBody): void {
    this.bodies.push(body);
  }

  /** Register a bare logical position (e.g. player.pos, an AI navTarget, a
   *  carrier waypoint) so it is shifted with the world. */
  registerVector(v: THREE.Vector3): void {
    this.vectors.push(v);
  }

  unregisterBody(body: RigidBody): void {
    const i = this.bodies.indexOf(body);
    if (i >= 0) this.bodies.splice(i, 1);
  }

  unregisterObject(obj: THREE.Object3D): void {
    const i = this.objects.indexOf(obj);
    if (i >= 0) this.objects.splice(i, 1);
  }

  unregisterVector(v: THREE.Vector3): void {
    const i = this.vectors.indexOf(v);
    if (i >= 0) this.vectors.splice(i, 1);
  }

  /** Called with the shift vector after every applyShift (e.g. so the world
   *  generator can update its groundHeight frame). */
  registerCallback(cb: (shift: THREE.Vector3) => void): void {
    this.callbacks.push(cb);
  }

  /** v2 shift rule: the shift is the player's FULL xz offset, bringing the
   *  player back to (0, z=0) in the new frame. Returns null when no shift
   *  is needed. This method is pure detection — it does not move anything;
   *  call applyShift() with the returned vector. */
  shiftIfNeeded(playerPos: THREE.Vector3): THREE.Vector3 | null {
    if (Math.hypot(playerPos.x, playerPos.z) <= this.threshold) return null;
    return new THREE.Vector3(playerPos.x, 0, playerPos.z);
  }

  /** Move the world by -shift: every registered THREE object and every
   *  registered Rapier body (via setTranslation, woken) plus every
   *  registered logical vector. The player is NOT special-cased — its group
   *  / body / pos vector must be registered like everything else, exactly
   *  as v2 shifts player.pos along with the world. After this call the
   *  player's scene position is near the origin and cumulativeShift has
   *  grown by `shift`. */
  applyShift(shift: THREE.Vector3): void {
    if (shift.x === 0 && shift.z === 0) return;
    for (const obj of this.objects) {
      obj.position.x -= shift.x;
      obj.position.z -= shift.z;
    }
    for (const body of this.bodies) {
      const t = body.translation();
      body.setTranslation({ x: t.x - shift.x, y: t.y, z: t.z - shift.z }, true);
    }
    for (const v of this.vectors) {
      v.x -= shift.x;
      v.z -= shift.z;
    }
    this.cumulativeShift.x += shift.x;
    this.cumulativeShift.z += shift.z;
    for (const cb of this.callbacks) cb(shift);
  }

  /** Shift everything back to the original frame (v2 resetOrigin). */
  reset(): void {
    if (this.cumulativeShift.x === 0 && this.cumulativeShift.z === 0) return;
    this.applyShift(new THREE.Vector3(-this.cumulativeShift.x, 0, -this.cumulativeShift.z));
  }
}
