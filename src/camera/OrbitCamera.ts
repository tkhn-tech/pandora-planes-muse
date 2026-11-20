// OrbitCamera.ts — pass 11 (wreck free orbit).
//
// Pure helpers for the free-orbit camera (camMode 2), shared by the
// normal free-orbit view and the wreck-spectate view. Kept pure so a
// probe can drive them without the game: the camera orbits a target,
// zooms by distance, and stays focused on it.
//
// The wreck view uses the same math as the normal free orbit — the
// only difference is the target: the wreck's position (p.pos) instead
// of the live jet. The terrain floor + sightline rule is the pass-8
// one (the live "flat voids" view, where a ridge between the camera
// and the wreck hid the wreck): the camera clears the ground under
// itself AND under the midpoint to the target, and never drops below
// the target + 2 m.

import * as THREE from 'three';

export interface OrbitAngles {
  yaw: number;
  pitch: number;
}

/** Camera position for a free orbit around `target` at the given
 *  angles and distance. Yaw 0 = south of the target (+Z), positive
 *  yaw swings the camera toward +X; pitch is the elevation angle
 *  above the target (clamped by the caller to -0.1…1.2). */
export function orbitCameraPosition(
  target: THREE.Vector3,
  angles: OrbitAngles,
  dist: number,
): THREE.Vector3 {
  return new THREE.Vector3(
    target.x + Math.sin(angles.yaw) * Math.cos(angles.pitch) * dist,
    target.y + Math.sin(angles.pitch) * dist,
    target.z + Math.cos(angles.yaw) * Math.cos(angles.pitch) * dist,
  );
}

/** Raise a camera position so the target stays visible: clear the
 *  ground under the camera (+3 m) and under the midpoint to the
 *  target (+5 m), and never drop below target + 2 m. Returns the
 *  (possibly raised) position. */
export function applyCameraFloor(
  pos: THREE.Vector3,
  target: THREE.Vector3,
  groundHeightAt: (x: number, z: number) => number,
): THREE.Vector3 {
  const midX = (pos.x + target.x) / 2;
  const midZ = (pos.z + target.z) / 2;
  const floor = Math.max(
    groundHeightAt(pos.x, pos.z) + 3,
    groundHeightAt(midX, midZ) + 5,
    target.y + 2,
  );
  if (pos.y < floor) pos.y = floor;
  return pos;
}
