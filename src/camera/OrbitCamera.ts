// OrbitCamera.ts — pass 11 (wreck free orbit), pass 12 (carrier deck
// camera + free-orbit terrain floor).
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
//
// Pass 12 (carrier deck camera): the live v4.6.1 check found the jet
// invisible at a deck spawn in chase AND free orbit — the frame filled
// by carrier structure. Measured against the real SHIELD GLB
// (probe-deck-roll, raycast triangle hits, not bboxes): the culprit is
// detail_0, a thin sheet spanning the stern deck at deckTop + 4.7 m
// (lowest vertex) to + 6.6 m, local x -45..+42, z -166..+28 — the jet
// parks UNDER it (centre deckTop + 2.2), and every 30 m orbit/chase
// sightline from above crosses it (chase blocked at 21.6 m,
// free-orbit yaw 0 blocked at 20.4 m). The island tower (island_6,
// local x 10..32, z -84..-61, deck + 9.4..+38 m) blocks the orbit
// from the north side too. A camera that stays UNDER the sheet — at
// most deckTop + 4.0 m — keeps its sightline to the jet clear of the
// sheet from the chase position and from the default orbit angle.
// That ceiling only applies while the target is on a deck; off the
// deck the orbit is unchanged.

import * as THREE from 'three';

export interface OrbitAngles {
  yaw: number;
  pitch: number;
}

/** Highest a camera may sit above a carrier deck (m above deckTopY)
 *  while the target is on the deck: under the detail_0 overhead sheet
 *  (lowest vertex deckTop + 4.7 m), with 0.7 m of margin. */
export const CARRIER_DECK_CAMERA_CEILING_M = 4.0;
/** Lowest a deck camera may sit (m above deckTopY): above the deck
 *  surface, so the camera never sinks into the deck. */
export const CARRIER_DECK_CAMERA_FLOOR_M = 3.0;

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

/** Cap a camera position under a carrier deck's overhead sheet: when
 *  the target is on a deck (`deckTopY` given), the camera may sit no
 *  higher than deckTop + CARRIER_DECK_CAMERA_CEILING_M, or the
 *  detail_0 sheet (deckTop + 4.7 m at its lowest) blocks the sightline
 *  to the jet from every angle. Off a deck (`deckTopY` null) this is
 *  the identity. Returns the (possibly lowered) position. */
export function applyDeckCeiling(
  pos: THREE.Vector3,
  deckTopY: number | null,
): THREE.Vector3 {
  if (deckTopY === null) return pos;
  const ceiling = deckTopY + CARRIER_DECK_CAMERA_CEILING_M;
  if (pos.y > ceiling) pos.y = ceiling;
  return pos;
}

/** Chase-camera position for a jet on a carrier deck: the normal chase
 *  (behind the jet by `dist`, up `0.34 * dist`), capped so the camera
 *  stays under the deck's overhead sheet and floored so it never
 *  sinks into the deck. The sightline from the capped camera to the
 *  jet (deckTop + 2.2) never rises above deckTop + 4.0, so it passes
 *  under the sheet (lowest vertex deckTop + 4.7) for its whole length.
 *  `deckTopY` is the live carrier deck-top Y; pass null off a deck
 *  and the plain chase position is returned. */
export function deckChaseCameraPosition(
  jetPos: THREE.Vector3,
  fwd: THREE.Vector3,
  up: THREE.Vector3,
  dist: number,
  deckTopY: number | null,
): THREE.Vector3 {
  const want = jetPos
    .clone()
    .addScaledVector(fwd, -dist)
    .addScaledVector(up, dist * 0.34);
  if (deckTopY === null) return want;
  want.y = Math.min(want.y, deckTopY + CARRIER_DECK_CAMERA_CEILING_M);
  want.y = Math.max(want.y, deckTopY + CARRIER_DECK_CAMERA_FLOOR_M);
  return want;
}

/** Chase-camera want position (pass 14, worker D): the single source
 *  of truth for the chase camera's target position — the deck-aware
 *  chase (deckChaseCameraPosition) plus the terrain floor (ground
 *  under the camera +6 m, under the midpoint +8 m, target +2 m) and
 *  the deck ceiling. updateCamera's chase branch converges toward
 *  this position by lerp; snapCameraToChase (main.ts) puts the camera
 *  exactly here on restart/respawn, so the first frame after a spawn
 *  is already the correct chase view instead of a ~1 s lerp from
 *  wherever the camera was (the v4.8 live finding: a side-on,
 *  below-ground view after every Apply & Restart — the lerp path
 *  runs straight through terrain from the old position, and at
 *  1 fps during a restart the lerp takes 4-9 s to converge). */
export function chaseCameraPosition(
  jetPos: THREE.Vector3,
  fwd: THREE.Vector3,
  up: THREE.Vector3,
  dist: number,
  groundHeightAt: (x: number, z: number) => number,
  deckTopY: number | null,
): THREE.Vector3 {
  const want = deckChaseCameraPosition(jetPos, fwd, up, dist, deckTopY);
  const mid = jetPos.clone().addScaledVector(fwd, -dist * 0.5);
  const floor = Math.max(
    groundHeightAt(want.x, want.z) + 6,
    groundHeightAt(mid.x, mid.z) + 8,
    jetPos.y + 2,
  );
  if (want.y < floor) want.y = floor;
  if (deckTopY !== null) applyDeckCeiling(want, deckTopY);
  return want;
}

/** Default free-orbit yaw for a deck spawn: the camera sits ASTERN of
 *  the jet on the deck centreline (carrier yaw + π), looking forward
 *  along the deck. The generic default (yaw 0, world-south) puts the
 *  camera over the port-side deck structures (detail_16, local x
 *  -39..-18, up to deck + 4.6 m) or the starboard island base
 *  (island_23) whenever the carrier is not heading north — at 30 m
 *  the sightline clips those structures from every side angle. The
 *  fore/aft centreline is the only clear corridor (probe-deck-roll
 *  yaw sweep at pitch 0.32, dist 30: 6/16 clear, all within ±0.4 rad
 *  of the carrier axis). */
export function deckOrbitYaw(carrierYaw: number): number {
  return carrierYaw + Math.PI;
}

/** Free-orbit camera position with the terrain floor AND the deck
 *  ceiling: the single entry point for the free-orbit branch. The
 *  camera orbits the TARGET (the jet — never the carrier), stays
 *  above the terrain under itself and under the midpoint to the
 *  target (the pass-8 rule the live inline orbit skips: it clamps
 *  only at the camera's own (x, z)), and stays under the deck sheet
 *  while the target is on a deck (`deckTopY`, else null). The floor
 *  is applied first, then the ceiling — on a deck the terrain floor
 *  sits far below the deck, so they never fight. */
export function orbitCameraPositionDeckAware(
  target: THREE.Vector3,
  angles: OrbitAngles,
  dist: number,
  groundHeightAt: (x: number, z: number) => number,
  deckTopY: number | null,
): THREE.Vector3 {
  const pos = orbitCameraPosition(target, angles, dist);
  applyCameraFloor(pos, target, groundHeightAt);
  applyDeckCeiling(pos, deckTopY);
  return pos;
}
