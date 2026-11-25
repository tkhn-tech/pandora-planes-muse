// CanopyMaterials.ts — v4.11.
//
// The one surviving piece of the old CockpitBuilder.ts (deleted
// 2026-11-25, v4.11 cockpit removal — Nicholas: "remove the ugly
// extra additions on the nose. remove cockpit view").
//
// fixCanopyMaterials() is NOT a cockpit addition: it touches the
// GLB's OWN canopy meshes (the F-22/F-15E canopies use a "Glass"
// material, metalness 1, roughness 1, opaque — with no scene
// environment map a metal renders BLACK, so the canopies read as
// black shells from outside). It makes them transparent glass
// (opacity 0.28, no metal). Called after attachModel for the
// player, enemies and traffic — removing it would regress the
// external look, so it stays, under its own name.

import * as THREE from 'three';

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
