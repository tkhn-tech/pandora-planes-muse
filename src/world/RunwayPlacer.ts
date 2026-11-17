// RunwayPlacer.ts — runway definitions + strip meshes + terrain flattening.
//
// v2 source: pandora-template.html RUNWAYS (lines ~538-542), runwayBlend
// (v1.1 fix) and the runway mesh block (lines ~595-622). The runway DATA
// lives in PandoraTerrainWorker.ts (plain data, no THREE) so the pure height
// function can use it; this module re-exports it as RUNWAYS and owns the
// meshes. The terrain flattening is the plateau blend inside heightAt()
// (plateauWeight) — each runway sits on a flat-topped plateau at its own
// elevation; runwayBlend survives for the strip colouring / flora
// exclusion and as the flattenAt() query.

import * as THREE from 'three';
import { RUNWAY_TABLE, runwayBlend } from './PandoraTerrainWorker';

export interface Runway {
  name: string;
  x: number;
  z: number;
  elev: number; // v2 computes groundH(x, z) at build; comes out 0 for all three
  hdg: number; // degrees; heading vector = (sin hdg, 0, cos hdg)
  len: number;
  width: number; // v2 `wid`
  dirt: boolean; // v2 `surf === 'dirt'`
  /** v2 `r.dir`: runway heading vector (sin hdg, 0, cos hdg). Set at module
   *  load so it is always available, even before build()/place(). */
  dir: THREE.Vector3;
}

const D2R = Math.PI / 180;

/** Exact v2 values. Alpha is the player spawn runway. */
export const RUNWAYS: Runway[] = RUNWAY_TABLE.map((r) => ({ ...r, dir: runwayDirection(r) }));

/** Runway heading vector, v2 `r.dir`. */
export function runwayDirection(r: { hdg: number }): THREE.Vector3 {
  return new THREE.Vector3(Math.sin(r.hdg * D2R), 0, Math.cos(r.hdg * D2R));
}

export interface RunwayBuildResult {
  /** One group per runway (strip + dashes + edge lights + approach lights). */
  meshes: THREE.Group[];
  /** 0..1 flatten weight at (x, z) — the v1.1 runwayBlend. 1.0 means fully
   *  flat for the whole length + 100 m overrun; use it to prove the strip
   *  is flat (heightAt == 0.0 wherever this returns 1). */
  flattenAt: (x: number, z: number) => number;
}

export class RunwayPlacer {
  private heightAt: ((x: number, z: number) => number) | null = null;
  private groups: THREE.Group[] = [];
  private scene: THREE.Scene | null = null;
  /** Shared cumulative floating-origin shift, when the integrator wires it
   *  (see setOriginShift). Runways are stored in original-frame coords;
   *  positions passed to nearest() are scene coords. */
  private originShift: THREE.Vector3 = new THREE.Vector3();

  /**
   * @param scene optional scene the placer builds into. When given, the
   *   integrator may call place(runways) instead of build(scene, heightAt).
   */
  constructor(scene?: THREE.Scene) {
    this.scene = scene ?? null;
  }

  /** Wire the FloatingOrigin's cumulativeShift so nearest() converts scene
   *  coords to original-frame coords. Pass the live Vector3 object (not a
   *  copy) so it stays current. */
  setOriginShift(shift: THREE.Vector3): void {
    this.originShift = shift;
  }

  /**
   * Build the runway strip meshes.
   * @param scene scene to add the groups to.
   * @param heightAt pure original-frame height function (from
   *   PandoraWorldGenerator / PandoraTerrainWorker). Used to compute each
   *   runway's elevation — v2 does `r.elev = groundH(r.x, r.z)` after
   *   defining groundH; with the v2 height function that evaluates to 0.
   */
  build(scene: THREE.Scene, heightAt: (x: number, z: number) => number): RunwayBuildResult {
    this.heightAt = heightAt;
    this.scene = scene;
    this.groups = this.buildMeshes(scene, RUNWAYS, (r) => {
      r.elev = heightAt(r.x, r.z);
      return r.elev;
    });
    return { meshes: this.groups, flattenAt: (x, z) => runwayBlend(x, z) };
  }

  /**
   * Integrator entry point (main.ts call pattern): build the strip meshes
   * for the given runways into the scene passed to the constructor, using
   * each runway's own elev value (v2 elevations are all 0). Requires the
   * constructor scene; for elevation-from-heightAt use build() instead.
   */
  place(runways: Runway[]): RunwayBuildResult {
    if (!this.scene) throw new Error('RunwayPlacer.place requires a scene in the constructor (or use build(scene, heightAt))');
    this.groups = this.buildMeshes(this.scene, runways, (r) => r.elev);
    return { meshes: this.groups, flattenAt: (x, z) => runwayBlend(x, z) };
  }

  /** Nearest runway to a scene-frame position (main.ts respawn). */
  nearest(p: THREE.Vector3): Runway {
    const wx = p.x + this.originShift.x;
    const wz = p.z + this.originShift.z;
    let best = RUNWAYS[0];
    let bd = Infinity;
    for (const r of RUNWAYS) {
      const d = (wx - r.x) ** 2 + (wz - r.z) ** 2;
      if (d < bd) {
        bd = d;
        best = r;
      }
    }
    return best;
  }

  private buildMeshes(
    scene: THREE.Scene,
    runways: Runway[],
    elevOf: (r: Runway) => number,
  ): THREE.Group[] {
    const groups: THREE.Group[] = [];
    for (const r of runways) {
      const elev = elevOf(r);
      const g = new THREE.Group();
      g.name = `runway-${r.name}`;

      // Strip. v1.1: local +X must map to the heading vector, so the group
      // rotation is (hdg - 90) deg — v1 used -hdg (90 deg off), which put
      // the strip across the takeoff path.
      const strip = new THREE.Mesh(
        new THREE.PlaneGeometry(r.len, r.width),
        new THREE.MeshLambertMaterial({ color: r.dirt ? 0x8a6f4d : 0x2e3136 }),
      );
      strip.rotation.x = -Math.PI / 2;
      strip.position.y = 0.4;
      g.add(strip);

      // Centreline dashes ALONG the runway (local +X), not across it.
      const dashMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
      for (let d = -r.len / 2 + 30; d < r.len / 2 - 10; d += 40) {
        const dash = new THREE.Mesh(new THREE.PlaneGeometry(18, 2.4), dashMat);
        dash.rotation.x = -Math.PI / 2;
        dash.position.set(d, 0.55, 0);
        g.add(dash);
      }

      // Edge lights: green on local -Z, red on local +Z, every 55 m.
      const lightMatG = new THREE.MeshBasicMaterial({ color: 0x7dff8a });
      const lightMatR = new THREE.MeshBasicMaterial({ color: 0xff5a5a });
      for (let s = -1; s <= 1; s += 2) {
        for (let d = -r.len / 2; d <= r.len / 2; d += 55) {
          const l = new THREE.Mesh(new THREE.SphereGeometry(1.6, 6, 6), s < 0 ? lightMatG : lightMatR);
          l.position.set(d, 0.8, s * (r.width / 2 + 3));
          g.add(l);
        }
      }

      // Approach lights: 4 white bars before each threshold.
      const appMat = new THREE.MeshBasicMaterial({ color: 0xfff2b0 });
      for (const end of [-1, 1]) {
        for (let k = 1; k <= 4; k++) {
          const a = new THREE.Mesh(new THREE.PlaneGeometry(2, 14), appMat);
          a.rotation.x = -Math.PI / 2;
          a.position.set(end * (r.len / 2 + k * 28), 0.6, 0);
          g.add(a);
        }
      }

      g.position.set(r.x, elev, r.z);
      g.rotation.y = (r.hdg - 90) * D2R;
      scene.add(g);
      groups.push(g);
    }

    return groups;
  }

  /** 0..1 flatten weight at (x, z) — same function as the build result's
   *  flattenAt. Exposed on the class so the world generator can query it
   *  without keeping the build result around. */
  flattenAt(x: number, z: number): number {
    return runwayBlend(x, z);
  }

  /**
   * Flattened ground height at (x, z) in ORIGINAL-frame coordinates — the
   * height the world generator must use so the terrain is flattened at the
   * runways. This is just heightAt(), which returns the plateau elevation
   * wherever the plateau weight is 1; it is exposed here so the generator
   * has one obvious call site for "ground height with runways flattened".
   * Must be called after build().
   */
  groundHeight(x: number, z: number): number {
    if (!this.heightAt) throw new Error('RunwayPlacer.groundHeight called before build()');
    return this.heightAt(x, z);
  }

  get meshes(): THREE.Group[] {
    return this.groups;
  }
}
