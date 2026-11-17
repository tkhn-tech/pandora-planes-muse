// probe-heightfield.mjs — raycast the driver's heightfield along the
// Alpha roll path to check whether the surface is really flat there.
import RAPIER from '@dimforge/rapier3d-compat';
import {
  generateHeights,
  heightAt,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

await RAPIER.init();
const res = TERRAIN_SEGMENTS + 1;
const heights = generateHeights({
  seed: 1337,
  chunkX: 0,
  chunkZ: 0,
  size: WORLD_SIZE,
  resolution: res,
});
const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
const hfHeights = new Float32Array(heights.length);
for (let iz = 0; iz < res; iz++) {
  for (let ix = 0; ix < res; ix++) {
    hfHeights[ix * res + iz] = heights[iz * res + ix];
  }
}
const terrainBody = world.createRigidBody(
  RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
);
world.createCollider(
  RAPIER.ColliderDesc.heightfield(
    TERRAIN_SEGMENTS,
    TERRAIN_SEGMENTS,
    hfHeights,
    new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
  ),
  terrainBody,
);

world.step();
const ray = new RAPIER.Ray({ x: 0, y: 500, z: 0 }, { x: 0, y: -1, z: 0 });
const cast = (x, z) => {
  ray.origin.x = x;
  ray.origin.z = z;
  const hit = world.castRay(ray, 1000, true);
  return hit ? 500 - hit.timeOfImpact : null;
};

console.log('x, z=-900: analytic heightAt vs raycast heightfield');
for (let x = -1420; x <= -1300; x += 20) {
  console.log(
    `x=${x} analytic=${heightAt(x, -900).toFixed(3)} raycast=${(cast(x, -900) ?? NaN).toFixed(3)}`,
  );
}
console.log('gear-post paths (every 2 m along the roll):');
for (const z of [-901.6, -898.4, -900]) {
  let worst = 0; let worstX = 0;
  for (let x = -1410; x <= -1360; x += 2) {
    const h = cast(x, z) ?? -999;
    if (Math.abs(h) > Math.abs(worst)) { worst = h; worstX = x; }
  }
  console.log(`z=${z}: worst raycast height ${worst.toFixed(3)} at x=${worstX}`);
}
console.log('impulse point and gear posts:');
for (const [x, z] of [[-1379.5, -900], [-1382.4, -901.6], [-1382.4, -898.4], [-1372.7, -900]]) {
  console.log(`x=${x} z=${z} analytic=${heightAt(x, z).toFixed(3)} raycast=${(cast(x, z) ?? NaN).toFixed(3)}`);
}
