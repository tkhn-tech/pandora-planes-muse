// isolate-launch.mjs — minimal Rapier reproduction of the takeoff-roll
// launch impulse. Flat heightfield + one dynamic body with the F-15E's
// exact 4-collider layout, driven by a controller-like setLinvel loop.
// Usage: MODE=<mode> node isolate-launch.mjs
//   modes: full | noSetLinvel | noPosts | singleSled | noThrust
// Prints the first step where |vy| jumps above 0.5 m/s or speed drops.
import RAPIER from '@dimforge/rapier3d-compat';
import {
  generateHeights,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

await RAPIER.init();
const MODE = process.env['MODE'] ?? 'full';
const res = TERRAIN_SEGMENTS + 1;
const heights =
  process.env['MODE'] === 'zerohf'
    ? new Float32Array(res * res)
    : generateHeights({
        seed: 1337,
        chunkX: 0,
        chunkZ: 0,
        size: WORLD_SIZE,
        resolution: res,
      });
const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
if (process.env['MODE'] === 'iters8') world.integrationParameters.numSolverIterations = 8;
const hfHeights = new Float32Array(heights.length);
for (let iz = 0; iz < res; iz++) {
  for (let ix = 0; ix < res; ix++) {
    hfHeights[ix * res + iz] = heights[iz * res + ix];
  }
}
const terrainBody = world.createRigidBody(
  RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
);
const terrainCollider =
  MODE === 'flatbox'
    ? world.createCollider(
        RAPIER.ColliderDesc.cuboid(3000, 0.5, 3000)
          .setTranslation(0, -0.5, 0)
          .setFriction(0)
          .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min),
        terrainBody,
      )
    : world.createCollider(
        RAPIER.ColliderDesc.heightfield(
          TERRAIN_SEGMENTS,
          TERRAIN_SEGMENTS,
          hfHeights,
          new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
        ).setFriction(0)
          .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min),
        terrainBody,
      );

// F-15E: mass 20400, length 19.41, span 13.05. Colliders mirror
// FlightPhysicsController: fuselage (hy 1.4, centre +0.6, hz 8.73) +
// 3 gear posts (feet at -2.2). Density chosen so Rapier mass == spec.
const mass = 20400;
const mkBody = () => {
  const desc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(-1400, 2.2, -900)
    .setRotation({ x: 0, y: 0.70710678, z: 0, w: 0.70710678 }) // yaw 90 deg
    .setLinearDamping(0)
    .setAngularDamping(0.2)
    .setGravityScale(0)
    .setCcdEnabled(false)
    .setCanSleep(false);
  const body = world.createRigidBody(desc);
  const add = (hx, hy, hz, x, y, z, density) => {
    const c = RAPIER.ColliderDesc.cuboid(hx, hy, hz)
      .setTranslation(x, y, z)
      .setDensity(density)
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min);
    world.createCollider(c, body);
  };
  if (MODE === 'singleSled') {
    // One box spanning the gear footprint, bottom at -2.2.
    const vol = 3.2 * 0.8 * 19.41;
    add(1.6, 0.4, 9.7, 0, -1.8, 0, mass / vol);
  } else {
    const fusVol = 2 * 1.4 * 2 * 8.73 * 2; // 2*hx * 2*hy * 2*hz
    const postVol = 0.7 * 0.8 * 0.7;
    const totalVol = fusVol + 3 * postVol;
    const density = mass / totalVol;
    add(1.4, 1.4, 8.73, 0, 0.6, 0, density); // fuselage
    if (MODE !== 'noPosts') {
      if (MODE === 'ballfeet') {
        const addBall = (x: number, y: number, z: number) => {
          const c = RAPIER.ColliderDesc.ball(0.35)
            .setTranslation(x, y, z)
            .setDensity(density)
            .setFriction(0)
            .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min);
          world.createCollider(c, body);
        };
        addBall(-1.6, -1.85, -2.91);
        addBall(1.6, -1.85, -2.91);
        addBall(0, -1.85, 6.79);
      } else if (MODE === 'ski') {
        add(0.35, 0.4, 2.0, -1.6, -1.8, -2.91, density);
        add(0.35, 0.4, 2.0, 1.6, -1.8, -2.91, density);
        add(0.35, 0.4, 2.0, 0, -1.8, 6.79, density);
      } else {
        add(0.35, 0.4, 0.35, -1.6, -1.8, -2.91, density);
        add(0.35, 0.4, 0.35, 1.6, -1.8, -2.91, density);
        add(0.35, 0.4, 0.35, 0, -1.8, 6.79, density);
      }
    }
  }
  return body;
};

const body = mkBody();
console.log(`mode=${MODE} Rapier mass=${body.mass().toFixed(0)} kg (spec 20400)`);
console.log(`terrain collider handle=${terrainCollider.handle} body handle=${body.handle}`);

const dt = 1 / 60;
const G = 9.81;
let throttle = 0;
let prevVy = 0;
let prevVx = 0;
let launched = false;
for (let i = 0; i < 600; i++) {
  const t = i * dt;
  world.timestep = dt;
  world.step();
  const v = body.linvel();
  const p = body.translation();
  // Controller-like: thrust accel along +x (heading east), gravity,
  // ground friction 0.3 m/s^2, setLinvel + setAngvel(0) every step.
  throttle = Math.min(1, throttle + 0.55 * dt);
  const thrustAcc = MODE === 'noThrust' ? 0 : (258000 * throttle) / mass;
  let vx = v.x + thrustAcc * dt;
  let vy = v.y - G * dt;
  const sp = Math.hypot(vx, v.z);
  if (sp > 0.5) {
    const k = Math.max(0, 1 - (0.3 * dt) / sp);
    vx *= k;
  }
  if (MODE !== 'noSetLinvel') {
    if (MODE === 'clampVy' && p.y <= 2.21) vy = Math.max(0, vy);
    body.setLinvel({ x: vx, y: vy, z: v.z }, true);
    body.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }
  const dvx = v.x - prevVx;
  const dvy = v.y - prevVy;
  if (i > 5 && (Math.abs(v.y) > 0.5 || Math.abs(dvx) > 0.5 || Math.abs(dvy) > 0.5)) {
    console.log(
      `LAUNCH? step=${i} t=${t.toFixed(3)} pos=(${p.x.toFixed(2)},${p.y.toFixed(3)},${p.z.toFixed(2)}) ` +
        `vel=(${v.x.toFixed(2)},${v.y.toFixed(2)},${v.z.toFixed(2)}) ` +
        `dVx=${dvx.toFixed(2)} dVy=${dvy.toFixed(2)} thr=${throttle.toFixed(2)}`,
    );
    launched = true;
    break;
  }
  prevVx = v.x;
  prevVy = v.y;
  if (MODE === 'full' && p.x > -1030 && p.x < -1000) {
    console.log(
      `  step=${i} t=${t.toFixed(3)} y=${p.y.toFixed(4)} vy=${v.y.toFixed(3)} vx=${v.x.toFixed(2)}`,
    );
  }
  if (i % 60 === 0) {
    console.log(
      `t=${t.toFixed(2)} x=${(p.x + 1400).toFixed(1)} y=${p.y.toFixed(3)} ` +
        `vx=${v.x.toFixed(2)} vy=${v.y.toFixed(3)} thr=${throttle.toFixed(2)}`,
    );
  }
}
if (!launched) console.log('no launch in 10 s');
