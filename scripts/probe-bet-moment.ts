// probe-bet-moment.ts — pass 7 diagnostic: call the BET solver directly
// at fixed states and print the pitch/roll/yaw rates it returns, to find
// which term drives the hands-off pitch-up. States: v = 60..220 m/s,
// AoA = 0..5 deg (level pitch, flight path = -AoA), throttle 0 and 1,
// pitchIn/rollIn/yawIn = 0. Prints aoaDeg, rates (deg/s) and gLoad.
//
// Usage: node_modules/.bin/esbuild scripts/probe-bet-moment.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-bet-moment.mjs
//   && node /tmp/probe-bet-moment.mjs

import * as THREE from 'three';
import { BladeElementSolver } from '../src/physics/BladeElementSolver';
import { SPECS } from '../src/aircraft/AircraftSpec';

function main(): void {
  for (const key of ['fa18', 'f22'] as const) {
    const spec = SPECS[key]!;
    const solver = new BladeElementSolver(spec);
    console.log(`--- ${key} (mass=${spec.mass} thrust=${spec.thrustMax} span=${spec.span} S=${spec.wingArea} L=${spec.length}) ---`);
    for (const thr of [0, 1]) {
      for (const v of [60, 100, 150, 200]) {
        for (const aoa of [0, 2, 5]) {
          // Level pitch, flight path descending by AoA: velocity in body
          // frame has vBody.y = -v*sin(AoA), vBody.z = v*cos(AoA).
          const aoaR = (aoa * Math.PI) / 180;
          const quat = new THREE.Quaternion(); // level, nose +Z
          const vBody = new THREE.Vector3(0, -v * Math.sin(aoaR), v * Math.cos(aoaR));
          const vAir = vBody.clone(); // quat is identity
          solver.reset();
          const r = solver.solve({
            velocityAir: vAir,
            quat,
            mass: spec.mass,
            wingArea: spec.wingArea,
            wingAreaLift: spec.wingAreaEffective ?? spec.wingArea,
            span: spec.span,
            chord: spec.wingArea / spec.span,
            length: spec.length,
            throttle: thr,
            thrustMax: spec.thrustMax,
            vectorDeg: 0,
            flapsDeg: 0,
            airbrake: 0,
            gearDown: false,
            damage: {
              wingL: 0, wingR: 0, tail: 0, engineL: 0, engineR: 0, fuel: 0, fin: 0,
            },
            altitude: 500,
            density: 1.225,
            archetype: spec.archetype,
            controls: { pitch: 0, roll: 0, yaw: 0, throttle: null },
            heightAboveGround: 500,
            controlAuthority: { turn: spec.turn, rollDeg: spec.rollDeg, pitch: spec.pitch },
            failures: {},
            inducedK: spec.inducedK,
            clCurve: spec.clCurve,
            angVel: new THREE.Vector3(),
            tvcDeg: spec.tvcDeg,
            rollCapSchedule: spec.rollCapSchedule,
            liftScale: 1,
            dying: false,
            dt: 1 / 60,
          });
          console.log(
            `  thr=${thr} v=${v}m/s aoa=${aoa}deg -> aoaDeg=${r.aoaDeg.toFixed(2)} pitchRate=${((r.rates?.pitchRate ?? 0) * 57.3).toFixed(2)}deg/s rollRate=${((r.rates?.rollRate ?? 0) * 57.3).toFixed(2)}deg/s yawRate=${((r.rates?.yawRate ?? 0) * 57.3).toFixed(2)}deg/s g=${r.gLoad.toFixed(2)}`,
          );
        }
      }
    }
  }
  console.log('DONE');
}

main();
