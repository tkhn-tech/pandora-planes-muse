// Probe: call the BladeElementSolver directly for the F-35A hover state.
import * as THREE from 'three';
import { BladeElementSolver } from '../src/physics/BladeElementSolver';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { ZERO_DAMAGE } from '../src/physics/IAerodynamicsSolver';

const spec = SPECS.f35a;
const solver = new BladeElementSolver(spec);
console.log('stations:', solver.stationCount);
const quat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, (90 * Math.PI) / 180, 0, 'YXZ'));
const res = solver.solve({
  velocityAir: new THREE.Vector3(0, 0, 0),
  quat,
  mass: spec.mass,
  wingArea: spec.wingArea,
  wingAreaLift: spec.wingAreaEffective ?? spec.wingArea,
  span: spec.span,
  chord: spec.wingArea / spec.span,
  length: spec.length,
  throttle: 1,
  thrustMax: spec.thrustMax,
  vectorDeg: 90,
  flapsDeg: 0,
  airbrake: 0,
  gearDown: true,
  damage: { ...ZERO_DAMAGE },
  altitude: 10,
  density: 1.225,
  archetype: spec.archetype,
  controls: { pitchIn: 0, rollIn: 0, yawIn: 0 },
  heightAboveGround: 10,
  controlAuthority: { turn: spec.turn, rollDeg: spec.rollDeg, pitch: spec.pitch },
  failures: {},
  inducedK: spec.inducedK,
  clCurve: spec.clCurve,
  angVel: new THREE.Vector3(),
  tvcDeg: spec.tvcDeg,
  rollCapSchedule: spec.rollCapSchedule,
  dt: 1 / 60,
});
console.log('thrust:', res.thrust.toArray().map((v) => v.toFixed(0)));
console.log('lift:', res.lift.toArray().map((v) => v.toFixed(0)));
console.log('drag:', res.drag.toArray().map((v) => v.toFixed(0)));
console.log('rates:', res.rates);
console.log('aoa:', res.aoaDeg.toFixed(1), 'cl:', res.cl.toFixed(3), 'g:', res.gLoad.toFixed(2));
console.log('weight N:', (spec.mass * 9.81).toFixed(0));
