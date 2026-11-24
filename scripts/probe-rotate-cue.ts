// probe-rotate-cue.ts — pass 6, item 6: the v4.1 live test reported
// "ROTATE — HOLD ↑ / PULL BACK" staying on screen while airborne at
// 100–250 kt in every flight. The ROTATE cue is shown when
// ac.rotateCue (alive && onSurface && ias >= Vr) — so it should clear
// the moment the jet leaves the surface. This probe drives a real
// takeoff (throttle 100%, full pull at Vr) and checks the cue is ON
// on the runway at/above Vr and OFF once airborne, for the F-22 and
// the F/A-18F. It also checks the takeoff does NOT invert the jet
// (item 10: the live F-22 veered right then pitched into an inverted
// climb, Bank 180°, heading 011° -> 197°).
//
// Usage: node_modules/.bin/esbuild scripts/probe-rotate-cue.ts
//   --bundle --platform=node --format=esm
//   --outfile=/tmp/probe-rotate-cue.mjs
//   && node /tmp/probe-rotate-cue.mjs

if (typeof (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent === 'undefined') {
  (globalThis as unknown as { ProgressEvent?: unknown }).ProgressEvent = class ProgressEvent {
    type: string;
    lengthComputable = false;
    loaded = 0;
    total = 0;
    constructor(
      type: string,
      init?: { lengthComputable?: boolean; loaded?: number; total?: number },
    ) {
      this.type = type;
      this.lengthComputable = init?.lengthComputable ?? false;
      this.loaded = init?.loaded ?? 0;
      this.total = init?.total ?? 0;
    }
  };
}
if (typeof (globalThis as unknown as { self?: unknown }).self === 'undefined') {
  (globalThis as unknown as { self?: unknown }).self = globalThis;
}
if (typeof (globalThis as unknown as { document?: unknown }).document === 'undefined') {
  (globalThis as unknown as { document?: unknown }).document = {
    createElement: (tag: string) => {
      if (tag === 'canvas') {
        return {
          width: 128,
          height: 128,
          getContext: () => ({
            createRadialGradient: () => ({ addColorStop: () => {} }),
            fillRect: () => {},
            set fillStyle(_v: unknown) {},
          }),
        };
      }
      return {};
    },
    createElementNS: (_ns: string, tag: string) => {
      if (tag !== 'img') return {};
      const listeners: Record<string, Array<() => void>> = {};
      const img: Record<string, unknown> = {
        addEventListener: (type: string, fn: () => void) => {
          (listeners[type] ??= []).push(fn);
        },
        removeEventListener: () => {},
      };
      Object.defineProperty(img, 'src', {
        set: () => {
          queueMicrotask(() => {
            for (const fn of listeners['load'] ?? []) fn();
          });
        },
      });
      return img;
    },
  };
}

import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { Aircraft } from '../src/physics/FlightPhysicsController';
import { SPECS } from '../src/aircraft/AircraftSpec';
import { FlightHUD } from '../src/ui/FlightHUD';
import {
  generateHeights,
  heightfieldHeightAt,
  RUNWAY_TABLE,
  WORLD_SIZE,
  TERRAIN_SEGMENTS,
} from '../src/world/PandoraTerrainWorker';

let failures = 0;
function check(name: string, ok: boolean, detail = ''): void {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ' — ' + detail : ''}`);
  if (!ok) failures++;
}

async function main(): Promise<void> {
  await RAPIER.init();
  const heights = generateHeights({
    seed: 1337,
    chunkX: 0,
    chunkZ: 0,
    size: WORLD_SIZE,
    resolution: TERRAIN_SEGMENTS + 1,
  });
  const res = TERRAIN_SEGMENTS + 1;
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);
  const terrainBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, 0),
  );
  const hfHeights = new Float32Array(heights.length);
  for (let iz = 0; iz < res; iz++) {
    for (let ix = 0; ix < res; ix++) {
      hfHeights[ix * res + iz] = heights[iz * res + ix];
    }
  }
  const terrainCollider = world.createCollider(
    RAPIER.ColliderDesc.heightfield(
      TERRAIN_SEGMENTS,
      TERRAIN_SEGMENTS,
      hfHeights,
      new RAPIER.Vector3(WORLD_SIZE, 1, WORLD_SIZE),
    )
      .setFriction(0)
      .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
      .setRestitution(0)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
    terrainBody,
  );
  for (const r of RUNWAY_TABLE) {
    const along = r.len / 2 + 100;
    const across = r.width / 2 + 2;
    const hdg = (r.hdg * Math.PI) / 180;
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(across, 0.5, along)
        .setTranslation(r.x, r.elev - 0.45, r.z)
        .setRotation({
          x: 0,
          y: Math.sin(hdg / 2),
          z: 0,
          w: Math.cos(hdg / 2),
        })
        .setFriction(0)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      terrainBody,
    );
  }
  const scene = new THREE.Scene();
  const wind = new THREE.Vector3(3, 0, 0);
  const heightAt = (x: number, z: number): number =>
    heightfieldHeightAt(heights, res, x, z);

  for (const key of ['f22', 'fa18'] as const) {
    const spec = SPECS[key]!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    ac.terrainHeightAt = heightAt;
    // Charlie runway (the one the live test used): centre, heading 20.
    const rwy = RUNWAY_TABLE.find((r) => r.name === 'Charlie')!;
    ac.resetAt(rwy.x, rwy.elev + 2.2, rwy.z, rwy.hdg);
    // Release the brake, full throttle, roll straight down the runway.
    ac.brake = false;
    ac.thrTarget = 1;
    let cueOnAtVr = false;
    let cueOffAfterLiftoff = false;
    let cueBackOn = false; // pass 12: must NEVER reappear in flight
    let liftedOffAt = -1;
    let maxBank = 0;
    let maxBankAt = '';
    let headingAtLiftoff = 0;
    let headingAtEnd = 0;
    let inverted = false;
    let t = 0;
    for (let i = 0; i < 40 * 60; i++) {
      // Full pull once at/above Vr (no roll input — the item-10
      // question is whether full pull ALONE inverts the jet).
      if (ac.ias >= spec.rotateSpeedKt * 0.514444) {
        ac.pitchIn = -1; // full pull (nose up)
      }
      ac.rollIn = 0;
      ac.yawIn = 0;
      world.timestep = 1 / 60;
      world.step(eventQueue);
      ac.step(1 / 60, wind, null);
      t = i / 60;
      // Bank from the quaternion (Euler YXZ z), as probe-roll does —
      // the Aircraft does not expose a `bank` field.
      const euler = new THREE.Euler().setFromQuaternion(ac.quat, 'YXZ');
      const bankDeg = Math.abs(euler.z * 180 / Math.PI);
      // Heading: respawn sets quat from Euler(0, +hdg, 0, 'YXZ'), so
      // the Euler Y IS the heading (a negated read here reported the
      // spawn heading 20° as 340°, a phantom 40° "veer").
      const headingDeg = ((euler.y * 180 / Math.PI) % 360 + 360) % 360;
      if (bankDeg > maxBank) {
        maxBank = bankDeg;
        maxBankAt = `t=${t.toFixed(1)}s ias=${(ac.ias * 1.944).toFixed(0)}kt`;
      }
      if (bankDeg > 90) inverted = true;
      if (!ac.alive) break;
      if (ac.onSurface && ac.ias >= spec.rotateSpeedKt * 0.514444 && ac.rotateCue) {
        cueOnAtVr = true;
      }
      if (liftedOffAt < 0 && !ac.onSurface && ac.ias > 10) {
        liftedOffAt = t;
        headingAtLiftoff = headingDeg;
      }
      if (liftedOffAt >= 0 && t > liftedOffAt + 1 && !ac.rotateCue) {
        cueOffAfterLiftoff = true;
      }
      // Pass 12: once off after liftoff it must stay off for the
      // rest of the flight (5 s window) — never flicker back on.
      if (liftedOffAt >= 0 && t > liftedOffAt + 1 && ac.rotateCue) {
        cueBackOn = true;
      }
      if (liftedOffAt >= 0 && t > liftedOffAt + 5) {
        headingAtEnd = headingDeg;
        break;
      }
    }
    check(
      `${key}: ROTATE cue is ON on the runway at/above Vr`,
      cueOnAtVr,
      `Vr=${spec.rotateSpeedKt}kt`,
    );
    check(
      `${key}: ROTATE cue is OFF 1 s after liftoff`,
      cueOffAfterLiftoff,
      liftedOffAt >= 0
        ? `liftoff at t=${liftedOffAt.toFixed(1)}s`
        : 'never lifted off',
    );
    check(
      `${key}: ROTATE cue NEVER comes back in the 5 s after liftoff`,
      !cueBackOn,
      liftedOffAt >= 0
        ? `liftoff at t=${liftedOffAt.toFixed(1)}s`
        : 'never lifted off',
    );
    check(
      `${key}: full-pull takeoff does NOT invert the jet`,
      !inverted && maxBank < 60,
      `max bank=${maxBank.toFixed(1)}° at ${maxBankAt}; heading ${headingAtLiftoff.toFixed(0)}° -> ${headingAtEnd.toFixed(0)}°`,
    );
    ac.dispose();
  }

  // Pass 9 (item 7): the v4.3 live test had ROTATE stuck ON while
  // parked, while dead, and while airborne in VTOL hover — states
  // where the getter must be false. The getter is correct and the
  // live stick was the double-shifted height query holding onSurface
  // true while airborne; these checks pin the OFF states so a
  // regression is caught.
  {
    const spec = SPECS['f35a']!;
    const ac = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    ac.terrainHeightAt = heightAt;
    await ac.attachModel();
    const rwy = RUNWAY_TABLE.find((r) => r.name === 'Charlie')!;
    // Parked on the runway, 0 kt: OFF.
    ac.resetAt(rwy.x, rwy.elev + 2.2, rwy.z, rwy.hdg);
    ac.thrTarget = 0;
    ac.brake = true;
    for (let i = 0; i < 120; i++) {
      world.timestep = 1 / 60;
      world.step(eventQueue);
      ac.step(1 / 60, wind, null);
    }
    check(
      'rotate-cue OFF while parked (0 kt on the runway)',
      !ac.rotateCue,
      `ias=${(ac.ias * 1.944).toFixed(0)} kt onSurf=${ac.onSurface}`,
    );
    // Airborne in VTOL hover (vector 90, slow, 100 m up): OFF.
    ac.resetAt(rwy.x, rwy.elev + 100, rwy.z, rwy.hdg);
    ac.vectorDeg = 90;
    ac.thrTarget = 0.94;
    ac.brake = false;
    for (let i = 0; i < 300; i++) {
      world.timestep = 1 / 60;
      world.step(eventQueue);
      ac.step(1 / 60, wind, null);
    }
    check(
      'rotate-cue OFF while airborne in VTOL hover',
      !ac.rotateCue,
      `ias=${(ac.ias * 1.944).toFixed(0)} kt onSurf=${ac.onSurface} vec=${ac.vectorDeg} y=${ac.pos.y.toFixed(1)}`,
    );
    // Dead (crashed): OFF — the getter requires alive.
    ac.crash('terrain');
    check('rotate-cue OFF while dead', !ac.rotateCue, `alive=${ac.alive}`);
    // Fast and airborne (247 kt, the live reading): OFF.
    const ac2 = new Aircraft({
      scene,
      world,
      spec,
      isPlayer: true,
      modelKey: spec.modelKey,
    });
    ac2.terrainHeightAt = heightAt;
    await ac2.attachModel();
    ac2.resetAt(rwy.x, rwy.elev + 150, rwy.z, rwy.hdg);
    ac2.setAirspeed(247 / 1.944);
    ac2.thrTarget = 1;
    for (let i = 0; i < 120; i++) {
      world.timestep = 1 / 60;
      world.step(eventQueue);
      ac2.step(1 / 60, wind, null);
    }
    check(
      'rotate-cue OFF while flying at 247 kt (airborne)',
      !ac2.rotateCue,
      `ias=${(ac2.ias * 1.944).toFixed(0)} kt onSurf=${ac2.onSurface} y=${ac2.pos.y.toFixed(1)}`,
    );
    ac.dispose();
    ac2.dispose();
  }

  // --- FlightHUD DOM test (pass 12, worker D): the ROTATE cue's
  // text must leave the DOM when the cue hides, not just go
  // transparent (opacity 0). The v4.6.1 live check read
  // "ROTATE — HOLD ↑ / PULL BACK" in the DOM while flying, with
  // nothing visible — the static HTML text lingered after the
  // `show` class was removed. FlightHUD now writes the text only
  // while the cue is shown. This drives the real FlightHUD against
  // a stub document and a fake aircraft (only the fields update()
  // reads).
  {
    type StubEl = {
      textContent: string;
      className: string;
      style: Record<string, string>;
      parentElement: null;
      classList: {
        add(c: string): void;
        remove(c: string): void;
        toggle(c: string, force?: boolean): void;
        contains(c: string): boolean;
      };
      querySelector(_sel: string): StubEl;
    };
    const makeStubEl = (): StubEl => {
      const classes = new Set<string>();
      const el: StubEl = {
        textContent: '',
        className: '',
        style: {},
        parentElement: null,
        classList: {
          add: (c) => {
            classes.add(c);
          },
          remove: (c) => {
            classes.delete(c);
          },
          toggle: (c, force) => {
            if (force === undefined) {
              if (classes.has(c)) classes.delete(c);
              else classes.add(c);
            } else if (force) classes.add(c);
            else classes.delete(c);
          },
          contains: (c) => classes.has(c),
        },
        querySelector: (_sel) => makeStubEl(),
      };
      return el;
    };
    const els = new Map<string, StubEl>();
    const docHolder = globalThis as unknown as {
      document?: Record<string, unknown>;
    };
    const docObj = (docHolder.document ?? {}) as Record<string, unknown>;
    docObj['getElementById'] = (id: string): unknown => {
      if (id === 'miniCv') return { getContext: () => null };
      if (!els.has(id)) els.set(id, makeStubEl());
      return els.get(id);
    };
    docHolder.document = docObj;
    const hud = new FlightHUD();
    const rc = els.get('rotateCue')!;
    // Mirror the real DOM: index.html ships #rotateCue with static
    // text ("ROTATE — HOLD ↑ / PULL BACK"). Without this seed the
    // test would pass vacuously — the stub would start empty and
    // staying empty looks identical to being cleared.
    rc.textContent = 'ROTATE — HOLD ↑ / PULL BACK';
    const fakeAc = (over: { alive: boolean; rotateCue: boolean }): Aircraft => {
      const ac = {
        grounded: true,
        vel: new THREE.Vector3(0, 0, 0),
        ias: 0,
        pos: new THREE.Vector3(0, 50, 0),
        vsi: 0,
        fwd: () => new THREE.Vector3(0, 0, 1),
        aoa: 0,
        quat: new THREE.Quaternion(),
        gload: 1,
        fuel: 1,
        alive: over.alive,
        rotateCue: over.rotateCue,
        msl: 6,
        flares: 30,
        bombs: 4,
        damageEntity: {
          maxHpFor: (_z: string) => 100,
          hpFor: (_z: string) => 100,
        },
        fireZone: null,
        dying: false,
        spin: 0,
        spinArrow: 0,
        fireKind: null,
        stalled: false,
        spec: { vtol: false },
        vector: 0,
        lockT: 0,
        lockTarget: null,
      };
      return ac as unknown as Aircraft;
    };
    const game = { score: 0, kills: 0, traps: 0, elapsed: 0, lockTarget: null };
    hud.update(fakeAc({ alive: true, rotateCue: true }), game);
    check(
      'HUD ROTATE shown: class show + text present while on runway at Vr',
      rc.classList.contains('show') && rc.textContent.includes('ROTATE'),
      `text=${JSON.stringify(rc.textContent)}`,
    );
    hud.update(fakeAc({ alive: true, rotateCue: false }), game);
    check(
      'HUD ROTATE hidden in flight: no show class AND text cleared from DOM',
      !rc.classList.contains('show') && rc.textContent === '',
      `text=${JSON.stringify(rc.textContent)}`,
    );
    hud.update(fakeAc({ alive: false, rotateCue: true }), game);
    check(
      'HUD ROTATE hidden while dead: no show class AND text cleared',
      !rc.classList.contains('show') && rc.textContent === '',
      `text=${JSON.stringify(rc.textContent)}`,
    );
    hud.update(fakeAc({ alive: true, rotateCue: true }), game);
    check(
      'HUD ROTATE shows again after hiding: text restored',
      rc.classList.contains('show') && rc.textContent.includes('ROTATE'),
      `text=${JSON.stringify(rc.textContent)}`,
    );
    hud.clearTransient();
    check(
      'HUD clearTransient: ROTATE hidden AND text cleared',
      !rc.classList.contains('show') && rc.textContent === '',
      `text=${JSON.stringify(rc.textContent)}`,
    );

    // Pass 13 (worker E), toast auto-hide: the old timeout only
    // removed the `show` class, so "FORWARD FLIGHT" stayed in the
    // DOM long after the VTOL transition — the same class of bug as
    // the ROTATE cue above. Real 3.2 s timeout, so real waits.
    const sleep = (ms: number): Promise<void> =>
      new Promise((resolve) => setTimeout(resolve, ms));
    const toastEl = els.get('toast')!;
    const toastTitle = els.get('toastTitle')!;
    const toastSub = els.get('toastSub')!;
    hud.toast('FORWARD FLIGHT', 'VTOL off — wings level');
    check(
      'HUD toast shown: class show + title/sub text present',
      toastEl.classList.contains('show') &&
        toastTitle.textContent === 'FORWARD FLIGHT' &&
        toastSub.textContent === 'VTOL off — wings level',
      `title=${JSON.stringify(toastTitle.textContent)} sub=${JSON.stringify(toastSub.textContent)}`,
    );
    await sleep(3500);
    check(
      'HUD toast auto-hide: hidden AND title/sub text cleared from DOM after 3.2 s',
      !toastEl.classList.contains('show') &&
        toastTitle.textContent === '' &&
        toastSub.textContent === '',
      `show=${toastEl.classList.contains('show')} title=${JSON.stringify(toastTitle.textContent)} sub=${JSON.stringify(toastSub.textContent)}`,
    );
    // A re-fired toast must not be wiped by the first toast's timer:
    // toast A, re-fire B at 1.5 s (A's timer is cancelled), then at
    // 3.5 s — past A's original 3.2 s deadline — B must still show.
    hud.toast('VTOL HOVER', 'first');
    await sleep(1500);
    hud.toast('FORWARD FLIGHT', 'second');
    await sleep(2000);
    check(
      'HUD toast re-fire: first toast timer does not hide the second toast early',
      toastEl.classList.contains('show') &&
        toastTitle.textContent === 'FORWARD FLIGHT' &&
        toastSub.textContent === 'second',
      `show=${toastEl.classList.contains('show')} title=${JSON.stringify(toastTitle.textContent)}`,
    );
    await sleep(1500);
    check(
      'HUD toast re-fire: second toast auto-hides AND clears after its own 3.2 s',
      !toastEl.classList.contains('show') &&
        toastTitle.textContent === '' &&
        toastSub.textContent === '',
      `show=${toastEl.classList.contains('show')} title=${JSON.stringify(toastTitle.textContent)}`,
    );

    // Pass 13 (worker E), mirrored compass: the game heading is
    // atan2(fwd.x, fwd.z) with north = −Z, so a right turn DECREASES
    // it — the HUD heading read backwards. Displayed heading must
    // be the TRUE heading (180 − game): game 0 (flying +Z, south) →
    // 180, game 90 (east) → 090, game 180 (north) → 000, game 270
    // (west) → 270, and a right turn (game 90 → 80) must read as an
    // INCREASE (090 → 100). Real FlightHUD, stub document, same
    // harness as the ROTATE checks above.
    const hdgEl = els.get('hudHdg')!;
    const hdgFor = (gameDeg: number): string => {
      const g = (gameDeg * Math.PI) / 180;
      const ac = fakeAc({ alive: true, rotateCue: false });
      // fakeAc's fwd() returns (0,0,1); override per heading.
      (ac as unknown as { fwd: () => THREE.Vector3 }).fwd = () =>
        new THREE.Vector3(Math.sin(g), 0, Math.cos(g));
      hud.update(ac, game);
      return hdgEl.textContent;
    };
    check('HUD compass: game 0 (south) reads 180°', hdgFor(0) === '180°', `got ${hdgFor(0)}`);
    check('HUD compass: game 90 (east) reads 090°', hdgFor(90) === '090°', `got ${hdgFor(90)}`);
    check('HUD compass: game 180 (north) reads 000°', hdgFor(180) === '000°', `got ${hdgFor(180)}`);
    check('HUD compass: game 270 (west) reads 270°', hdgFor(270) === '270°', `got ${hdgFor(270)}`);
    check(
      'HUD compass: right turn (game 90 → 80) reads as an increase (090 → 100)',
      hdgFor(90) === '090°' && hdgFor(80) === '100°',
      `90→${hdgFor(90)} 80→${hdgFor(80)}`,
    );
  }

  console.log(failures === 0 ? '\nALL ROTATE-CUE CHECKS PASSED' : `\n${failures} ROTATE-CUE CHECK(S) FAILED`);
  process.exit(failures === 0 ? 0 : 1);
}

void main();
