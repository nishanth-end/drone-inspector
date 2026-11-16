// Browser version of demo/dashboard.py: same missions, planner, camera model and packets.
// Runs entirely in the page, so it works on a static host (no server needed).
const ALT = 6.0, CLIMB = 2.0, LAND = 1.0, CRUISE = 4.0, ACCEL = 2.0, TURN = 60.0, TURN_COST = 0.15;
const CAM = 35 * Math.PI / 180;

const OBJECTS = [
  {n: 12, e: -16, label: 'crack', cat: 'damage'},
  {n: -6, e: 12, label: 'spalling', cat: 'damage'},
  {n: 16, e: 8, label: 'corrosion', cat: 'damage'},
  {n: -14, e: -8, label: 'leak', cat: 'damage'},
  {n: -2, e: -18, label: 'crack', cat: 'damage'},
  {n: 18, e: -6, label: 'spalling', cat: 'damage'},
  {n: 4, e: -4, label: 'person', cat: 'emergency'},
  {n: -18, e: 16, label: 'person', cat: 'emergency'},
  {n: -8, e: -14, label: 'person', cat: 'emergency'},
  {n: 14, e: -2, label: 'person', cat: 'emergency'},
  {n: -12, e: 4, label: 'person', cat: 'emergency'},
];

const mod = (x, m) => ((x % m) + m) % m;
const r = (x, d) => Math.round(x * 10 ** d) / 10 ** d;
const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
const conf = i => r(0.82 + ((i * 37) % 15) / 100, 2);

// Same as area_patrol.lawnmower_waypoints (north-south lanes, 8 m apart)
function lawnmower(nMin, nMax, eMin, eMax, spacing) {
  const lanes = [];
  let e = eMin;
  while (e < eMax) { lanes.push(e); e += spacing; }
  lanes.push(eMax);
  const pts = [];
  lanes.forEach((ln, i) => {
    const [s, t] = i % 2 === 0 ? [nMin, nMax] : [nMax, nMin];
    pts.push([s, ln]); pts.push([t, ln]);
  });
  return pts;
}
function greedy(start, pts) {
  const order = [], left = pts.slice();
  let pos = start;
  while (left.length) {
    let bi = 0, bd = Infinity;
    left.forEach((p, i) => { const d = Math.hypot(p[0] - pos[0], p[1] - pos[1]); if (d < bd) { bd = d; bi = i; } });
    pos = left.splice(bi, 1)[0];
    order.push(pos);
  }
  return order;
}
function mission(name, kind, targets) {
  let path;
  if (kind === 'coverage') {
    path = targets.map(t => [t[0], t[1]]);
  } else {
    const byKey = new Map(targets.map(t => [t[0] + ',' + t[1], t]));
    path = greedy([0, 0], targets.map(t => [t[0], t[1]])).map(p => { const t = byKey.get(p[0] + ',' + p[1]); return [t[0], t[1]]; });
  }
  return {name, kind, targets, path};
}
const PERIMETER = [[-20, -20], [20, -20], [20, 20], [-20, 20], [-20, -20]];
const PEOPLE = OBJECTS.map((o, i) => [o, i]).filter(([o]) => o.cat === 'emergency').map(([, i]) => i);
const DAMAGE = OBJECTS.map((o, i) => [o, i]).filter(([o]) => o.cat === 'damage').map(([, i]) => i);
const LANES_N = lawnmower(-20, 20, -20, 20, 8).map(w => [w[0], w[1], null]);
const LANES_E = lawnmower(-20, 20, -20, 20, 8).map(w => [w[1], w[0], null]);
const MISSIONS = [
  mission('Coverage A: north lanes', 'coverage', LANES_N),
  mission('Coverage B: east lanes', 'coverage', LANES_E),
  mission('Perimeter inspection', 'coverage', PERIMETER.map(p => [p[0], p[1], null])),
  mission('Emergency response', 'response', PEOPLE.map(i => [OBJECTS[i].n, OBJECTS[i].e, i])),
  mission('Damage survey', 'survey', DAMAGE.map(i => [OBJECTS[i].n, OBJECTS[i].e, i])),
];

class Sim {
  constructor() { this.packetsSent = 0; this.reset(); }

  reset() {
    this.t = 0; this.n = 0; this.e = 0; this.alt = 0; this.speed = 0; this.heading = 0;
    this.state = 'TAKEOFF'; this.mi = -1; this.remaining = []; this.target = null;
    this.visits = 0; this.turnNo = 0; this.plan = null; this.battery = 100;
    this.trail = []; this.trailT = 0; this.progressT = -1; this.found = {};
    this.events = []; this.packets = []; this.doneAt = null;
    this.log('Power on. Taking off.');
  }
  log(msg) { this.events.push({t: r(this.t, 1), msg}); this.events = this.events.slice(-16); }
  send(p) { this.packets.push({...p, t: r(this.t, 1)}); this.packets = this.packets.slice(-40); this.packetsSent++; }

  startMission() {
    this.mi++;
    const m = MISSIONS[this.mi];
    this.remaining = m.targets.map(t => ({n: t[0], e: t[1], obj: t[2]}));
    this.visits = 0; this.turnNo = 1; this.target = null;
    this.log(`Mission ${this.mi + 1}/${MISSIONS.length}: ${m.name} (${this.remaining.length} targets).`);
    this.send({type: 'mission', mission: this.mi + 1, name: m.name, targets: this.remaining.length});
  }

  planNext() {
    const t0 = performance.now();
    const scored = this.remaining.map(tgt => {
      const dn = tgt.n - this.n, de = tgt.e - this.e;
      const dist = Math.hypot(dn, de);
      const bearing = mod(Math.atan2(de, dn) * 180 / Math.PI, 360);
      const turn = Math.abs(mod(bearing - this.heading + 180, 360) - 180);
      return [dist + TURN_COST * turn, dist, turn, tgt];
    });
    scored.sort((a, b) => a[0] - b[0]);
    const ms = performance.now() - t0;
    const best = scored[0][3];
    this.remaining.splice(this.remaining.indexOf(best), 1);
    const tag = best.obj != null
      ? `${OBJECTS[best.obj].label} at N ${Math.round(best.n)}, E ${Math.round(best.e)}`
      : `waypoint ${this.visits + 1}`;
    this.plan = {
      turn: this.turnNo, mission: this.mi + 1, scored: scored.length, compute_ms: r(ms, 3), next: tag,
      chosen: {n: r(best.n, 1), e: r(best.e, 1)},
      candidates: scored.slice(0, 4).map(s => ({n: r(s[3].n, 1), e: r(s[3].e, 1), cost: r(s[0], 1), turn: Math.round(s[2])})),
    };
    this.log(`Turn ${this.turnNo}: scored ${scored.length} targets in ${ms.toFixed(2)} ms, next: ${tag}.`);
    this.send({type: 'plan', turn: this.turnNo, mission: this.mi + 1, targets_scored: scored.length,
               compute_ms: r(ms, 3), next: tag, cost: r(scored[0][0], 1)});
    this.turnNo++;
    this.lastDecision = best.obj != null ? (OBJECTS[best.obj].cat === 'emergency' ? 'reach person (emergency)' : 'inspect damage site') : 'continue coverage waypoint';
    return best;
  }

  mark(i) {
    const o = OBJECTS[i];
    this.found[i] = r(this.t, 1);
    this.log(`${o.cat === 'emergency' ? 'ALERT: ' : ''}${o.label} (${o.cat}) at N ${Math.round(o.n)}, E ${Math.round(o.e)}`);
    this.send({type: 'detection', category: o.cat, label: o.label, confidence: conf(i),
               north_m: r(o.n, 1), east_m: r(o.e, 1), altitude_m: r(this.alt, 1)});
  }

  fly(dt) {
    if (!this.target) {
      if (this.remaining.length) this.target = this.planNext();
      else if (this.state === 'FLYING' && this.mi + 1 < MISSIONS.length) { this.startMission(); return; }
      else {
        this.state = 'RETURN';
        this.target = {n: 0, e: 0, obj: null, home: true};
        this.log('All missions complete. Returning home.');
        return;
      }
    }
    const tg = this.target;
    const dn = tg.n - this.n, de = tg.e - this.e, dist = Math.hypot(dn, de);
    if (dist < 1.0) {
      if (tg.home) { this.state = 'LANDING'; this.log('Over home pad. Landing.'); }
      else {
        this.visits++;
        if (tg.obj != null && this.found[tg.obj] === undefined) this.mark(tg.obj);
        this.log(`Reached target ${this.visits} of mission ${this.mi + 1}.`);
        this.target = null;
      }
      return;
    }
    const targetSpeed = Math.min(CRUISE, 0.8 * dist + 0.3);
    this.speed += clamp(targetSpeed - this.speed, -ACCEL * dt, ACCEL * dt);
    const want = mod(Math.atan2(de, dn) * 180 / Math.PI, 360);
    const diff = mod(want - this.heading + 180, 360) - 180;
    this.heading = mod(this.heading + clamp(diff, -TURN * dt, TURN * dt), 360);
    const rad = this.heading * Math.PI / 180;
    this.n += Math.cos(rad) * this.speed * dt;
    this.e += Math.sin(rad) * this.speed * dt;
    if (this.t - this.trailT >= 0.4) {
      this.trail.push([r(this.n, 2), r(this.e, 2)]);
      this.trail = this.trail.slice(-500);
      this.trailT = this.t;
    }
    const radius = this.alt * Math.tan(CAM);
    OBJECTS.forEach((o, i) => {
      if (this.found[i] === undefined && Math.hypot(o.n - this.n, o.e - this.e) <= radius) this.mark(i);
    });
  }

  step(dt) {
    this.t += dt;
    this.battery = Math.max(0, this.battery - 0.02 * dt * (1 + this.speed / 4));
    if (this.state === 'TAKEOFF') {
      this.alt = Math.min(ALT, this.alt + CLIMB * dt);
      if (this.alt >= ALT - 1e-6) { this.state = 'FLYING'; this.log(`At ${ALT} m.`); this.startMission(); }
    } else if (this.state === 'LANDING') {
      this.alt = Math.max(0, this.alt - LAND * dt);
      this.speed = 0;
      if (this.alt <= 0) { this.state = 'DONE'; this.doneAt = this.t; this.log('Landed. All missions complete.'); }
    } else if (this.state === 'DONE') {
      if (this.t - this.doneAt > 10) this.reset();
      return;
    } else {
      this.fly(dt);
    }
    if (this.t - this.progressT >= 1) {
      this.progressT = this.t;
      this.send({type: 'progress', state: this.state, mission: this.mi + 1, north_m: r(this.n, 1),
                 east_m: r(this.e, 1), altitude_m: r(this.alt, 1), battery: Math.round(this.battery)});
    }
  }

  cameraBoxes() {
    const rr = this.alt * Math.tan(CAM);
    if (rr <= 0) return [];
    const h = this.heading * Math.PI / 180;
    const boxes = [];
    OBJECTS.forEach((o, i) => {
      const dn = o.n - this.n, de = o.e - this.e;
      const fwd = dn * Math.cos(h) + de * Math.sin(h);
      const right = de * Math.cos(h) - dn * Math.sin(h);
      if (Math.abs(fwd) > rr || Math.abs(right) > rr) return;
      boxes.push({label: o.label, cat: o.cat, conf: conf(i), x: 0.5 + right / (2 * rr), y: 0.5 - fwd / (2 * rr)});
    });
    return boxes;
  }

  snapshot() {
    const m = this.mi >= 0 ? MISSIONS[this.mi] : null;
    return {
      state: this.state, t: r(this.t, 1), n: r(this.n, 2), e: r(this.e, 2), alt: r(this.alt, 2),
      heading: r(this.heading, 1), speed: r(this.speed, 2), battery: r(this.battery, 1),
      footprint_r: r(this.alt * Math.tan(CAM), 2),
      mission_index: this.mi + 1, mission_total: MISSIONS.length,
      mission_name: m ? m.name : '', mission_kind: m ? m.kind : '',
      visits: this.visits, targets_total: m ? m.targets.length : 0,
      missions: MISSIONS.map(x => ({name: x.name, kind: x.kind, path: x.path})),
      plan: this.plan,
      trail: this.trail.concat([[r(this.n, 2), r(this.e, 2)]]),
      objects: OBJECTS.map((o, i) => ({...o, id: i, found: this.found[i] !== undefined, at: this.found[i] ?? null, conf: conf(i)})),
      camera: this.cameraBoxes(),
      events: this.events.slice().reverse(),
      packets: this.packets.slice().reverse().slice(0, 25),
      packets_sent: this.packetsSent,
      simulated: true,
      gps: 'denied',
      algorithms: [
        {layer: 'Localization', name: 'Visual-inertial odometry + IMU (EKF fusion)', status: 'simulated, GPS denied'},
        {layer: 'Coverage', name: 'Lawnmower sweep over the predefined area', status: `mission ${this.mi + 1}/${MISSIONS.length}`},
        {layer: 'Target selection', name: 'Greedy: distance + 0.15 m per degree of turn', status: this.plan ? `turn ${this.plan.turn}, ${this.plan.scored} scored in ${this.plan.compute_ms} ms` : 'waiting'},
        {layer: 'Detection', name: 'YOLOv8 on camera frames', status: `${this.cameraBoxes().length} object(s) in view`},
        {layer: 'Decision', name: 'Act on the chosen target', status: this.lastDecision || '-'},
      ],
    };
  }
}

window.SIM = new Sim();
let lastTick = performance.now();
setInterval(() => {
  const now = performance.now();
  const dt = Math.min(0.1, (now - lastTick) / 1000);
  lastTick = now;
  window.SIM.step(dt);
}, 50);
