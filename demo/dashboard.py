"""
Drone-Inspector 3D demo: simulated mission with a live browser view.

Standard library only. It does not run PX4, Gazebo or YOLO. A background loop
simulates the drone flying five missions in one flight:

  1. Coverage A  - north lanes over the predefined area
  2. Coverage B  - east lanes
  3. Perimeter   - inspect the boundary
  4. Emergency   - reach every person (emergency) on the ground
  5. Damage      - reach every damage site (crack, spalling, corrosion, leak)

At every turn the planner scores all remaining targets (distance plus a
turn penalty), picks the cheapest, and records how long the scoring took.
Camera boxes, detections and ground-station packets are produced as in the
real system (see ground_link.py), but only in simulation.

Run:  python3 demo/dashboard.py     then open http://localhost:4344
Models (optional): demo/models/drone.glb and demo/models/map.glb
"""

import json
import math
import os
import shutil
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'px4_orbit_inspection'))
from px4_orbit_inspection.area_patrol import lawnmower_waypoints  # noqa: E402

PORT = 4344
ALT = 6.0
CLIMB, LAND = 2.0, 1.0
CRUISE, ACCEL, TURN = 4.0, 2.0, 60.0
TURN_COST = 0.15                       # planner cost per degree of turn (metres)
CAMERA_HALF_FOV = math.radians(35)

# Ground objects: north, east (m), label, category (emergency = a person needs help).
OBJECTS = [
    dict(n=12.0, e=-16.0, label='crack', cat='damage'),
    dict(n=-6.0, e=12.0, label='spalling', cat='damage'),
    dict(n=16.0, e=8.0, label='corrosion', cat='damage'),
    dict(n=-14.0, e=-8.0, label='leak', cat='damage'),
    dict(n=-2.0, e=-18.0, label='crack', cat='damage'),
    dict(n=18.0, e=-6.0, label='spalling', cat='damage'),
    dict(n=4.0, e=-4.0, label='person', cat='emergency'),
    dict(n=-18.0, e=16.0, label='person', cat='emergency'),
    dict(n=-8.0, e=-14.0, label='person', cat='emergency'),
    dict(n=14.0, e=-2.0, label='person', cat='emergency'),
    dict(n=-12.0, e=4.0, label='person', cat='emergency'),
]
STRUCTURES = {
    'pipeline': {'n0': -23.0, 'n1': 23.0, 'e': -2.0},
    'tank': {'n': -4.0, 'e': 23.0, 'r': 3.0, 'h': 6.0},
    'building': {'n0': 8.0, 'n1': 18.0, 'e0': -23.0, 'e1': -13.0, 'h': 5.0},
}


def _greedy_order(start, points):
    """Nearest-neighbour order, used only to draw each mission's reference path."""
    order, pos, left = [], start, list(points)
    while left:
        nxt = min(left, key=lambda p: math.hypot(p[0] - pos[0], p[1] - pos[1]))
        order.append(nxt)
        left.remove(nxt)
        pos = nxt
    return order


def _lanes(north_first):
    pts = lawnmower_waypoints(-20.0, 20.0, -20.0, 20.0, 8.0, ALT)
    return [(w[0], w[1]) for w in pts] if north_first else [(w[1], w[0]) for w in pts]


PERIMETER = [(-20.0, -20.0), (20.0, -20.0), (20.0, 20.0), (-20.0, 20.0), (-20.0, -20.0)]
PEOPLE = [i for i, o in enumerate(OBJECTS) if o['cat'] == 'emergency']
DAMAGE = [i for i, o in enumerate(OBJECTS) if o['cat'] == 'damage']


def _mission(name, kind, targets):
    """targets: list of (n, e, object_index or None)."""
    return {'name': name, 'kind': kind, 'targets': targets,
            'path': [[t[0], t[1]] for t in (targets if kind == 'coverage' else
                     _sorted_by_order([(t[0], t[1], t[2]) for t in targets]))]}


def _sorted_by_order(targets):
    keys = {(t[0], t[1]): t for t in targets}
    return [keys[p] for p in _greedy_order((0.0, 0.0), [(t[0], t[1]) for t in targets])]


MISSIONS = [
    _mission('Coverage A: north lanes', 'coverage', [(w[0], w[1], None) for w in _lanes(True)]),
    _mission('Coverage B: east lanes', 'coverage', [(w[0], w[1], None) for w in _lanes(False)]),
    _mission('Perimeter inspection', 'coverage', [(p[0], p[1], None) for p in PERIMETER]),
    _mission('Emergency response', 'response', [(OBJECTS[i]['n'], OBJECTS[i]['e'], i) for i in PEOPLE]),
    _mission('Damage survey', 'survey', [(OBJECTS[i]['n'], OBJECTS[i]['e'], i) for i in DAMAGE]),
]


def confidence(idx):
    """Simulated detector confidence, stable per object."""
    return round(0.82 + ((idx * 37) % 15) / 100.0, 2)


class Sim:
    def __init__(self):
        self.lock = threading.Lock()
        self.packets_sent = 0
        self._reset()
        threading.Thread(target=self._run, daemon=True).start()

    def _reset(self):
        self.t = 0.0
        self.n = self.e = self.alt = 0.0
        self.speed = 0.0
        self.heading = 0.0
        self.state = 'TAKEOFF'
        self.mi = -1
        self.remaining = []
        self.target = None
        self.visits = 0
        self.turn_no = 0
        self.plan = None
        self.battery = 100.0
        self.trail = []
        self.trail_t = 0.0
        self.progress_t = -1.0
        self.found = {}
        self.events = []
        self.packets = []
        self.done_at = None
        self._log('Power on. Taking off.')

    def _log(self, msg):
        self.events.append({'t': round(self.t, 1), 'msg': msg})
        self.events = self.events[-16:]

    def _send(self, packet):
        """Stand-in for the UDP JSON link to the ground station (see ground_link.py)."""
        self.packets.append(dict(packet, t=round(self.t, 1)))
        self.packets = self.packets[-40:]
        self.packets_sent += 1

    def _start_mission(self):
        self.mi += 1
        m = MISSIONS[self.mi]
        self.remaining = [{'n': t[0], 'e': t[1], 'obj': t[2]} for t in m['targets']]
        self.visits = 0
        self.turn_no = 1
        self.target = None
        self._log(f'Mission {self.mi + 1}/{len(MISSIONS)}: {m["name"]} ({len(self.remaining)} targets).')
        self._send({'type': 'mission', 'mission': self.mi + 1, 'name': m['name'], 'targets': len(self.remaining)})

    def _plan(self):
        """Score every remaining target from the current pose and pick the cheapest."""
        t0 = time.perf_counter()
        scored = []
        for tgt in self.remaining:
            dn, de = tgt['n'] - self.n, tgt['e'] - self.e
            dist = math.hypot(dn, de)
            bearing = math.degrees(math.atan2(de, dn)) % 360
            turn = abs((bearing - self.heading + 180) % 360 - 180)
            scored.append((dist + TURN_COST * turn, dist, turn, tgt))
        scored.sort(key=lambda s: s[0])
        compute_ms = (time.perf_counter() - t0) * 1000
        best = scored[0][3]
        self.remaining.remove(best)
        if best['obj'] is not None:
            tag = f"{OBJECTS[best['obj']]['label']} at N {best['n']:.0f}, E {best['e']:.0f}"
        else:
            tag = f'waypoint {self.visits + 1}'
        self.plan = {
            'turn': self.turn_no, 'mission': self.mi + 1, 'scored': len(scored),
            'compute_ms': round(compute_ms, 3), 'next': tag,
            'chosen': {'n': round(best['n'], 1), 'e': round(best['e'], 1)},
            'candidates': [{'n': round(s[3]['n'], 1), 'e': round(s[3]['e'], 1),
                            'cost': round(s[0], 1), 'turn': round(s[2])} for s in scored[:4]],
        }
        self._log(f'Turn {self.turn_no}: scored {len(scored)} targets in {compute_ms:.2f} ms, next: {tag}.')
        self._send({'type': 'plan', 'turn': self.turn_no, 'mission': self.mi + 1,
                    'targets_scored': len(scored), 'compute_ms': round(compute_ms, 3),
                    'next': tag, 'cost': round(scored[0][0], 1)})
        self.turn_no += 1
        return best

    def _step(self, dt):
        self.t += dt
        self.battery = max(0.0, self.battery - 0.02 * dt * (1 + self.speed / 4))
        if self.state == 'TAKEOFF':
            self.alt = min(ALT, self.alt + CLIMB * dt)
            if self.alt >= ALT - 1e-6:
                self.state = 'FLYING'
                self._log(f'At {ALT:.0f} m.')
                self._start_mission()
        elif self.state == 'LANDING':
            self.alt = max(0.0, self.alt - LAND * dt)
            self.speed = 0.0
            if self.alt <= 0.0:
                self.state = 'DONE'
                self.done_at = self.t
                self._log('Landed. All missions complete.')
        elif self.state == 'DONE':
            if self.t - self.done_at > 10.0:
                self._reset()
            return
        else:
            self._fly(dt)

        if self.t - self.progress_t >= 1.0:
            self.progress_t = self.t
            self._send({'type': 'progress', 'state': self.state,
                        'mission': self.mi + 1, 'north_m': round(self.n, 1), 'east_m': round(self.e, 1),
                        'altitude_m': round(self.alt, 1), 'battery': round(self.battery)})

    def _fly(self, dt):
        if self.target is None:
            if self.remaining:
                self.target = self._plan()
            elif self.state == 'FLYING' and self.mi + 1 < len(MISSIONS):
                self._start_mission()
                return
            else:
                self.state = 'RETURN'
                self.target = {'n': 0.0, 'e': 0.0, 'obj': None, 'home': True}
                self._log('All missions complete. Returning home.')
                return

        tgt = self.target
        dn, de = tgt['n'] - self.n, tgt['e'] - self.e
        dist = math.hypot(dn, de)
        if dist < 1.0:
            if tgt.get('home'):
                self.state = 'LANDING'
                self._log('Over home pad. Landing.')
            else:
                self.visits += 1
                if tgt['obj'] is not None and tgt['obj'] not in self.found:
                    self._mark(tgt['obj'])
                self._log(f'Reached target {self.visits} of mission {self.mi + 1}.')
                self.target = None
            return

        target_speed = min(CRUISE, 0.8 * dist + 0.3)
        self.speed += max(-ACCEL * dt, min(ACCEL * dt, target_speed - self.speed))
        want = math.degrees(math.atan2(de, dn)) % 360
        diff = (want - self.heading + 180) % 360 - 180
        self.heading = (self.heading + max(-TURN * dt, min(TURN * dt, diff))) % 360
        rad = math.radians(self.heading)
        self.n += math.cos(rad) * self.speed * dt
        self.e += math.sin(rad) * self.speed * dt

        if self.t - self.trail_t >= 0.4:
            self.trail.append([round(self.n, 2), round(self.e, 2)])
            self.trail = self.trail[-500:]
            self.trail_t = self.t

        radius = self.alt * math.tan(CAMERA_HALF_FOV)
        for i, obj in enumerate(OBJECTS):
            if i not in self.found and math.hypot(obj['n'] - self.n, obj['e'] - self.e) <= radius:
                self._mark(i)

    def _mark(self, i):
        obj = OBJECTS[i]
        self.found[i] = round(self.t, 1)
        prefix = 'ALERT: ' if obj['cat'] == 'emergency' else ''
        self._log(f"{prefix}{obj['label']} ({obj['cat']}) at N {obj['n']:.0f}, E {obj['e']:.0f}")
        self._send({'type': 'detection', 'category': obj['cat'], 'label': obj['label'],
                    'confidence': confidence(i), 'north_m': round(obj['n'], 1),
                    'east_m': round(obj['e'], 1), 'altitude_m': round(self.alt, 1)})

    def _run(self):
        last = time.time()
        while True:
            now = time.time()
            dt = min(0.1, now - last)
            last = now
            with self.lock:
                self._step(dt)
            time.sleep(0.05)

    def _camera_boxes(self):
        """Simulated onboard AI: boxes for objects inside the camera view, in 0..1 image coordinates."""
        r = self.alt * math.tan(CAMERA_HALF_FOV)
        h = math.radians(self.heading)
        boxes = []
        if r <= 0:
            return boxes
        for i, obj in enumerate(OBJECTS):
            dn, de = obj['n'] - self.n, obj['e'] - self.e
            fwd = dn * math.cos(h) + de * math.sin(h)
            right = de * math.cos(h) - dn * math.sin(h)
            if abs(fwd) > r or abs(right) > r:
                continue
            boxes.append({'label': obj['label'], 'cat': obj['cat'], 'conf': confidence(i),
                          'x': 0.5 + right / (2 * r), 'y': 0.5 - fwd / (2 * r)})
        return boxes

    def snapshot(self):
        with self.lock:
            r = self.alt * math.tan(CAMERA_HALF_FOV)
            mission = MISSIONS[self.mi] if self.mi >= 0 else None
            return {
                'state': self.state, 't': round(self.t, 1),
                'n': round(self.n, 2), 'e': round(self.e, 2), 'alt': round(self.alt, 2),
                'heading': round(self.heading, 1), 'speed': round(self.speed, 2),
                'battery': round(self.battery, 1),
                'footprint_r': round(r, 2),
                'mission_index': self.mi + 1, 'mission_total': len(MISSIONS),
                'mission_name': mission['name'] if mission else '',
                'mission_kind': mission['kind'] if mission else '',
                'visits': self.visits,
                'targets_total': len(mission['targets']) if mission else 0,
                'missions': [{'name': m['name'], 'kind': m['kind'], 'path': m['path']} for m in MISSIONS],
                'plan': self.plan,
                'trail': self.trail + [[round(self.n, 2), round(self.e, 2)]],
                'objects': [dict(o, id=i, found=(i in self.found), at=self.found.get(i), conf=confidence(i))
                            for i, o in enumerate(OBJECTS)],
                'structures': STRUCTURES,
                'camera': self._camera_boxes(),
                'events': self.events[::-1],
                'packets': self.packets[::-1][:25],
                'packets_sent': self.packets_sent,
                'simulated': True,
            }


SIM = Sim()
PAGE = open(os.path.join(HERE, 'dashboard.html'), encoding='utf-8').read()
MODEL_DIR = os.path.join(HERE, 'models')


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/api/state':
            body = json.dumps(SIM.snapshot()).encode()
            return self._send_bytes(body, 'application/json')
        if path.startswith('/models/'):
            name = os.path.basename(path)
            full = os.path.join(MODEL_DIR, name)
            if not (name.endswith('.glb') and os.path.isfile(full)):
                self.send_response(404)
                self.send_header('Content-Length', '0')
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'model/gltf-binary')
            self.send_header('Content-Length', str(os.path.getsize(full)))
            self.end_headers()
            with open(full, 'rb') as f:  # stream: map.glb is ~100 MB
                shutil.copyfileobj(f, self.wfile)
            return
        self._send_bytes(PAGE.encode('utf-8'), 'text/html; charset=utf-8')

    def _send_bytes(self, body, ctype):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


if __name__ == '__main__':
    server = ThreadingHTTPServer(('127.0.0.1', PORT), Handler)
    print(f'Drone-Inspector demo at http://localhost:{PORT}  (Ctrl+C to stop)')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
