#!/usr/bin/env python3
"""
corkscrew_driver.py - AI racing league driver for TORCS (Corkscrew, standing start).

Run it from the folder that also holds snakeoil3_gym.py and corkscrew_line.py:

    python corkscrew_driver.py

How it drives
-------------
* corkscrew_line.py holds a racing line and a speed plan for the Corkscrew track,
  one entry per metre of track.  The line is the minimum-curvature path inside the
  track edges; the speed plan is the fastest speed the tyres can hold on that line
  (tyre load sensitivity, downforce, crests and banking included), with braking and
  acceleration zones worked backwards and forwards from every corner.
* Every 20 ms the driver reads three sensors - distFromStart, trackPos and angle -
  to work out exactly where the car is, then:
    - steers at a point a few metres ahead on the racing line (pure pursuit),
    - brakes by the planned deceleration plus a correction, with ABS,
    - accelerates as hard as the rear tyres allow (friction circle + traction control),
    - shifts gears to keep the engine near its power peak.
* The car model itself is untouched: this file only decides steer / throttle / brake / gear.

Older, sensor-only versions are kept next to this file (corkscrew_driver_v2.py).
"""
import csv
import math
import os

import snakeoil3_gym as snakeoil
import corkscrew_line as LINE

VERSION = 'v4'
LOG_TELEMETRY = True           # writes telemetry.csv next to this file

# ---------------- car constants (read from the car1-ow1 description, not changed) ----------------
G = 9.81
STEER_LOCK = 0.366519          # 21 degrees of steering lock, in radians
WHEELBASE = 2.95               # m
R_REAR, R_FRONT = 0.3151, 0.3024   # wheel radii, m
RATIOS = [0, 3.9, 2.9, 2.3, 1.87, 1.68, 1.54, 1.46]   # gears 1..7
DIFF = 4.5
MASS = 674.0                   # car + fuel, kg
TORQUE = [(0, 100), (1000, 100), (2000, 120), (3000, 140), (4000, 160), (5000, 180), (6000, 220),
          (7000, 260), (8000, 300), (9000, 340), (13000, 345), (14000, 350), (15000, 355),
          (16000, 360), (18000, 360), (19000, 330), (20000, 300)]

# ---------------- tuning (found by automatic search in a headless copy of the simulator) ----------------
LOOK_BASE, LOOK_GAIN = 0.50, 0.0549     # steering look-ahead = base + gain * speed (m)
LOOK_MIN, LOOK_MAX = 3.62, 28.0
CROSS_GAIN, CROSS_SPEED = 0.035, 29.0   # small extra steering per metre off the line (fades with speed)
PLAN_AHEAD = 0.0406              # s, how far ahead the speed plan is read
KP_THROTTLE = 1.257            # throttle per m/s below the plan
KP_BRAKE = 5.00               # extra deceleration (m/s^2) per m/s above the plan
DEADBAND = 0.3                 # m/s
BRAKE_SWITCH = 1.5             # m/s^2 of wanted deceleration before the brakes are used
DRAG = 0.0013                  # aerodynamic drag deceleration = DRAG * v^2
DOWNFORCE = 0.0048             # extra tyre load per v^2 (in m/s^2)
K_BRAKE = 47.3                 # deceleration (m/s^2) of a full brake command if no wheel locks
GRIP_USE = 0.9595              # share of tyre grip the friction circle assumes
BRAKE_GRIP = 1.9
TRACTION = 1.200              # usable drive force, as a multiple of tyre load
CIRCLE_FLOOR = 0.12
TCS_SLIP, TCS_GAIN = 3.50, 0.210   # traction control: allowed wheel-spin (m/s), throttle cut per m/s
ABS_SLIP = 0.218          # ABS: allowed wheel slip ratio
RPM_UP, RPM_DOWN = 18500.0, 16291.4
OFF_TRACK_SPEED = 14.0         # m/s, speed limit while off the track


def engine_torque(rpm):
    if rpm <= 0:
        return 100.0
    for k in range(1, len(TORQUE)):
        if rpm <= TORQUE[k][0]:
            a, b = TORQUE[k - 1], TORQUE[k]
            return a[1] + (b[1] - a[1]) * (rpm - a[0]) / (b[0] - a[0])
    return 300.0


def tyre_accel_limit(v):
    """Largest acceleration the tyres can give at speed v (load-sensitive tyres + downforce)."""
    an = G + DOWNFORCE * v * v
    return GRIP_USE * 1.76 * (0.8 + 0.8 * math.exp(-1.386294 * an / G / 1.2)) * an, an


class Racer:
    def __init__(self):
        self.n = len(LINE.CX)
        self.ds = LINE.DS
        self.half = LINE.HALF_WIDTH
        self.cx, self.cy = LINE.CX, LINE.CY
        self.hx = [math.cos(h) for h in LINE.HEADING]
        self.hy = [math.sin(h) for h in LINE.HEADING]
        self.off, self.v, self.k, self.ax = LINE.OFFSET, LINE.SPEED, LINE.CURVATURE, LINE.ACCEL
        # racing-line points in world coordinates
        self.px = [self.cx[i] - self.hy[i] * self.off[i] for i in range(self.n)]
        self.py = [self.cy[i] + self.hx[i] * self.off[i] for i in range(self.n)]
        self.abs = 1.0
        self.throttle = 0.0
        self.since_shift = 99

    def at(self, arr, idx):
        """Linear interpolation in a per-metre table that wraps around the lap."""
        base = math.floor(idx)
        i0 = int(base) % self.n
        i1 = (i0 + 1) % self.n
        return arr[i0] + (arr[i1] - arr[i0]) * (idx - base)

    def drive(self, S, R):
        v = max(S['speedX'], 0.0) / 3.6                     # m/s
        idx = S['distFromStart'] / self.ds
        y = S['trackPos'] * self.half                       # metres left of the centre line
        ang = S['angle']

        # ---- where is the car? (world position and heading from the track table)
        hx, hy = self.at(self.hx, idx), self.at(self.hy, idx)
        norm = math.hypot(hx, hy)
        hx, hy = hx / norm, hy / norm
        X = self.at(self.cx, idx) - hy * y
        Y = self.at(self.cy, idx) + hx * y
        ca, sa = math.cos(ang), math.sin(ang)
        fx, fy = hx * ca + hy * sa, hy * ca - hx * sa       # unit vector the car points along

        # ---- steering: pure pursuit of a point ahead on the racing line
        look = min(max(LOOK_BASE + LOOK_GAIN * v, LOOK_MIN), LOOK_MAX)
        j = idx + look / self.ds
        dx, dy = self.at(self.px, j) - X, self.at(self.py, j) - Y
        ahead, left = dx * fx + dy * fy, -dx * fy + dy * fx
        alpha = math.atan2(left, ahead)
        delta = math.atan2(2.0 * WHEELBASE * math.sin(alpha), max(math.hypot(ahead, left), 1.0))
        line_err = self.at(self.off, idx) - y              # metres the racing line is to our left
        steer = delta / STEER_LOCK + CROSS_GAIN * line_err / (1.0 + v / CROSS_SPEED)
        R['steer'] = max(-1.0, min(1.0, steer))

        # ---- speed plan
        jp = idx + v * PLAN_AHEAD / self.ds
        target = self.at(self.v, jp)
        off_track = abs(S['trackPos']) > 1.0
        if off_track:
            target = min(target, OFF_TRACK_SPEED)
        err = target - v
        # friction circle: how much of the tyre is already used for cornering
        limit, load = tyre_accel_limit(v)
        used = min(1.0, v * v * abs(self.at(self.k, idx)) / limit)
        spare = max(CIRCLE_FLOOR, math.sqrt(1.0 - used * used))
        w = S['wheelSpinVel']
        v_rear = 0.5 * (w[2] + w[3]) * R_REAR
        v_front = 0.5 * (w[0] + w[1]) * R_FRONT
        wanted = self.at(self.ax, jp) + KP_BRAKE * err      # planned acceleration + correction
        if off_track:
            wanted = min(wanted, KP_BRAKE * err)
        gear = int(S['gear'])

        if err > -DEADBAND and wanted > -BRAKE_SWITCH:
            # ---- throttle
            thr = max(0.0, min(1.0, KP_THROTTLE * (err + DEADBAND)))
            g = min(max(gear, 1), 7)
            engine = engine_torque(S['rpm']) * RATIOS[g] * DIFF * 0.92 / R_REAR / MASS
            thr = min(thr, TRACTION * load * spare / max(engine, 1.0))
            spin = v_rear - max(v, v_front)
            if spin > TCS_SLIP:
                thr -= TCS_GAIN * (spin - TCS_SLIP)
            thr = max(0.0, thr)
            self.throttle = min(thr, self.throttle + 0.25)
            self.abs = min(1.0, self.abs + 0.1)
            R['accel'], R['brake'] = self.throttle, 0.0
        else:
            # ---- brakes: planned deceleration, limited by grip, with ABS
            self.throttle = 0.0
            decel = max(0.0, -wanted - DRAG * v * v)
            decel = min(decel, BRAKE_GRIP * load * (spare if used > 0.3 else 1.0))
            brake = min(1.0, decel / K_BRAKE)
            if v > 4.0:
                slowest = min(w[0] * R_FRONT, w[1] * R_FRONT, w[2] * R_REAR, w[3] * R_REAR)
                if (v - slowest) / v > ABS_SLIP:
                    self.abs = max(0.2, self.abs * 0.8)
                else:
                    self.abs = min(1.0, self.abs + 0.05)
            R['accel'], R['brake'] = 0.0, brake * self.abs

        # ---- gears: stay near the power peak
        self.since_shift += 1
        new_gear = gear if gear >= 1 else 1
        if gear >= 1 and self.since_shift > 8:
            if S['rpm'] > RPM_UP and gear < 7:
                new_gear = gear + 1
            elif gear > 1:
                rpm_lower = max(v, v_rear) / R_REAR * RATIOS[gear - 1] * DIFF * 30.0 / math.pi
                if rpm_lower < RPM_DOWN:
                    new_gear = gear - 1
            if new_gear != gear:
                self.since_shift = 0
        R['gear'] = new_gear
        R['clutch'] = 0.0
        return target * 3.6, line_err


class Report:
    def __init__(self):
        self.best_lap = None
        self.last_reported = 0.0
        self.damage = 0.0
        self.top = 0.0
        self.cur = 0.0
        self.raced = 0.0

    def update(self, S):
        self.damage = max(self.damage, S.get('damage', 0.0))
        self.top = max(self.top, S['speedX'])
        self.cur = max(self.cur, S.get('curLapTime', 0.0))
        self.raced = S.get('distRaced', self.raced)
        lap = S.get('lastLapTime', 0.0)
        if lap and lap > 0 and lap != self.last_reported:
            self.last_reported = lap
            if self.best_lap is None or lap < self.best_lap:
                self.best_lap = lap
            mins, secs = divmod(lap, 60)
            print('\n=== LAP COMPLETE: %d:%06.3f  | damage %.0f | top speed %.0f km/h ===\n'
                  % (mins, secs, self.damage, self.top))


def main():
    print('Corkscrew driver %s' % VERSION)
    c = snakeoil.Client()
    racer = Racer()
    rep = Report()
    log = writer = None
    last_logged = None
    if LOG_TELEMETRY:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'telemetry.csv')
        log = open(path, 'w', newline='')
        writer = csv.writer(log)
        writer.writerow(['lapTime', 'distFromStart', 'speed', 'target', 'gear', 'rpm', 'steer',
                         'accel', 'brake', 'trackPos', 'angle', 'lineError', 'damage'])
    try:
        for step in range(c.maxSteps, 0, -1):
            c.get_servers_input()
            if c.so is None:
                break
            S, R = c.S.d, c.R.d
            if not S or 'track' not in S:
                continue
            target, line_err = racer.drive(S, R)
            rep.update(S)
            now = S.get('curLapTime', 0)
            if writer and step % 5 == 0 and now != last_logged:
                last_logged = now
                writer.writerow(['%.2f' % now, '%.1f' % S.get('distFromStart', 0), '%.1f' % S['speedX'],
                                 '%.1f' % target, int(S['gear']), int(S['rpm']), '%.3f' % R['steer'],
                                 '%.2f' % R['accel'], '%.2f' % R['brake'], '%.3f' % S['trackPos'],
                                 '%.3f' % S['angle'], '%.2f' % line_err, '%.0f' % S.get('damage', 0)])
            c.respond_to_server()
    finally:
        if log:
            log.close()
        if rep.best_lap:
            print('Best lap: %.3f s, damage %.0f' % (rep.best_lap, rep.damage))
        else:
            print('Race ended at about %.2f s after %.0f m | damage %.0f | top speed %.0f km/h (%s)'
                  % (rep.cur, rep.raced, rep.damage, rep.top, VERSION))
        c.shutdown()


if __name__ == '__main__':
    main()
