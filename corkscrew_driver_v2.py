#!/usr/bin/env python3
"""
corkscrew_driver.py - AI racing league driver for TORCS (Corkscrew, standing start).

Put this file in the same folder as snakeoil3_gym.py (the gym_torcs starter kit),
start a Quick Race on Corkscrew with the scr_server 1 driver, then run:

    python corkscrew_driver.py

What is different from the base drive_example():
  * target speed comes from how far the track is clear ahead (a braking curve),
    instead of one fixed 100 km/h target
  * real braking, with ABS so the wheels do not lock
  * traction control on the throttle
  * rpm based gear shifting (the base script sits in 6th from 170 km/h)
  * steering that also aims at the longest clear line through a corner
  * off-track recovery, lap time and damage report, telemetry.csv for tuning

All tuning numbers are in the TUNING block below.
"""
import csv
import math
import os
import re

import snakeoil3_gym as snakeoil

# ============================ TUNING ============================
A_CORNER = 13.0        # m/s^2, shapes the speed allowed inside a corner (short clear distance)
A_BRAKE = 19.0         # m/s^2 braking assumed on the approach to a corner. Higher = brakes later
CORNER_DIST = 60.0     # m of clear track below which we are "in" a corner
CORNER_BOOST = 1.06    # extra corner speed for medium/fast corners (1.0 = lap 1 behaviour)
BRAKE_MARGIN = 8.0     # metres kept in hand before the track edge ahead
V_CORNER_MIN = 58.0    # km/h, slowest speed the plan ever asks for (tightest hairpin)
V_MAX_OPEN = 330.0     # km/h cap while the sensors see 200 m of clear track
STEER_LOCK = 0.785     # rad, steering lock used to scale the steer command
CENTRE_GAIN = 0.45     # how strongly the car is pulled back to the middle of the track
AIM_GAIN = 0.035       # how strongly it aims at the longest clear sensor (0 = off)
UPSHIFT_RPM = 17500.0  # shift up above this
DOWNSHIFT_RPM = 10500.0  # shift down below this
WHEEL_RADIUS = 0.33    # m, used by ABS / traction control
LOG_TELEMETRY = True   # writes telemetry.csv next to this file
# ================================================================

VERSION = 'v2'
SENSOR_MAX = 199.5     # track sensors saturate at 200 m


class Memory:
    """Things the driver remembers between simulation steps (50 per second)."""
    def __init__(self):
        self.steps_since_shift = 100
        self.limiter_steps = 0
        self.last_rpm = 0.0
        self.upshift_rpm = UPSHIFT_RPM
        self.downshift_rpm = DOWNSHIFT_RPM
        self.accel = 1.0
        self.best_lap = None
        self.last_reported_lap = 0.0
        self.max_damage = 0.0
        self.top_speed = 0.0
        self.last_cur_lap = 0.0
        self.last_dist = 0.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def target_speed_kmh(clear_dist):
    """Fastest speed from which we can still slow to V_CORNER_MIN in the clear distance."""
    if clear_dist >= SENSOR_MAX:
        return V_MAX_OPEN
    v_min = V_CORNER_MIN / 3.6

    def corner_speed(d):
        usable = max(0.0, d - BRAKE_MARGIN)
        # no boost in hairpins (d <= 12 m), full boost from 25 m of clear track
        boost = 1.0 + (CORNER_BOOST - 1.0) * clamp((d - 12.0) / 13.0, 0.0, 1.0)
        return math.sqrt(v_min * v_min + 2.0 * A_CORNER * usable) * boost

    if clear_dist <= CORNER_DIST:
        v = corner_speed(clear_dist)
    else:
        # approach: brake at A_BRAKE down to the speed allowed at CORNER_DIST
        v0 = corner_speed(CORNER_DIST)
        v = math.sqrt(v0 * v0 + 2.0 * A_BRAKE * (clear_dist - CORNER_DIST))
    return min(V_MAX_OPEN, v * 3.6)


def steering(S):
    track = S['track']
    # basic: point along the track and drift back to the centre line
    steer = (S['angle'] - S['trackPos'] * CENTRE_GAIN) / STEER_LOCK
    # aim: lean towards the direction with the most clear track (higher index = right,
    # and positive steer = left, hence the minus sign)
    if AIM_GAIN and len(track) == 19 and min(track) >= 0 and abs(S['trackPos']) < 0.9:
        centre = range(5, 14)
        best = max(centre, key=lambda i: track[i])
        if track[best] < SENSOR_MAX and track[best] > track[9] + 1.0:
            steer -= (best - 9) * AIM_GAIN
    return clamp(steer, -1.0, 1.0)


def wheel_speeds(S):
    w = S['wheelSpinVel']
    front = (w[0] + w[1]) / 2.0 * WHEEL_RADIUS
    rear = (w[2] + w[3]) / 2.0 * WHEEL_RADIUS
    return front, rear


def gearbox(S, R, mem):
    gear = int(S['gear'])
    rpm = S['rpm']
    mem.steps_since_shift += 1
    if gear < 1:
        mem.steps_since_shift = 0
        return 1
    new_gear = gear
    if mem.steps_since_shift > 12:
        # rev limiter detection: flat out, rpm no longer rising -> shift up and learn the limit
        if R['accel'] > 0.9 and rpm > 6000 and abs(rpm - mem.last_rpm) < 15:
            mem.limiter_steps += 1
        else:
            mem.limiter_steps = 0
        if gear < 6 and (rpm > mem.upshift_rpm or mem.limiter_steps > 20):
            if mem.limiter_steps > 20:
                mem.upshift_rpm = rpm * 0.97
                mem.downshift_rpm = mem.upshift_rpm * 0.6
            new_gear = gear + 1
        elif gear > 1 and rpm < mem.downshift_rpm:
            new_gear = gear - 1
        if new_gear != gear:
            mem.steps_since_shift = 0
            mem.limiter_steps = 0
    mem.last_rpm = rpm
    return new_gear


def drive(c, mem):
    S, R = c.S.d, c.R.d
    speed = S['speedX']                      # km/h
    v = speed / 3.6                          # m/s
    track = S['track']
    off_track = abs(S['trackPos']) > 1.0 or min(track) < 0

    R['steer'] = steering(S)

    # ---------- speed plan ----------
    if off_track:
        target = 60.0
    else:
        clear = max(track[8], track[9], track[10])
        target = target_speed_kmh(clear)
        # do not ask for full speed while the wheels are turned hard
        target *= 1.0 - 0.35 * min(1.0, abs(R['steer']) ** 1.5)
        target = max(target, V_CORNER_MIN * 0.75)

    front, rear = wheel_speeds(S)
    error = target - speed

    if error > 0:
        # ---------- throttle ----------
        accel = clamp(0.3 + error / 6.0, 0.0, 1.0)
        if speed > 30:
            accel *= 1.0 - 0.4 * min(1.0, abs(R['steer']))
        # traction control: rear wheels spinning faster than the fronts
        slip = rear - front
        if slip > 2.0 and speed < 200:
            accel -= min(0.8, (slip - 2.0) * 0.12)
        if off_track:
            accel = min(accel, 0.4)
        # smooth increases a little, cuts are instant
        mem.accel = min(accel, mem.accel + 0.1)
        R['accel'] = clamp(mem.accel, 0.0, 1.0)
        R['brake'] = 0.0
    else:
        # ---------- brakes with ABS ----------
        mem.accel = 0.0
        R['accel'] = 0.0
        brake = clamp(-error / 6.0, 0.0, 1.0)
        if v > 3.0:
            slowest = min(S['wheelSpinVel']) * WHEEL_RADIUS
            lock = v - slowest
            if lock > 2.0:
                brake -= min(brake, (lock - 2.0) / 3.0)
        # less brake while steering hard, so the car still turns
        brake *= 1.0 - 0.4 * min(1.0, abs(R['steer']))
        R['brake'] = clamp(brake, 0.0, 1.0)

    R['gear'] = gearbox(S, R, mem)
    R['clutch'] = 0.0
    return target


def report(S, mem):
    mem.max_damage = max(mem.max_damage, S.get('damage', 0.0))
    mem.top_speed = max(mem.top_speed, S['speedX'])
    mem.last_cur_lap = max(mem.last_cur_lap, S.get('curLapTime', 0.0))
    mem.last_dist = S.get('distRaced', mem.last_dist)
    lap = S.get('lastLapTime', 0.0)
    if lap and lap > 0 and lap != mem.last_reported_lap:
        mem.last_reported_lap = lap
        if mem.best_lap is None or lap < mem.best_lap:
            mem.best_lap = lap
        mins, secs = divmod(lap, 60)
        print('\n=== LAP COMPLETE: %d:%06.3f  | damage %.0f | top speed %.0f km/h ===\n'
              % (mins, secs, mem.max_damage, mem.top_speed))


def server_port():
    """Use the same port the starter kit's own script uses, if it names one."""
    try:
        with open(snakeoil.__file__, encoding='utf-8', errors='ignore') as f:
            m = re.search(r'Client\(\s*p\s*=\s*(\d+)', f.read())
        return int(m.group(1)) if m else None
    except OSError:
        return None


def main():
    print('Corkscrew driver %s' % VERSION)
    c = snakeoil.Client(p=server_port())
    mem = Memory()
    last_logged = None
    log = writer = None
    if LOG_TELEMETRY:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'telemetry.csv')
        log = open(path, 'w', newline='')
        writer = csv.writer(log)
        writer.writerow(['lapTime', 'distFromStart', 'speed', 'target', 'gear', 'rpm',
                         'steer', 'accel', 'brake', 'trackPos', 'angle', 'clear', 'damage'])
    try:
        for step in range(c.maxSteps, 0, -1):
            c.get_servers_input()
            S, R = c.S.d, c.R.d
            if not S or 'track' not in S:
                continue
            target = drive(c, mem)
            report(S, mem)
            lap_now = S.get('curLapTime', 0)
            if writer and step % 5 == 0 and lap_now != last_logged:
                last_logged = lap_now
                writer.writerow(['%.2f' % S.get('curLapTime', 0), '%.1f' % S.get('distFromStart', 0),
                                 '%.1f' % S['speedX'], '%.1f' % target, int(S['gear']), int(S['rpm']),
                                 '%.3f' % R['steer'], '%.2f' % R['accel'], '%.2f' % R['brake'],
                                 '%.3f' % S['trackPos'], '%.3f' % S['angle'],
                                 '%.1f' % max(S['track'][8:11]), '%.0f' % S.get('damage', 0)])
            c.respond_to_server()
    finally:
        if log:
            log.close()
        if mem.best_lap:
            print('Best lap: %.3f s, damage %.0f' % (mem.best_lap, mem.max_damage))
        else:
            print('Race ended at about %.2f s after %.0f m | damage %.0f | top speed %.0f km/h (%s)'
                  % (mem.last_cur_lap, mem.last_dist, mem.max_damage, mem.top_speed, VERSION))
        c.shutdown()


if __name__ == '__main__':
    main()
