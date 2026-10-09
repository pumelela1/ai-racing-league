"""Track-aware racing driver logic (pure Python, no third-party imports)."""
import math
G = 9.81
STEER_LOCK = 0.366519      # 21 deg
WHEELBASE = 2.95
R_REAR = 0.3151; R_FRONT = 0.3024
RATIOS = [0, 3.9, 2.9, 2.3, 1.87, 1.68, 1.54, 1.46]; DIFF = 4.5
MASS = 674.0
TQ = [(0,100),(1000,100),(2000,120),(3000,140),(4000,160),(5000,180),(6000,220),(7000,260),(8000,300),(9000,340),(13000,345),(14000,350),(15000,355),(16000,360),(18000,360),(19000,330),(20000,300)]

def torque(rpm):
    if rpm <= 0: return 100.0
    for k in range(1, len(TQ)):
        if rpm <= TQ[k][0]:
            a, b = TQ[k-1], TQ[k]
            return a[1] + (b[1]-a[1]) * (rpm-a[0]) / (b[0]-a[0])
    return 300.0

DEFAULT = dict(
    la0=1.2, la1=0.06, la_min=5.0, la_max=28.0,   # pure-pursuit lookahead (m, s)
    ke=0.0, ke_v=30.0, mode='pp', merge=0.0, merge_db=1.0,                 # cross-track gain
    pv0=2.0, pv1=0.10, kv0=1.0, kv1=0.05, kff=1.0, kh=1.0, kes=2.0, v0=5.0, us=0.0,
    tl=0.05,                                      # speed-plan anticipation (s)
    kpa=0.6, kpb=2.5, db=0.3, a_sw=1.5, cd=0.0013, k_brake=47.3, mu_bc=1.9,   # throttle gain; brake: accel-domain gains
    uy_c=0.95, ca=0.0048,                         # grip model used for the friction circle in the controller
    ax_k=1.05,                                    # usable traction (g-multiple of load) under power
    circ_floor=0.12,
    tcs_slip=2.5, tcs_k=0.25,
    abs_slip=0.18,
    rpm_up=18200.0, rpm_dn=17000.0,
    v_scale=1.0,
    off_v=14.0,
)

class Racer:
    def __init__(self, D, P=None):
        self.P = dict(DEFAULT); self.P.update(P or {})
        self.N = len(D['cx']); self.ds = D['ds']; self.half = D.get('half', 6.0)
        self.cx = D['cx']; self.cy = D['cy']
        self.hx = [math.cos(h) for h in D['hd']]; self.hy = [math.sin(h) for h in D['hd']]
        self.off = D['off']; self.v = D['v']; self.k = D['k']; self.ax = D['ax']
        self.px = [self.cx[i] - self.hy[i]*self.off[i] for i in range(self.N)]
        self.py = [self.cy[i] + self.hx[i]*self.off[i] for i in range(self.N)]
        self.lx = []; self.ly = []
        for i in range(self.N):
            j = (i + 1) % self.N; ddx = self.px[j] - self.px[i]; ddy = self.py[j] - self.py[i]; n = math.hypot(ddx, ddy) or 1.0
            self.lx.append(ddx / n); self.ly.append(ddy / n)
        self.abs = 1.0; self.thr = 0.0; self.gear = 1; self.since_shift = 99
        self.max_lat_err = 0.0; self.max_tp = 0.0; self.offtrack_steps = 0; self.steps = 0
        self.min_edge = 9.0

    def _lerp(self, arr, idx):
        i0 = int(math.floor(idx)) % self.N; f = idx - math.floor(idx); i1 = (i0 + 1) % self.N
        return arr[i0] + (arr[i1] - arr[i0]) * f

    def drive(self, S, R):
        P = self.P
        v = max(S['speedX'], 0.0) / 3.6
        s = S['distFromStart']; idx = s / self.ds
        y = S['trackPos'] * self.half
        ang = S['angle']
        # --- global pose of the car, reconstructed from the track table
        hx = self._lerp(self.hx, idx); hy = self._lerp(self.hy, idx)
        nrm = math.hypot(hx, hy); hx /= nrm; hy /= nrm
        X = self._lerp(self.cx, idx) - hy * y; Y = self._lerp(self.cy, idx) + hx * y
        ca_, sa_ = math.cos(ang), math.sin(ang)
        yx = hx * ca_ + hy * sa_; yy = hy * ca_ - hx * sa_          # car heading unit vector
        # --- pure pursuit on the racing line
        lat_err0 = self._lerp(self.off, idx) - y
        Ld = min(max(P['la0'] + P['la1'] * v, P['la_min']) + P['merge'] * max(0.0, abs(lat_err0) - P['merge_db']), P['la_max'])
        j = idx + Ld / self.ds
        dx = self._lerp(self.px, j) - X; dy = self._lerp(self.py, j) - Y
        xl = dx * yx + dy * yy; yl = -dx * yy + dy * yx
        dist = math.hypot(xl, yl)
        alpha = math.atan2(yl, xl)
        delta = math.atan2(2.0 * WHEELBASE * math.sin(alpha), max(dist, 1.0))
        lat_err = self._lerp(self.off, idx) - y
        if P['mode'] == 'pp':
            steer = delta / STEER_LOCK + P['ke'] * lat_err / (1.0 + v / P['ke_v'])
        else:
            jp = idx + (P['pv0'] + P['pv1'] * v) / self.ds
            lxx = self._lerp(self.lx, jp); lyy = self._lerp(self.ly, jp)
            psi = math.atan2(yx * lyy - yy * lxx, yx * lxx + yy * lyy)
            kff = self._lerp(self.k, idx + (P['kv0'] + P['kv1'] * v) / self.ds)
            d_ff = math.atan(WHEELBASE * kff) * (1.0 + P['us'] * v * v)
            steer = (P['kff'] * d_ff + P['kh'] * psi + math.atan2(P['kes'] * lat_err, v + P['v0'])) / STEER_LOCK
        steer = max(-1.0, min(1.0, steer))
        # --- speed plan
        vt = self._lerp(self.v, idx + v * P['tl'] / self.ds) * P['v_scale']
        off_track = abs(S['trackPos']) > 1.0
        if off_track:
            vt = min(vt, P['off_v'])
        kap = abs(self._lerp(self.k, idx))
        an = G + P['ca'] * v * v
        aymax = P['uy_c'] * 1.76 * (0.8 + 0.8 * math.exp(-1.386294 * an / G / 1.2)) * an
        fr = min(1.0, v * v * kap / aymax)
        circ = max(P['circ_floor'], math.sqrt(1.0 - fr * fr))
        w = S['wheelSpinVel']
        vr = 0.5 * (w[2] + w[3]) * R_REAR; vf = 0.5 * (w[0] + w[1]) * R_FRONT
        err = vt - v
        a_ff = self._lerp(self.ax, idx + v * P['tl'] / self.ds)
        accel = 0.0; brake = 0.0
        a_des = a_ff + P['kpb'] * err
        if off_track:
            a_des = min(a_des, P['kpb'] * err)
        if err > -P['db'] and a_des > -P['a_sw']:
            want = max(0.0, min(1.0, P['kpa'] * (err + P['db'])))
            # traction available under power vs what the engine can deliver in this gear
            g = max(1, int(S['gear']))
            a_eng = torque(S['rpm']) * RATIOS[min(g, 7)] * DIFF * 0.92 / R_REAR / MASS
            a_ok = P['ax_k'] * (G + P['ca'] * v * v) * circ
            cap = min(1.0, a_ok / max(a_eng, 1.0))
            want = min(want, cap)
            slip = vr - max(v, vf)
            if slip > P['tcs_slip']:
                want -= P['tcs_k'] * (slip - P['tcs_slip'])
            want = max(0.0, want)
            self.thr = min(want, self.thr + 0.25)
            accel = self.thr
            self.abs = min(1.0, self.abs + 0.1)
        else:
            self.thr = 0.0
            dec = max(0.0, -a_des - P['cd'] * v * v)
            dec = min(dec, P['mu_bc'] * an * (circ if fr > 0.3 else 1.0))
            want = min(1.0, dec / P['k_brake'])
            if v > 4.0:
                wmin = min(w[0] * R_FRONT, w[1] * R_FRONT, w[2] * R_REAR, w[3] * R_REAR)
                if (v - wmin) / v > P['abs_slip']:
                    self.abs = max(0.2, self.abs * 0.8)
                else:
                    self.abs = min(1.0, self.abs + 0.05)
            brake = want * self.abs
        # --- gears
        g = int(S['gear']); self.since_shift += 1
        ng = g if g >= 1 else 1
        if g >= 1 and self.since_shift > 8:
            if S['rpm'] > P['rpm_up'] and g < 7:
                ng = g + 1
            elif g > 1:
                rpm_low = max(v, vr) / R_REAR * RATIOS[g-1] * DIFF * 30.0 / math.pi
                if rpm_low < P['rpm_dn']:
                    ng = g - 1
            if ng != g: self.since_shift = 0
        R['steer'] = steer; R['accel'] = accel; R['brake'] = brake; R['gear'] = ng; R['clutch'] = 0.0
        # --- bookkeeping for analysis
        self.steps += 1
        if S.get('curLapTime', 0) > 0:
            self.max_lat_err = max(self.max_lat_err, abs(lat_err)); self.max_tp = max(self.max_tp, abs(S['trackPos']))
            if off_track: self.offtrack_steps += 1
        return vt, lat_err, fr
