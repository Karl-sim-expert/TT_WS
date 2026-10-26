"""Ball flight prediction (M3): drag + Magnus + table bounce, same model as tt_gazebo/ball_dynamics.hpp.

Pure numpy, no ROS, so it can be unit-tested offline. World frame: x along the table (net at 0,
robot side x > 0), z up, table top at z = 0.76.
"""
from dataclasses import dataclass

import numpy as np

G = np.array([0.0, 0.0, -9.81])


@dataclass
class BallModel:
    mass: float = 0.0027
    radius: float = 0.02
    air_density: float = 1.2
    drag_coeff: float = 0.40
    lift_slope: float = 1.0
    lift_max: float = 0.6
    restitution: float = 0.9
    friction: float = 0.25
    table_z: float = 0.76
    table_half_x: float = 1.37
    table_half_y: float = 0.7625


def acceleration(m, v, w):
    speed = np.linalg.norm(v)
    a = G.copy()
    if speed < 1e-6:
        return a
    area = np.pi * m.radius ** 2
    f = -0.5 * m.air_density * m.drag_coeff * area * speed * v
    wxv = np.cross(w, v)
    n = np.linalg.norm(wxv)
    if n > 1e-9:
        cl = min(m.lift_slope * m.radius * np.linalg.norm(w) / speed, m.lift_max)
        f = f + 0.5 * m.air_density * cl * area * speed ** 2 / n * wxv
    return a + f / m.mass


def surface_bounce(m, e, mu, n, v, w):
    vn = float(np.dot(v, n))
    if vn >= 0.0:
        return v, w
    vt = v - vn * n
    u = vt + m.radius * np.cross(n, w)
    un = np.linalg.norm(u)
    dv = min(mu * (1.0 + e) * abs(vn), un / 2.5)
    dvt = -dv / un * u if un > 1e-9 else np.zeros(3)
    return vt + dvt - e * vn * n, w - 3.0 / (2.0 * m.radius) * np.cross(n, dvt)


def _rk4(m, p, v, w, dt):
    k1v = acceleration(m, v, w)
    k2v = acceleration(m, v + 0.5 * dt * k1v, w)
    k3v = acceleration(m, v + 0.5 * dt * k2v, w)
    k4v = acceleration(m, v + dt * k3v, w)
    vn = v + dt / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
    return p + 0.5 * dt * (v + vn), vn


def simulate(p0, v0, w0, t_end, model=None, dt=0.002):
    """Integrate until t_end. Returns arrays (t, pos, vel, w) and a list of bounce times."""
    m = model or BallModel()
    p, v, w = (np.array(a, float) for a in (p0, v0, w0))
    ts, ps, vs, ws, bounces = [0.0], [p.copy()], [v.copy()], [w.copy()], []
    t = 0.0
    while t < t_end - 1e-12:
        h = min(dt, t_end - t)
        pn, vn = _rk4(m, p, v, w, h)
        floor = m.table_z + m.radius
        if (v[2] < 0 and pn[2] <= floor and abs(pn[0]) <= m.table_half_x
                and abs(pn[1]) <= m.table_half_y):
            frac = (p[2] - floor) / max(p[2] - pn[2], 1e-12)
            frac = min(max(frac, 0.0), 1.0)
            pc, vc = _rk4(m, p, v, w, h * frac)
            vc, w = surface_bounce(m, m.restitution, m.friction, np.array([0, 0, 1.0]), vc, w)
            pc[2] = floor
            rest = h * (1.0 - frac)
            pn, vn = _rk4(m, pc, vc, w, rest) if rest > 0 else (pc, vc)
            bounces.append(t + h * frac)
        p, v, t = pn, vn, t + h
        ts.append(t); ps.append(p.copy()); vs.append(v.copy()); ws.append(w.copy())
    return np.array(ts), np.array(ps), np.array(vs), np.array(ws), bounces


def predict_plane_crossing(p0, v0, w0, x_plane, model=None, t_max=2.0, dt=0.002):
    """First time the ball crosses x = x_plane moving toward +x. Returns (t, p, v, w, bounces) or None."""
    ts, ps, vs, ws, bounces = simulate(p0, v0, w0, t_max, model, dt)
    for i in range(1, len(ts)):
        if ps[i - 1][0] < x_plane <= ps[i][0]:
            f = (x_plane - ps[i - 1][0]) / (ps[i][0] - ps[i - 1][0])
            lerp = lambda a: a[i - 1] + f * (a[i] - a[i - 1])  # noqa: E731
            tc = ts[i - 1] + f * (ts[i] - ts[i - 1])
            return tc, lerp(ps), lerp(vs), lerp(ws), [b for b in bounces if b <= tc]
    return None


def fast_landing(p, v, w, model=None, dt=0.004, t_max=1.2):
    """Plain-float version of simulate() that stops at the first table bounce (planner inner loop).

    Returns (landing position, height when crossing x = 0 going -x or None) or (None, None).
    """
    m = model or BallModel()
    px, py, pz = (float(a) for a in p)
    vx, vy, vz = (float(a) for a in v)
    wx, wy, wz = (float(a) for a in w)
    area = np.pi * m.radius ** 2
    kd = 0.5 * m.air_density * m.drag_coeff * area / m.mass
    kl = 0.5 * m.air_density * area / m.mass
    wn = (wx * wx + wy * wy + wz * wz) ** 0.5
    floor = m.table_z + m.radius
    net_z = None

    def acc(vx, vy, vz):
        sp = (vx * vx + vy * vy + vz * vz) ** 0.5
        if sp < 1e-6:
            return 0.0, 0.0, -9.81
        ax, ay, az = -kd * sp * vx, -kd * sp * vy, -kd * sp * vz - 9.81
        cx, cy, cz = wy * vz - wz * vy, wz * vx - wx * vz, wx * vy - wy * vx
        cn = (cx * cx + cy * cy + cz * cz) ** 0.5
        if cn > 1e-9:
            f = kl * min(m.lift_slope * m.radius * wn / sp, m.lift_max) * sp * sp / cn
            ax, ay, az = ax + f * cx, ay + f * cy, az + f * cz
        return ax, ay, az

    t = 0.0
    while t < t_max:
        ax, ay, az = acc(vx, vy, vz)
        mx, my, mz = vx + 0.5 * dt * ax, vy + 0.5 * dt * ay, vz + 0.5 * dt * az
        bx, by, bz = acc(mx, my, mz)
        npx, npy, npz = px + dt * mx, py + dt * my, pz + dt * mz
        vx, vy, vz = vx + dt * bx, vy + dt * by, vz + dt * bz
        if px > 0.0 >= npx and net_z is None:
            net_z = pz + (npz - pz) * (px / (px - npx))
        if vz < 0 and npz <= floor and abs(npx) <= m.table_half_x and abs(npy) <= m.table_half_y:
            f = (pz - floor) / max(pz - npz, 1e-12)
            return np.array([px + f * (npx - px), py + f * (npy - py), floor]), net_z
        px, py, pz = npx, npy, npz
        t += dt
    return None, net_z
