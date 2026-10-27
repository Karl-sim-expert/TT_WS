"""Stroke trajectory generation (design doc section 5): ready -> backswing -> hit -> follow-through.

The paddle moves on a straight line along the hit velocity direction u with orientation held
constant. Speed along the line follows smoothstep ramps that end exactly at the hit speed s at
the hit time t*:

    accel:  v(t) = s S(tau/T1),      T1 = 2 d_back / s,    S(u) = 10u^3 - 15u^4 + 6u^5
    decel:  v(t) = s (1 - S(tau/T2)), T2 = 2 d_follow / s

(S has zero slope at both ends, so the integral over a ramp is s T / 2, hence T = 2 d / s.)
Every sample is turned into joint space with IK (seeded by its neighbour) and the joint velocity
comes from the Jacobian pseudo-inverse: qdot = J+ [v; 0]. If a joint velocity limit would be
exceeded the hit speed is reduced and the stroke rebuilt.
"""
from dataclasses import dataclass, field

import numpy as np

from tt_motion.collision import TableGuard
from tt_motion.kinematics import rotation_aligning

PEAK_SMOOTH = 1.875  # max of S'(u)


def smoothstep5(u):
    u = np.clip(u, 0.0, 1.0)
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


def smoothstep5_integral(u):
    u = np.clip(u, 0.0, 1.0)
    return 2.5 * u ** 4 - 3 * u ** 5 + u ** 6


def smoothstep5_rate(u):
    u = np.clip(u, 0.0, 1.0)
    return 30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4


def paddle_rotation_for(chain, q_ready, normal):
    """Paddle orientation with local +z = normal, closest to the ready orientation (the disc is
    symmetric about its normal, so the roll is free: keep the one nearest to the ready pose)."""
    r_ready = chain.fk(q_ready)[:3, :3]
    return rotation_aligning(r_ready[:, 2], normal) @ r_ready


@dataclass
class StrokeParams:
    d_back: float = 0.20          # minimum backswing distance before the hit [m]
    d_follow: float = 0.15        # minimum follow-through distance after the hit [m]
    min_ramp: float = 0.15        # shortest accel / decel ramp [s]: distance grows as s * ramp / 2
    sample_dt: float = 0.005      # swing sampling period [s]
    prep_dt: float = 0.02         # sampling period of the slow moves [s]
    prep_vel_frac: float = 0.5    # slow moves use this fraction of the joint velocity limit
    min_move_time: float = 0.5    # shortest slow move [s]
    lead_margin: float = 0.05     # idle time kept between the preparation and the backswing [s]
    vel_margin: float = 0.95      # swing may use this fraction of the joint velocity limit
    vel_scale: float = 1.0        # multiplies the URDF joint velocity limits (sim-only 'virtual arm')
    return_pause: float = 0.1     # pause at the end of the follow-through before returning [s]
    grid_step: float = 0.01       # spacing of the IK grid along the swing line [m]
    max_retries: int = 4
    table_guard: bool = True      # reject strokes in which the arm / rail would enter the table


@dataclass
class Stroke:
    times: np.ndarray             # [N] s since the start of the plan (strictly increasing)
    q: np.ndarray                 # [N, n]
    qd: np.ndarray                # [N, n]
    hit_speed: float              # speed actually used (<= requested)
    t_hit: float
    min_lead: float               # shortest hit time (since plan start) this stroke could meet
    max_joint_vel_ratio: float    # peak |qdot| / limit during the swing
    info: dict = field(default_factory=dict)


class StrokeError(Exception):
    pass


def _quintic_move(q0, q1, t0, duration, dt):
    """Smoothstep joint move (zero velocity at both ends); returns times, q, qd (excluding t0)."""
    n = max(2, int(np.ceil(duration / dt)))
    u = np.linspace(0.0, 1.0, n + 1)[1:]
    s = smoothstep5(u)[:, None]
    rate = (smoothstep5_rate(u)[:, None]) / duration
    dq = (q1 - q0)[None, :]
    return t0 + u * duration, q0[None, :] + s * dq, rate * dq


def _move_time(q0, q1, vlim, params):  # vlim: effective limits (already scaled)
    ratio = np.max(np.abs(q1 - q0) / vlim)
    return max(params.min_move_time, PEAK_SMOOTH * ratio / params.prep_vel_frac)


def build_stroke(chain, q_start, q_ready, hit_pos, hit_rot, hit_vel, t_hit, params=None):
    """Joint trajectory that brings the paddle to hit_pos with orientation hit_rot and velocity
    hit_vel (3-vector, world frame) at time t_hit (seconds since the start of the plan).

    q_start is the current joint state (the arm is assumed at rest); the arm returns to q_ready
    after the follow-through.
    """
    params = params or StrokeParams()
    vlim = chain.vel_limit * params.vel_scale
    q_start = np.asarray(q_start, float)
    q_ready = np.asarray(q_ready, float)
    hit_pos = np.asarray(hit_pos, float)
    hit_vel = np.asarray(hit_vel, float)
    speed = float(np.linalg.norm(hit_vel))
    if speed < 1e-3:
        raise StrokeError('hit speed is zero')
    u = hit_vel / speed

    def pose_at(x):
        t = np.eye(4)
        t[:3, :3] = hit_rot
        t[:3, 3] = hit_pos + x * u
        return t

    # IK on a fixed grid along the swing line, continuing outwards from the hit point (seeded by
    # the ready pose). The swing samples interpolate this grid, so a retry costs no extra IK.
    q_hit, ok = chain.ik(pose_at(0.0), q_ready, q_pref=q_ready)
    if not ok:
        raise StrokeError('hit pose is not reachable (IK failed at the hit point)')

    def grid_solutions(xs):
        """Solutions along xs, stopping at the first unreachable point (the arm's reach limit)."""
        out = []
        q_prev = q_hit
        for x in xs:
            q_i, ok_i = chain.ik(pose_at(x), q_prev, q_pref=q_ready)
            if not ok_i:
                break
            out.append(q_i)
            q_prev = q_i
        return out

    def distances(s):
        # Faster hits need longer ramps (bounded acceleration): d = max(d_min, s * ramp / 2).
        return (max(params.d_back, 0.5 * s * params.min_ramp),
                max(params.d_follow, 0.5 * s * params.min_ramp))

    d_back_max, d_follow_max = distances(speed)
    nb = int(np.ceil(d_back_max / params.grid_step))
    nf = int(np.ceil(d_follow_max / params.grid_step))
    x_back = -d_back_max * np.linspace(0.0, 1.0, nb + 1)[1:]          # 0 -> -d_back
    x_fore = d_follow_max * np.linspace(0.0, 1.0, nf + 1)[1:]         # 0 -> +d_follow
    sol_back = grid_solutions(x_back)
    sol_fore = grid_solutions(x_fore)
    reach_back = 0.0 if not sol_back else -x_back[len(sol_back) - 1]
    reach_fore = 0.0 if not sol_fore else x_fore[len(sol_fore) - 1]
    if reach_back < params.d_back or reach_fore < params.d_follow:
        raise StrokeError(
            f'swing line leaves the workspace (reach {reach_back:.2f} m back, {reach_fore:.2f} m '
            f'forward; need {params.d_back:.2f} / {params.d_follow:.2f} m)')
    grid_x = np.concatenate([x_back[:len(sol_back)][::-1], [0.0], x_fore[:len(sol_fore)]])
    grid_q = np.array(sol_back[::-1] + [q_hit] + sol_fore)
    # The ramps must fit inside the reachable part of the line: s <= 2 d / ramp.
    speed_by_reach = 2.0 * min(reach_back, reach_fore) / params.min_ramp

    def q_at(x):
        return np.array([np.interp(x, grid_x, grid_q[:, j]) for j in range(grid_q.shape[1])])

    def swing_samples(s):
        d_back, d_follow = distances(s)
        t1 = 2.0 * d_back / s
        t2 = 2.0 * d_follow / s
        n1 = max(2, int(np.ceil(t1 / params.sample_dt)))
        n2 = max(2, int(np.ceil(t2 / params.sample_dt)))
        tau1 = np.linspace(0.0, 1.0, n1 + 1)
        tau2 = np.linspace(0.0, 1.0, n2 + 1)
        # position along the line and speed: accel ramp ends at x = 0, decel ramp starts there
        x1 = -d_back + s * t1 * smoothstep5_integral(tau1)
        v1 = s * smoothstep5(tau1)
        x2 = s * t2 * (tau2 - smoothstep5_integral(tau2))
        v2 = s * (1.0 - smoothstep5(tau2))
        x1[-1] = 0.0  # the hit sample itself (exact)
        xs = np.concatenate([x1[:-1], x2])
        vs = np.concatenate([v1[:-1], v2])
        ts = np.concatenate([tau1[:-1] * t1, t1 + tau2 * t2])  # relative to the backswing start
        q_s = np.array([q_at(x) for x in xs])
        qd_s = np.array([
            chain.joint_velocity(q_i, np.concatenate([v * u, np.zeros(3)]))
            for q_i, v in zip(q_s, vs)])
        return t1, t2, ts, q_s, qd_s

    # Joint speeds are proportional to the hit speed: scale it down once if a limit is exceeded.
    s = min(speed, speed_by_reach)
    for attempt in range(params.max_retries + 1):
        t1, t2, ts, q_swing, qd_swing = swing_samples(s)
        ratio = float(np.max(np.abs(qd_swing) / vlim))
        if ratio <= params.vel_margin:
            break
        s *= 0.98 * params.vel_margin / ratio
    else:
        raise StrokeError(
            f'joint velocity limits exceeded even at {s:.2f} m/s hit speed (ratio {ratio:.2f})')

    q_back = q_swing[0]
    t_move = _move_time(q_start, q_back, vlim, params)
    min_lead = t_move + t1 + params.lead_margin
    t_back = t_hit - t1                      # backswing start (arm at q_back, at rest)
    t_prep = t_back - t_move                 # slow move start
    if t_prep < params.lead_margin * 0.0 + 1e-9 and t_hit < min_lead:
        raise StrokeError(
            f'hit time too soon: need at least {min_lead:.2f} s of lead, got {t_hit:.2f} s '
            f'(slow move {t_move:.2f} s, backswing {t1:.2f} s, largest joint change '
            f'{np.max(np.abs(q_back - q_start)):.2f} at joint {int(np.argmax(np.abs(q_back - q_start)))}); '
            f'q_start={np.round(q_start, 2).tolist()} q_back={np.round(q_back, 2).tolist()}')

    times = [0.0]
    qs = [q_start]
    qds = [np.zeros_like(q_start)]
    if t_prep > 1e-6:
        times.append(t_prep)
        qs.append(q_start)
        qds.append(np.zeros_like(q_start))
    tm, qm, qdm = _quintic_move(q_start, q_back, t_prep, t_move, params.prep_dt)
    times.extend(tm.tolist())
    qs.extend(qm)
    qds.extend(qdm)
    # swing (skip the first sample, equal to q_back at rest)
    times.extend((t_back + ts[1:]).tolist())
    qs.extend(q_swing[1:])
    qds.extend(qd_swing[1:])
    t_end = times[-1] + params.return_pause
    times.append(t_end)
    qs.append(q_swing[-1])
    qds.append(np.zeros_like(q_start))
    t_ret = _move_time(q_swing[-1], q_ready, vlim, params)
    tr, qr, qdr = _quintic_move(q_swing[-1], q_ready, t_end, max(t_ret, 1.0), params.prep_dt)
    times.extend(tr.tolist())
    qs.extend(qr)
    qds.extend(qdr)

    times = np.array(times)
    if params.table_guard:
        bad = TableGuard(chain).first_violation(np.array(qs), step=3)
        if bad is not None:
            raise StrokeError(f'stroke would hit the table: {bad[1]} (t = {times[bad[0]]:.2f} s)')
    keep = np.concatenate([[True], np.diff(times) > 1e-6])
    return Stroke(
        times=times[keep], q=np.array(qs)[keep], qd=np.array(qds)[keep], hit_speed=s,
        t_hit=t_hit, min_lead=min_lead, max_joint_vel_ratio=ratio,
        info={'q_hit': q_hit, 'q_back': q_back, 't1': t1, 't2': t2, 'retries': attempt})
