"""M3/M4 return planner: from one ball state decide when/where to hit and how to move the paddle.

Pure numpy (no ROS). Steps:
  1. predict the flight (ball_predictor) and find the first bounce on the robot half;
  2. among post-bounce samples inside the reachable box, pick the hit point;
  3. find the outgoing velocity that lands the ball at the target (fixed elevation, speed by bisection);
  4. paddle normal / speed so that the rubber contact model (restitution, friction, as in the
     BallPhysics plugin) turns the incoming velocity into the wanted one.
"""
from dataclasses import dataclass, field, replace

import numpy as np

from tt_motion.ball_predictor import BallModel, fast_flight, fast_landing, surface_bounce


@dataclass
class ReturnParams:
    box_x: tuple = (1.0, 1.4)
    box_y: tuple = (-0.7, 0.7)
    box_z: tuple = (0.85, 1.15)
    min_lead: float = 0.9          # earliest hit time after the observation [s] (arm preparation)
    z_pref: float = 0.98           # preferred hit height [m]
    target: tuple = (-0.7, 0.0)    # landing point on the opponent half
    elevations_deg: tuple = (25.0, 30.0, 20.0, 35.0)
    speed_range: tuple = (2.0, 9.0)
    min_paddle_speed: float = 0.3
    paddle_restitution: float = 0.85
    paddle_friction: float = 0.45
    ball_radius: float = 0.02
    paddle_half_thickness: float = 0.006
    net_x: float = 0.0
    net_top: float = 0.76 + 0.1525
    net_margin: float = 0.03
    flight_time: float = 1.6


@dataclass
class ReturnPlan:
    t_hit: float                   # seconds after the observation
    ball_pos: np.ndarray           # ball centre at impact
    v_in: np.ndarray
    w_in: np.ndarray
    paddle_pos: np.ndarray         # paddle origin at impact
    paddle_normal: np.ndarray
    paddle_speed: float
    v_out: np.ndarray              # expected outgoing velocity (contact model)
    w_out: np.ndarray
    landing: np.ndarray            # predicted first bounce of the returned ball
    elevation_deg: float
    target: tuple = (-0.7, 0.0)    # landing point that was aimed at
    stroke_type: int = 0           # StrokePlan.TOPSPIN_DRIVE / LOOP / BLOCK
    info: dict = field(default_factory=dict)


def first_landing(p, v, w, model):
    """(first table bounce position, ball height over the net line) or (None, None)."""
    pos, clearance = fast_landing(p, v, w, model)
    return (pos, clearance) if pos is not None else (None, None)


def contact(v_in, w_in, n, s, params, model):
    """Outgoing velocity/spin when the paddle (normal n, moving s*n) meets the ball."""
    v_rel = np.asarray(v_in, float) - s * np.asarray(n, float)
    m = BallModel(radius=params.ball_radius)
    v2, w2 = surface_bounce(m, params.paddle_restitution, params.paddle_friction, n, v_rel,
                            np.asarray(w_in, float))
    return v2 + s * n, w2


def paddle_for(v_in, w_in, v_want, params, model):
    """Normal and speed so that the contact model gives ~v_want (fixed-point on the friction error)."""
    v_cmd = np.asarray(v_want, float).copy()
    best = None
    for _ in range(8):
        d = v_cmd - v_in
        k = np.linalg.norm(d)
        n = d / k
        s = max(float(np.dot(v_in, n)) + k / (1.0 + params.paddle_restitution),
                params.min_paddle_speed)
        v_out, w_out = contact(v_in, w_in, n, s, params, model)
        err = v_want - v_out
        best = (n, s, v_out, w_out)
        if np.linalg.norm(err) < 0.02:
            break
        v_cmd = v_cmd + err
    return best


def solve_outgoing(hit_pos, target, params, model):
    """(velocity, elevation_deg, landing, clearance) of a return landing near target, or None."""
    yaw = np.arctan2(target[1] - hit_pos[1], target[0] - hit_pos[0])
    for el_deg in params.elevations_deg:
        el = np.radians(el_deg)
        d = np.array([np.cos(el) * np.cos(yaw), np.cos(el) * np.sin(yaw), np.sin(el)])
        lo, hi = params.speed_range
        reach = lambda s: first_landing(hit_pos, s * d, np.zeros(3), model)  # noqa: E731
        for _ in range(14):
            mid = 0.5 * (lo + hi)
            land, _c = reach(mid)
            dist = np.inf if land is None else np.hypot(land[0] - hit_pos[0], land[1] - hit_pos[1])
            want = np.hypot(target[0] - hit_pos[0], target[1] - hit_pos[1])
            if dist < want:
                lo = mid
            else:
                hi = mid
        v = 0.5 * (lo + hi) * d
        land, clear = reach(0.5 * (lo + hi))
        if land is None or clear is None:
            continue
        if np.hypot(land[0] - target[0], land[1] - target[1]) > 0.15:
            continue
        if clear < params.net_top + params.ball_radius + params.net_margin:
            continue
        return v, el_deg, land, clear
    return None


def _plan_hit(hit_ball, v_in, w_in, params, model):
    """Outgoing velocity + paddle for one hit toward params.target, or None."""
    # The paddle's friction adds spin and shifts the landing: aim again at a corrected point.
    aim = np.array(params.target, float)
    best, best_err = None, np.inf
    for _ in range(8):
        sol = solve_outgoing(hit_ball, aim, params, model)
        if sol is None:
            break
        v_want, el, land, clear = sol
        n, s, v_out, w_out = paddle_for(v_in, w_in, v_want, params, model)
        land2, clear2 = first_landing(hit_ball, v_out, w_out, model)
        if land2 is None:
            d = hit_ball[:2] - aim                      # flew past the table: aim shorter
            aim = aim + 0.25 * d / np.linalg.norm(d)
            continue
        err = np.array(params.target) - land2[:2]
        e = float(np.linalg.norm(err))
        if clear2 is not None and clear2 > params.net_top + params.ball_radius and e < best_err:
            best, best_err = (v_want, el, n, s, v_out, w_out, land2, clear2), e
        if e < 0.03:
            break
        aim = aim + 0.7 * err   # damped: the contact spin makes the landing move ~1.3x the aim
    # accept only if the corrected aim really converged (otherwise try the next candidate)
    if best is None or best_err > 0.10:
        return None
    return best


def plan_return(p, v, w, params=None, model=None, selector=None, aim_offset=(0.0, 0.0)):
    """Plan a return from the ball state (p, v, w) at observation time. Returns ReturnPlan or a
    string with the reason it cannot be returned.

    selector(hit_ball, v_in) -> ShotChoice (M4) picks target, elevations and stroke type from the
    predicted hit; without it the fixed params.target is used. If the chosen shot cannot be solved
    the planner falls back to the fixed target. aim_offset (x, y) is added to the aimed point to
    cancel a measured systematic landing error (the plan still reports the unshifted target)."""
    params = params or ReturnParams()
    model = model or BallModel()
    ts, ps, vs, ws, bounces = fast_flight(p, v, w, params.flight_time, model)
    robot_bounces = [b for b in bounces
                     if 0.0 < ps[int(np.searchsorted(ts, b))][0] < model.table_half_x]
    if not robot_bounces:
        return 'no bounce on the robot half (ball is long, wide or short)'
    tb = robot_bounces[0]
    best = None
    for i in range(len(ts)):
        if ts[i] < max(params.min_lead, tb + 0.03):
            continue
        x, y, z = ps[i]
        if not (params.box_x[0] <= x <= params.box_x[1] and params.box_y[0] <= y <= params.box_y[1]
                and params.box_z[0] <= z <= params.box_z[1]):
            continue
        score = abs(z - params.z_pref) + 0.2 * (ts[i] - params.min_lead)
        if best is None or score < best[0]:
            best = (score, i)
    if best is None:
        return 'no post-bounce point inside the reachable box after the minimum lead time'
    i = best[1]
    hit_ball, v_in, w_in, t_hit = ps[i], vs[i], ws[i], float(ts[i])
    choices = []
    if selector is not None:
        c = selector(hit_ball, v_in)
        choices.append((tuple(c.target), c.elevations, c.stroke_type, c))
    choices.append((tuple(params.target), params.elevations_deg, 0, None))
    for target, elevations, stroke_type, choice in choices:
        aim = (float(np.clip(target[0] + aim_offset[0], -1.25, -0.3)),
               float(np.clip(target[1] + aim_offset[1], -0.65, 0.65)))
        sub = replace(params, target=aim, elevations_deg=elevations)
        result = _plan_hit(hit_ball, v_in, w_in, sub, model)
        if result is not None:
            break
    if result is None:
        return 'no outgoing velocity lands on the target and clears the net'
    v_want, el, n, s, v_out, w_out, land2, clear2 = result
    paddle_pos = hit_ball - (params.ball_radius + params.paddle_half_thickness) * n
    return ReturnPlan(
        t_hit=t_hit, ball_pos=hit_ball, v_in=v_in, w_in=w_in, paddle_pos=paddle_pos,
        paddle_normal=n, paddle_speed=s, v_out=v_out, w_out=w_out,
        landing=land2, elevation_deg=el, target=target, stroke_type=stroke_type,
        info={'bounce_t': tb, 'v_want': v_want, 'net_clearance': clear2,
              'choice': choice, 'fallback': selector is not None and choice is None})
