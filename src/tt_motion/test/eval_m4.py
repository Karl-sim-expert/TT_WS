"""Offline M4 check (no ROS): random machine shots -> plan_return with the rule selector ->
independent flight simulation of the returned ball -> landing error and net clearance.

  python3 src/tt_motion/test/eval_m4.py [N] [speed] [seed]
"""
import importlib.util
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from tt_motion.ball_predictor import simulate  # noqa: E402
from tt_motion.return_planner import ReturnParams, plan_return  # noqa: E402
from tt_motion.shot_selector import NAMES, select_shot  # noqa: E402

NET_TOP = 0.76 + 0.1525


def machine():
    path = os.path.join(HERE, '..', '..', 'tt_bringup', 'scripts', 'ball_machine.py')
    spec = importlib.util.spec_from_file_location('ball_machine', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def check_return(plan):
    """Independent check of the returned ball: (landing error [m], net clearance [m]) or None."""
    ts, ps, vs, ws, bounces = simulate(plan.ball_pos, plan.v_out, plan.w_out, 1.2)
    if not bounces:
        return None
    i = int(np.searchsorted(ts, bounces[0]))
    land = ps[min(i, len(ps) - 1)]
    err = float(np.hypot(*(land[:2] - np.array(plan.target))))
    net = None
    for k in range(1, i + 1):
        if ps[k - 1][0] > 0.0 >= ps[k][0]:
            net = float(ps[k][2] - 0.02 - NET_TOP)
    return err, net


def run(n=20, speed=6.5, seed=1, spin_max=0.0, selector=select_shot, verbose=True):
    bm = machine()
    rnd = random.Random(seed)
    rows = []
    for _ in range(n):
        tx, ty = rnd.uniform(0.6, 1.25), rnd.uniform(-0.7, 0.7)
        sol = bm.solve(speed, tx, ty, rnd.uniform(-1, 1) * spin_max, rnd.uniform(-1, 1) * spin_max / 2)
        if sol is None:
            continue
        th, ph, _ = sol
        v, w = bm.launch_velocity(speed, th, ph, 0.0, 0.0)
        plan = plan_return(bm.LAUNCH_POS, v, [0, 0, 0], ReturnParams(min_lead=0.6), selector=selector)
        if isinstance(plan, str):
            rows.append((tx, ty, None, plan))
            continue
        res = check_return(plan)
        ch = plan.info['choice']
        rows.append((tx, ty, plan, res))
        if verbose:
            err, net = res if res else (float('nan'), None)
            print(f'ball@({tx:.2f},{ty:+.2f}) -> {NAMES[plan.stroke_type]:5s} '
                  f'target ({plan.target[0]:+.2f},{plan.target[1]:+.2f}) hit z={plan.ball_pos[2]:.2f} '
                  f'paddle {plan.paddle_speed:.1f} m/s  err {100 * err:.1f} cm  net +{100 * (net or 0):.0f} cm'
                  + (f'  [{ch.reason}]' if ch else '  [fallback]'))
    return rows


if __name__ == '__main__':
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    sp = float(sys.argv[2]) if len(sys.argv) > 2 else 6.5
    sd = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    rows = run(n, sp, sd)
    ok = [r for r in rows if r[2] is not None and r[3]]
    errs = [r[3][0] for r in ok]
    bad_net = sum(1 for r in ok if r[3][1] is None or r[3][1] <= 0)
    if not ok:
        sys.exit('no shot could be planned (machine speed too high?)')
    print(f'planned {len(ok)}/{len(rows)}; error mean {100 * np.mean(errs):.1f} cm, '
          f'max {100 * max(errs):.1f} cm; > 15 cm: {sum(e > 0.15 for e in errs)}; net failures: {bad_net}')
    from collections import Counter
    print(Counter(NAMES[r[2].stroke_type] for r in ok))
