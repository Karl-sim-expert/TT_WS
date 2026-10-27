import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from tt_motion.ball_predictor import simulate  # noqa: E402
from tt_motion.kinematics import Chain  # noqa: E402
from tt_motion.return_planner import ReturnParams, plan_return  # noqa: E402
from tt_motion.shot_selector import BLOCK, LOOP, TOPSPIN_DRIVE, select_shot  # noqa: E402
from tt_motion.stroke import StrokeParams, build_stroke, paddle_rotation_for  # noqa: E402
from urdf_utils import processed_urdf  # noqa: E402

Q_READY = np.array([0.0, 0.0, -0.058, 0.132, 0.351, -2.048, -0.631, 1.803, 1.559])

# A machine shot: from (-1.8, 0, 1.0), lands on the robot half.
SHOTS = [([-1.8, 0, 1.0], [5.342, 0.594, 2.667]), ([-1.8, 0, 1.0], [5.8, 0.0, 2.4]),
         ([-1.8, 0, 1.0], [6.5, -0.8, 2.2])]


class ReturnPlannerTest(unittest.TestCase):
    def test_plans_and_lands_on_opponent_half(self):
        found = 0
        for p, v in SHOTS:
            plan = plan_return(p, v, [0, 0, 0], ReturnParams(min_lead=0.6))
            if isinstance(plan, str):
                continue
            found += 1
            self.assertGreater(plan.paddle_normal[2], -0.2)
            self.assertLess(plan.paddle_normal[0], 0.0)      # swings toward the opponent
            self.assertLess(plan.landing[0], 0.0)
            self.assertLess(np.hypot(*(plan.landing[:2] - np.array([-0.7, 0.0]))), 0.15)
            self.assertGreater(plan.info['net_clearance'], 0.76 + 0.1525)
        self.assertEqual(found, len(SHOTS))

    def test_rejects_ball_that_misses_the_table(self):
        r = plan_return([-1.8, 0, 1.0], [8.0, 0.0, 3.0], [0, 0, 0])
        self.assertIsInstance(r, str)

    def test_arm_can_execute_most_plans(self):
        chain = Chain(processed_urdf())
        chain.set_weight_by_name('rail_joint', 0.1)
        chain.set_weight_by_name('rail_y_joint', 0.1)
        sp = StrokeParams(d_back=0.08, d_follow=0.10, min_move_time=0.2, prep_vel_frac=0.9)
        ok = 0
        for p, v in SHOTS:
            plan = plan_return(p, v, [0, 0, 0], ReturnParams(min_lead=0.6))
            rot = paddle_rotation_for(chain, Q_READY, plan.paddle_normal)
            try:
                build_stroke(chain, Q_READY, Q_READY, plan.paddle_pos, rot,
                             plan.paddle_speed * plan.paddle_normal, plan.t_hit - 0.16, sp)
                ok += 1
            except Exception:  # noqa: BLE001  (StrokeError: not reachable / too soon)
                pass
        self.assertGreaterEqual(ok, 2)


class ShotSelectorTest(unittest.TestCase):
    def test_rules(self):
        fast = select_shot([1.2, 0.0, 0.98], [3.9, 0.0, 1.0])
        self.assertEqual(fast.stroke_type, BLOCK)
        low = select_shot([1.2, 0.0, 0.90], [3.0, 0.0, 1.0])
        self.assertEqual(low.stroke_type, LOOP)
        steep = select_shot([1.2, 0.0, 0.98], [2.6, 0.0, 2.2])
        self.assertEqual(steep.stroke_type, LOOP)
        mid = select_shot([1.2, 0.0, 0.98], [3.0, 0.0, 1.0])
        self.assertEqual(mid.stroke_type, TOPSPIN_DRIVE)
        high = select_shot([1.2, 0.0, 1.10], [3.0, 0.0, 0.0])
        self.assertLess(high.target[0], mid.target[0])        # deeper than the standard drive
        self.assertLess(fast.target[0], 0.0)

    def test_cross_court_and_inside_the_table(self):
        left = select_shot([1.2, 0.5, 0.98], [3.0, 0.0, 1.0])
        right = select_shot([1.2, -0.5, 0.98], [3.0, 0.0, 1.0])
        self.assertLess(left.target[1], 0.0)
        self.assertGreater(right.target[1], 0.0)
        for y in np.linspace(-0.7, 0.7, 15):
            t = select_shot([1.2, y, 0.98], [3.0, 0.0, 1.0]).target
            self.assertLessEqual(abs(t[1]), 0.55)
            self.assertGreater(t[0], -1.37 + 0.25)

    def test_planner_hits_selected_targets(self):
        got = set()
        for p, v in SHOTS:
            plan = plan_return(p, v, [0, 0, 0], ReturnParams(min_lead=0.6), selector=select_shot)
            self.assertNotIsInstance(plan, str)
            got.add(plan.stroke_type)
            ts, ps, _, _, bounces = simulate(plan.ball_pos, plan.v_out, plan.w_out, 1.2)
            i = int(np.searchsorted(ts, bounces[0]))
            self.assertLess(np.hypot(*(ps[i][:2] - np.array(plan.target))), 0.15)
            self.assertGreater(plan.info['net_clearance'], 0.76 + 0.1525)
        self.assertTrue(got <= {TOPSPIN_DRIVE, LOOP, BLOCK})

    def test_falls_back_to_fixed_target(self):
        bad = lambda hit, v: types_choice((-5.0, 0.0))  # noqa: E731  (off the table: unsolvable)
        plan = plan_return(SHOTS[0][0], SHOTS[0][1], [0, 0, 0], ReturnParams(min_lead=0.6), selector=bad)
        self.assertNotIsInstance(plan, str)
        self.assertTrue(plan.info['fallback'])
        self.assertEqual(plan.target, (-0.7, 0.0))


def types_choice(target):
    from tt_motion.shot_selector import ShotChoice
    return ShotChoice(BLOCK, target, (25.0,), 'test')


if __name__ == '__main__':
    unittest.main()
