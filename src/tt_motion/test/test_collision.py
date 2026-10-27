import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from tt_motion.collision import TableGuard  # noqa: E402
from tt_motion.kinematics import Chain  # noqa: E402
from tt_motion.return_planner import ReturnParams, plan_return  # noqa: E402
from tt_motion.stroke import StrokeError, StrokeParams, build_stroke, paddle_rotation_for  # noqa: E402
from urdf_utils import processed_urdf  # noqa: E402

Q_READY = np.array([0.0, 0.0, -0.058, 0.132, 0.351, -2.048, -0.631, 1.803, 1.559])


class TableGuardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.chain = Chain(processed_urdf())
        cls.chain.set_weight_by_name('rail_joint', 0.1)
        cls.chain.set_weight_by_name('rail_y_joint', 0.1)
        cls.guard = TableGuard(cls.chain)

    def test_ready_pose_is_clear(self):
        self.assertIsNone(self.guard.check(Q_READY))

    def test_rail_cannot_enter_the_table_footprint(self):
        self.assertGreaterEqual(self.chain.lower[1], 0.0)   # rail_joint: carriage stays behind the end line
        q = Q_READY.copy()
        q[1] = -0.4                                         # outside the real limit: carriage under the table
        self.assertIn('rail carriage', self.guard.check(q))

    def test_paddle_below_table_top_is_rejected(self):
        hit = np.array([1.2, 0.0, 0.70])                    # paddle disc centre below the surface
        rot = paddle_rotation_for(self.chain, Q_READY, np.array([-1.0, 0.0, 0.0]))
        t = np.eye(4)
        t[:3, :3], t[:3, 3] = rot, hit
        q, ok = self.chain.ik(t, Q_READY, q_pref=Q_READY)
        if ok:
            self.assertIn('inside the table', self.guard.check(q))

    def test_stroke_through_the_table_is_rejected(self):
        # a low swing line at table height has to be refused (or be unreachable), never executed
        rot = paddle_rotation_for(self.chain, Q_READY, np.array([-1.0, 0.0, 0.0]))
        with self.assertRaises(StrokeError):
            build_stroke(self.chain, Q_READY, Q_READY, np.array([1.25, 0.0, 0.74]), rot,
                         np.array([-1.5, 0.0, 0.0]), 2.0, StrokeParams(d_back=0.08, d_follow=0.10))

    def test_planner_skips_a_vetoed_hit_point(self):
        p, v = [-1.8, 0, 1.0], [5.8, 0.0, 2.4]
        first = plan_return(p, v, [0, 0, 0], ReturnParams(min_lead=0.6))
        seen = []

        def veto_first(plan):
            seen.append(plan.t_hit)
            return 'vetoed' if len(seen) == 1 else None
        second = plan_return(p, v, [0, 0, 0], ReturnParams(min_lead=0.6), check=veto_first)
        self.assertNotIsInstance(second, str)
        self.assertEqual(seen[0], first.t_hit)
        self.assertNotAlmostEqual(second.t_hit, first.t_hit)
        reason = plan_return(p, v, [0, 0, 0], ReturnParams(min_lead=0.6), check=lambda pl: 'no')
        self.assertIsInstance(reason, str)


if __name__ == '__main__':
    unittest.main()
