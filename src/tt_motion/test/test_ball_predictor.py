import unittest

import numpy as np

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from tt_motion.ball_predictor import BallModel, simulate, predict_plane_crossing


class BallPredictorTest(unittest.TestCase):
    def test_free_fall_without_air(self):
        m = BallModel(air_density=0.0)
        t, p, v, w, b = simulate([3.0, 0, 2.0], [0, 0, 0], [0, 0, 0], 0.5, m)
        self.assertAlmostEqual(p[-1][2], 2.0 - 0.5 * 9.81 * 0.25, places=4)
        self.assertEqual(b, [])

    def test_bounce_restitution_and_no_spin(self):
        m = BallModel(air_density=0.0, friction=0.0)
        t, p, v, w, b = simulate([0.5, 0, 1.0], [0, 0, 0], [0, 0, 0], 0.6, m)
        self.assertEqual(len(b), 1)
        vimp = -np.sqrt(2 * 9.81 * (1.0 - 0.78))
        i = int(np.searchsorted(t, b[0])) + 1
        self.assertAlmostEqual(v[i][2], 0.9 * -vimp - 9.81 * (t[i] - b[0]), delta=0.05)

    def test_off_table_no_bounce(self):
        t, p, v, w, b = simulate([2.0, 0, 1.0], [0, 0, 0], [0, 0, 0], 0.6)
        self.assertEqual(b, [])

    def test_plane_crossing_after_bounce(self):
        r = predict_plane_crossing([-1.4, 0, 1.0], [6.0, 0, 0.5], [0, 0, 0], 0.9)
        self.assertIsNotNone(r)
        self.assertAlmostEqual(r[1][0], 0.9, places=3)
        self.assertEqual(len(r[4]), 2)  # own half, then robot half

    def test_topspin_dips_more_than_backspin(self):
        args = ([-1.4, 0, 1.0], [6.0, 0, 0.5])
        top = simulate(*args, [0, 300.0, 0], 0.6)[4]
        back = simulate(*args, [0, -300.0, 0], 0.6)[4]
        self.assertLess(top[0], back[0])  # Magnus pushes topspin down: it lands earlier


if __name__ == '__main__':
    unittest.main()
