"""Offline tests (no ROS): python3 -m unittest discover -s src/tt_motion/test"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))

from tt_motion.kinematics import Chain, matrix_to_quat, quat_to_matrix, rotation_aligning  # noqa
from tt_motion.stroke import (StrokeError, StrokeParams, build_stroke,  # noqa
                              paddle_rotation_for, smoothstep5, smoothstep5_integral)
from urdf_utils import processed_urdf  # noqa

Q_READY = np.array([0.0, 0.0, -0.058, 0.132, 0.351, -2.048, -0.631, 1.803, 1.559])
BALL_R, PADDLE_HALF = 0.02, 0.006
E_PADDLE = 0.85


def hit_setup(chain, ball, out_speed, elevation_deg=25.0, yaw_deg=0.0):
    el, yaw = np.radians(elevation_deg), np.radians(yaw_deg)
    n = np.array([-np.cos(el) * np.cos(yaw), -np.cos(el) * np.sin(yaw), np.sin(el)])
    pos = np.asarray(ball) - (BALL_R + PADDLE_HALF) * n
    return pos, paddle_rotation_for(chain, Q_READY, n), n * out_speed / (1 + E_PADDLE), n


class KinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.chain = Chain(processed_urdf())
        cls.chain.set_weight_by_name('rail_joint', 0.1)
        cls.chain.set_weight_by_name('rail_y_joint', 0.1)

    def test_chain_has_rail_and_seven_arm_joints_and_paddle_normal_faces_table(self):
        self.assertEqual(self.chain.n, 9)
        self.assertEqual(self.chain.joint_names[:2], ['rail_y_joint', 'rail_joint'])
        t = self.chain.fk(Q_READY)
        self.assertLess(t[0, 2], -0.9)            # paddle +z points toward -x (the table)
        self.assertGreater(t[0, 3], 1.0)          # paddle is in front of the base at x = 1.7

    def test_jacobian_matches_finite_differences(self):
        q = Q_READY + 0.1
        jac = self.chain.jacobian(q)
        for i in range(9):
            dq = np.zeros(9)
            dq[i] = 1e-6
            t0, t1 = self.chain.fk(q), self.chain.fk(q + dq)
            self.assertTrue(np.allclose((t1[:3, 3] - t0[:3, 3]) / 1e-6, jac[:3, i], atol=1e-4))

    def test_ik_reaches_pose(self):
        target = self.chain.fk(Q_READY + np.array([0.1, 0.1, 0.1, 0.1, -0.1, 0.1, 0.05, -0.1, 0.1]))
        q, ok = self.chain.ik(target, Q_READY)
        self.assertTrue(ok)
        self.assertLess(np.linalg.norm(self.chain.fk(q)[:3, 3] - target[:3, 3]), 5e-4)

    def test_quaternion_roundtrip(self):
        r = rotation_aligning([0, 0, 1], [1, 2, 3])
        self.assertTrue(np.allclose(quat_to_matrix(matrix_to_quat(r)), r, atol=1e-9))


class StrokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.chain = Chain(processed_urdf())
        cls.chain.set_weight_by_name('rail_joint', 0.1)
        cls.chain.set_weight_by_name('rail_y_joint', 0.1)

    def build(self, out_speed=1.0, scale=1.0, t_hit=4.0, ball=(1.15, 0.0, 0.95), **kw):
        pos, rot, v, n = hit_setup(self.chain, ball, out_speed, **kw)
        stroke = build_stroke(self.chain, Q_READY, Q_READY, pos, rot, v, t_hit,
                              StrokeParams(vel_scale=scale))
        return stroke, pos, rot, v

    def test_smoothstep_integral(self):
        self.assertAlmostEqual(float(smoothstep5(1.0)), 1.0)
        self.assertAlmostEqual(float(smoothstep5_integral(1.0)), 0.5)

    def test_paddle_arrives_at_hit_pose_with_hit_velocity_at_hit_time(self):
        stroke, pos, rot, v = self.build(out_speed=1.0)
        i = int(np.argmin(np.abs(stroke.times - stroke.t_hit)))
        self.assertAlmostEqual(stroke.times[i], stroke.t_hit, delta=1e-6)
        t = self.chain.fk(stroke.q[i])
        self.assertLess(np.linalg.norm(t[:3, 3] - pos), 1e-3)
        self.assertTrue(np.allclose(t[:3, :3], rot, atol=2e-3))
        twist = self.chain.jacobian(stroke.q[i]) @ stroke.qd[i]
        self.assertTrue(np.allclose(twist[:3], v, atol=1e-3), (twist[:3], v))
        self.assertLess(np.linalg.norm(twist[3:]), 1e-3)  # no paddle rotation at the hit

    def test_trajectory_is_consistent_and_within_limits(self):
        stroke, *_ = self.build(out_speed=1.0)
        self.assertTrue(np.all(np.diff(stroke.times) > 0))
        self.assertTrue(np.all(stroke.q >= self.chain.lower - 1e-9))
        self.assertTrue(np.all(stroke.q <= self.chain.upper + 1e-9))
        self.assertTrue(np.all(np.abs(stroke.qd) <= self.chain.vel_limit + 1e-9))
        # velocities agree with the position samples (finite differences, loose: coarse steps)
        dt = np.diff(stroke.times)[:, None]
        fd = np.diff(stroke.q, axis=0) / dt
        mid = 0.5 * (stroke.qd[1:] + stroke.qd[:-1])
        self.assertLess(np.max(np.abs(fd - mid)), 0.15)
        self.assertTrue(np.allclose(stroke.qd[0], 0) and np.allclose(stroke.qd[-1], 0))
        self.assertTrue(np.allclose(stroke.q[-1], Q_READY, atol=1e-9))   # returns to ready

    def test_hit_speed_is_reduced_to_the_joint_velocity_limits(self):
        stroke, *_ = self.build(out_speed=14.0, scale=1.0, t_hit=5.0)
        self.assertLess(stroke.hit_speed * (1 + E_PADDLE), 14.0)
        self.assertLessEqual(stroke.max_joint_vel_ratio, 0.95 + 1e-6)

    def test_rail_makes_net_clearing_speeds_possible_with_real_panda_limits(self):
        # Without the rail the Panda could only return ~1.2 m/s balls; the rail (4 m/s) carries
        # the translation, so >= 5 m/s is reachable at vel_scale = 1.
        stroke, *_ = self.build(out_speed=5.0, scale=1.0, t_hit=5.0)
        self.assertGreaterEqual(stroke.hit_speed * (1 + E_PADDLE), 4.95)

    def test_velocity_scale_raises_the_reachable_speed(self):
        slow, *_ = self.build(out_speed=14.0, scale=1.0, t_hit=5.0)
        fast, *_ = self.build(out_speed=14.0, scale=2.0, t_hit=5.0)
        self.assertGreater(fast.hit_speed, 1.5 * slow.hit_speed)

    def test_too_early_hit_time_is_rejected(self):
        with self.assertRaises(StrokeError):
            self.build(out_speed=1.0, t_hit=0.5)

    def test_unreachable_hit_is_rejected(self):
        with self.assertRaises(StrokeError):
            self.build(ball=(0.2, 0.0, 0.95))


def hermite(times, q, qd, t):
    """Cubic Hermite interpolation, like joint_trajectory_controller with positions+velocities."""
    i = int(np.clip(np.searchsorted(times, t, side='right') - 1, 0, len(times) - 2))
    h = times[i + 1] - times[i]
    u = (t - times[i]) / h
    h00, h10 = 2 * u ** 3 - 3 * u ** 2 + 1, u ** 3 - 2 * u ** 2 + u
    h01, h11 = -2 * u ** 3 + 3 * u ** 2, u ** 3 - u ** 2
    return h00 * q[i] + h10 * h * qd[i] + h01 * q[i + 1] + h11 * h * qd[i + 1]


def surface_bounce(n, v, w, e, mu, r=0.02):
    """numpy port of tt_gazebo surface_bounce (surface at rest)."""
    vn = float(np.dot(v, n))
    if vn >= 0:
        return v, w
    vt = v - vn * n
    u = vt + r * np.cross(n, w)
    un = np.linalg.norm(u)
    dv = min(mu * (1 + e) * abs(vn), un / 2.5)
    dvt = -dv * u / un if un > 1e-9 else np.zeros(3)
    return vt + dvt - e * vn * n, w + (-3.0 / (2 * r)) * np.cross(n, dvt)


class ImpactSimulationTest(unittest.TestCase):
    """Replays the stroke through the plugin's contact logic (1 ms steps, finite-difference paddle
    velocity, ball parked at rest) and checks the M2 goals: timing <= 10 ms and direction."""

    @classmethod
    def setUpClass(cls):
        cls.chain = Chain(processed_urdf())
        cls.chain.set_weight_by_name('rail_joint', 0.1)
        cls.chain.set_weight_by_name('rail_y_joint', 0.1)

    def replay(self, out_speed, elevation, yaw, ball=(1.15, 0.0, 0.95), scale=4.0):
        pos, rot, v, n = hit_setup(self.chain, ball, out_speed, elevation, yaw)
        t_hit = 4.0
        stroke = build_stroke(self.chain, Q_READY, Q_READY, pos, rot, v, t_hit,
                              StrokeParams(vel_scale=scale))
        slab = BALL_R + PADDLE_HALF
        ball = np.asarray(ball, float)
        prev = None
        for k in range(int(5.0 / 1e-3)):
            t = k * 1e-3
            tf = self.chain.fk(hermite(stroke.times, stroke.q, stroke.qd, t))
            p, z = tf[:3, 3], tf[:3, 2]
            if prev is not None:
                vel = (p - prev[0]) / 1e-3
                d = ball - p
                h = float(np.dot(d, z))
                radial = np.linalg.norm(d - h * z)
                if abs(h) <= slab and radial <= 0.085:
                    side = 1.0 if h >= 0 else -1.0
                    nn = side * z
                    v_rel = -vel  # ball at rest, paddle rotation neglected at the hit
                    if np.dot(v_rel, nn) < 0:
                        v_rel, _ = surface_bounce(nn, v_rel, np.zeros(3), 0.85, 0.45)
                        return t, v_rel + vel, stroke, n
            prev = (p, z)
        self.fail('paddle never touched the ball')

    def check(self, out_speed, elevation, yaw, **kw):
        t, v_out, stroke, n = self.replay(out_speed, elevation, yaw, **kw)
        self.assertLess(abs(t - stroke.t_hit), 0.010, f'timing error {1000 * (t - stroke.t_hit):.1f} ms')
        ang = np.degrees(np.arccos(np.clip(np.dot(v_out / np.linalg.norm(v_out), n), -1, 1)))
        self.assertLess(ang, 3.0, f'direction error {ang:.2f} deg')
        self.assertAlmostEqual(np.linalg.norm(v_out), stroke.hit_speed * 1.85, delta=0.1)

    def test_forward_lift(self):
        self.check(4.5, 25.0, 0.0)

    def test_slow_with_real_panda_limits(self):
        self.check(1.0, 25.0, 0.0, scale=1.0)

    def test_aimed_left_and_right(self):
        self.check(3.0, 20.0, 15.0)
        self.check(3.0, 20.0, -15.0)


if __name__ == '__main__':
    unittest.main()
