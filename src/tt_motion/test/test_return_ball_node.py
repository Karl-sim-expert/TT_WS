"""Smoke test of the return_ball node logic with stubbed ROS (only runs where rclpy is missing,
i.e. in the offline environment; it catches missing methods / typos that unit tests of the
planner cannot): python3 -m unittest discover -s src/tt_motion/test"""
import importlib.util
import os
import sys
import types
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from urdf_utils import processed_urdf  # noqa: E402


@unittest.skipIf(importlib.util.find_spec('rclpy') is not None, 'real ROS present')
class ReturnBallNodeSmokeTest(unittest.TestCase):
    def test_incoming_ball_produces_a_plan(self):
        published = []

        class Log:
            def info(s, m): pass  # noqa: E704
            warn = error = info

        class Clock:
            t = 0.0

            def now(s):
                class T:
                    nanoseconds = int(Clock.t * 1e9)

                    def to_msg(s2): return types.SimpleNamespace(sec=int(Clock.t), nanosec=0)  # noqa: E704
                return T()

        class Node:
            def __init__(s, name): s._p, s.subs = {}, {}  # noqa: E704
            def declare_parameter(s, n, d): s._p[n] = d  # noqa: E704
            def get_parameter(s, n): return types.SimpleNamespace(value=s._p[n])  # noqa: E704
            def create_publisher(s, *a): return types.SimpleNamespace(publish=published.append)  # noqa: E704
            def create_subscription(s, typ, topic, cb, q): s.subs[topic] = cb  # noqa: E704
            def get_logger(s): return Log()  # noqa: E704
            def get_clock(s): return Clock()  # noqa: E704

        class V:
            def __init__(s): s.x = s.y = s.z = s.w = 0.0  # noqa: E704

        class StrokePlan:
            TOPSPIN_DRIVE = 0
            LOOP = 1
            BLOCK = 2

            def __init__(s):
                s.header = types.SimpleNamespace(stamp=None)
                s.hit_time = types.SimpleNamespace(sec=0, nanosec=0)
                s.paddle_pose = types.SimpleNamespace(position=V(), orientation=V())
                s.paddle_twist = types.SimpleNamespace(linear=V())
                s.target_landing = V()

        stubs = {'rclpy': types.ModuleType('rclpy'),
                 'rclpy.node': types.SimpleNamespace(Node=Node),
                 'nav_msgs': types.ModuleType('nav_msgs'),
                 'nav_msgs.msg': types.SimpleNamespace(Odometry=object),
                 'sensor_msgs': types.ModuleType('sensor_msgs'),
                 'sensor_msgs.msg': types.SimpleNamespace(JointState=object),
                 'builtin_interfaces': types.ModuleType('builtin_interfaces'),
                 'builtin_interfaces.msg': types.SimpleNamespace(Duration=lambda **kw: types.SimpleNamespace(**kw)),
                 'trajectory_msgs': types.ModuleType('trajectory_msgs'),
                 'trajectory_msgs.msg': types.SimpleNamespace(
                     JointTrajectory=lambda: types.SimpleNamespace(
                         header=types.SimpleNamespace(stamp=None), joint_names=[], points=[]),
                     JointTrajectoryPoint=lambda: types.SimpleNamespace()),
                 'tt_msgs': types.ModuleType('tt_msgs'),
                 'tt_msgs.msg': types.SimpleNamespace(StrokePlan=StrokePlan)}
        saved = {k: sys.modules.get(k) for k in stubs}
        sys.modules.update(stubs)
        try:
            from tt_motion import return_ball
            n = return_ball.ReturnBall.__new__(return_ball.ReturnBall)
            Node.__init__(n, 'x')
            orig = n.get_parameter
            n.get_parameter = lambda name: (types.SimpleNamespace(value=processed_urdf())
                                            if name == 'robot_description' else orig(name))
            return_ball.ReturnBall.__init__(n)

            def odom(t, p):
                z = types.SimpleNamespace(x=0, y=0, z=0)
                return types.SimpleNamespace(
                    header=types.SimpleNamespace(
                        stamp=types.SimpleNamespace(sec=int(t), nanosec=int((t - int(t)) * 1e9))),
                    pose=types.SimpleNamespace(pose=types.SimpleNamespace(
                        position=types.SimpleNamespace(x=p[0], y=p[1], z=p[2]))),
                    twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=z, angular=z)))

            p, v, t = np.array([-1.8, 0.0, 1.0]), np.array([5.0, 0.2, 2.4]), 10.0
            for _ in range(200):
                Clock.t = t
                n.subs['/model/ball/odometry'](odom(t, p))
                p, t = p + v * 0.001, t + 0.001
            self.assertEqual(len(published), 1)
            self.assertLess(published[0].paddle_twist.linear.x, 0.0)  # swings toward the opponent
            self.assertIn(published[0].stroke_type, (0, 1, 2))
            self.assertLess(published[0].target_landing.x, -0.4)

            # With a joint state the stroke is computed here and sent straight to the controller.
            published.clear()
            n.q = np.array(return_ball.READY)
            p, t = np.array([-1.8, 0.0, 1.0]), t + 5.0
            for _ in range(200):
                Clock.t = t
                n.subs['/model/ball/odometry'](odom(t, p))
                p, t = p + v * 0.001, t + 0.001
            self.assertEqual(len(published), 1)
            traj = published[0]
            self.assertGreater(len(traj.points), 50)
            first = traj.points[1].time_from_start
            self.assertGreater(first.sec + first.nanosec * 1e-9, 0.0)   # nothing of the move is in the past
            self.assertEqual(len(traj.joint_names), 9)
        finally:
            for k, m in saved.items():
                if m is None:
                    sys.modules.pop(k, None)
                else:
                    sys.modules[k] = m
            sys.modules.pop('tt_motion.return_ball', None)


if __name__ == '__main__':
    unittest.main()
