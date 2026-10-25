#!/usr/bin/env python3
"""M2 test: hit a motionless (parked) ball in a chosen direction and measure how well it went.

Per repetition: park the ball (/ball/hold), publish a StrokePlan whose hit time is `lead` seconds
ahead, wait for the plugin's /ball/paddle_hit report and print
  * timing error   = reported impact time - planned hit time   (design doc M2 goal: <= 10 ms)
  * direction error = angle between the outgoing ball velocity and the requested direction
  * speed ratio    = outgoing speed / requested speed (< 1 if the joint velocity limits cap it)

For a resting ball the paddle only needs to move along the face normal: with restitution e the
ball leaves at (1 + e) times the paddle speed, so  v_paddle = v_out / (1 + e)  along  n = v_out/|v_out|.
The paddle origin at impact is  p_ball - (r + t/2) n  (ball touching the face).
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tt_msgs.msg import StrokePlan

from tt_motion.kinematics import Chain, matrix_to_quat
from tt_motion.stroke import paddle_rotation_for

READY = [0.0, -0.058, 0.132, 0.351, -2.048, -0.631, 1.803, 1.559]
BALL_RADIUS = 0.02
PADDLE_HALF_THICKNESS = 0.006
TIMING_LIMIT = 0.010  # s


class HitStaticBall(Node):
    def __init__(self):
        super().__init__('hit_static_ball')
        for name, default in (
                ('robot_description', ''), ('tip_link', 'paddle'), ('q_ready', READY),
                ('ball_x', 1.15), ('ball_y', 0.0), ('ball_z', 0.95),
                ('speed', 1.0), ('elevation_deg', 25.0), ('yaw_deg', 0.0),
                ('paddle_restitution', 0.85), ('lead', 4.0), ('repeat', 1), ('settle', 6.0)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.chain = Chain(g('robot_description'), tip=g('tip_link'))
        self.q_ready = np.array(g('q_ready'), float)
        self.ball = np.array([g('ball_x'), g('ball_y'), g('ball_z')], float)
        el, yaw = math.radians(g('elevation_deg')), math.radians(g('yaw_deg'))
        # direction from the robot side (x > 0) toward the opponent half (-x)
        self.n = np.array([-math.cos(el) * math.cos(yaw), -math.cos(el) * math.sin(yaw),
                           math.sin(el)])
        self.speed = float(g('speed'))
        self.e = float(g('paddle_restitution'))
        self.lead = float(g('lead'))
        self.repeat = int(g('repeat'))
        self.settle = float(g('settle'))

        self.hold_pub = self.create_publisher(Vector3, '/ball/hold', 10)
        self.plan_pub = self.create_publisher(StrokePlan, 'stroke_plan', 10)
        self.create_subscription(Odometry, '/ball/paddle_hit', self.on_hit, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.create_subscription(Odometry, '/model/ball/odometry', self.on_ball, 10)
        self.have_joints = False
        self.paddle_log = deque(maxlen=1500)   # (t, paddle origin, paddle normal) from joint states
        self.ball_log = deque(maxlen=1500)     # (t, ball position, ball speed)
        self.diag_done = True
        self.run = 0
        self.plan_hit_time = None
        self.results = []
        self.state = 'wait'
        self.state_since = self.now()
        self.create_timer(0.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joint_state(self, msg):
        self.have_joints = True
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in self.chain.joint_names):
            tf = self.chain.fk(np.array([pos[n] for n in self.chain.joint_names]))
            self.paddle_log.append((self.now(), tf[:3, 3].copy(), tf[:3, 2].copy()))

    def on_ball(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.ball_log.append((self.now(), np.array([p.x, p.y, p.z]),
                              math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)))

    def diagnose(self):
        """Where was the paddle (from joint states + FK) at the planned hit time, and the ball?"""
        t_hit = self.plan_hit_time
        if not self.paddle_log:
            return
        t, p, z = min(self.paddle_log, key=lambda e: abs(e[0] - t_hit))
        target = self.ball - (BALL_RADIUS + PADDLE_HALF_THICKNESS) * self.n
        gap = float(np.dot(self.ball - p, z))            # ball centre above the paddle face
        radial = float(np.linalg.norm((self.ball - p) - gap * z))
        self.get_logger().info(
            f'DIAG at planned hit time (sample {1000 * (t - t_hit):+.0f} ms): paddle origin '
            f'{np.round(p, 3).tolist()} (plan {np.round(target, 3).tolist()}, error '
            f'{1000 * np.linalg.norm(p - target):.0f} mm); ball centre is {1000 * gap:.0f} mm along '
            f'the paddle normal (contact at {1000 * (BALL_RADIUS + PADDLE_HALF_THICKNESS):.0f} mm) '
            f'and {1000 * radial:.0f} mm off the paddle axis; normal {np.round(z, 3).tolist()} '
            f'(plan {np.round(self.n, 3).tolist()})')
        later = [e for e in self.ball_log if e[0] >= t_hit + 0.3]
        if later:
            self.get_logger().info(
                f'DIAG ball 0.3 s after the planned hit: pos {np.round(later[0][1], 3).tolist()}, '
                f'speed {later[0][2]:.2f} m/s')

    def hold_ball(self):
        v = Vector3(x=float(self.ball[0]), y=float(self.ball[1]), z=float(self.ball[2]))
        self.hold_pub.publish(v)

    def send_plan(self):
        n = self.n
        pos = self.ball - (BALL_RADIUS + PADDLE_HALF_THICKNESS) * n
        rot = paddle_rotation_for(self.chain, self.q_ready, n)
        q = matrix_to_quat(rot)
        hit = self.now() + self.lead
        plan = StrokePlan()
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.hit_time.sec = int(hit)
        plan.hit_time.nanosec = int(round((hit - int(hit)) * 1e9))
        plan.paddle_pose.position.x, plan.paddle_pose.position.y, plan.paddle_pose.position.z = \
            map(float, pos)
        (plan.paddle_pose.orientation.x, plan.paddle_pose.orientation.y,
         plan.paddle_pose.orientation.z, plan.paddle_pose.orientation.w) = map(float, q)
        v = n * self.speed / (1.0 + self.e)
        plan.paddle_twist.linear.x, plan.paddle_twist.linear.y, plan.paddle_twist.linear.z = \
            map(float, v)
        plan.target_landing.x = float('nan')  # not used in M2
        plan.stroke_type = StrokePlan.TOPSPIN_DRIVE
        self.plan_hit_time = plan.hit_time.sec + plan.hit_time.nanosec * 1e-9
        self.diag_done = False
        self.plan_pub.publish(plan)

    def tick(self):
        t = self.now()
        if self.state == 'wait':
            # Wait for the sim clock, joint states and a subscriber on the plan topic.
            if t > 1.0 and self.have_joints and self.plan_pub.get_subscription_count() > 0:
                self.get_logger().info(
                    f'run {self.run + 1}/{self.repeat}: ball parked at {self.ball.tolist()}, '
                    f'requested out speed {self.speed:.2f} m/s along {np.round(self.n, 3).tolist()}')
                self.hold_ball()
                self.state, self.state_since = 'park', t
        elif self.state == 'park':
            self.hold_ball()  # repeated so a dropped first message does not matter
            if t - self.state_since > 1.0:
                self.send_plan()
                self.state, self.state_since = 'swing', t
        elif self.state == 'swing':
            if not self.diag_done and t >= self.plan_hit_time + 0.4:
                self.diag_done = True
                self.diagnose()
            if t - self.state_since > self.lead + 2.0:
                self.get_logger().error('no paddle hit reported (missed the ball?)')
                self.results.append(None)
                self.next_run(t)
        elif self.state == 'return':
            if t - self.state_since > self.settle:
                self.next_run(t)

    def next_run(self, t):
        self.run += 1
        if self.run >= self.repeat:
            self.summary()
            self.state = 'done'
            rclpy.shutdown()
            return
        self.state, self.state_since = 'wait', t

    def on_hit(self, msg):
        if self.state != 'swing' or self.plan_hit_time is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z])
        speed = float(np.linalg.norm(v))
        cosang = float(np.dot(v / speed, self.n)) if speed > 1e-9 else -1.0
        ang = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
        dt = stamp - self.plan_hit_time
        ok = abs(dt) <= TIMING_LIMIT
        self.results.append((dt, ang, speed / self.speed))
        self.get_logger().info(
            f'HIT: timing error {1000 * dt:+.1f} ms ({"OK" if ok else "FAIL"}, limit '
            f'{1000 * TIMING_LIMIT:.0f} ms), direction error {ang:.1f} deg, out speed '
            f'{speed:.2f} m/s ({100 * speed / self.speed:.0f}% of requested), '
            f'ball pos {[round(c, 3) for c in (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)]}')
        self.state, self.state_since = 'return', self.now()

    def summary(self):
        good = [r for r in self.results if r is not None]
        self.get_logger().info(f'=== {len(good)}/{self.repeat} hits ===')
        if good:
            dts = [abs(r[0]) for r in good]
            angs = [r[1] for r in good]
            self.get_logger().info(
                f'timing |error|: mean {1000 * np.mean(dts):.1f} ms, max {1000 * max(dts):.1f} ms '
                f'-> {"PASS" if max(dts) <= TIMING_LIMIT else "FAIL"} (<= {1000 * TIMING_LIMIT:.0f} ms)')
            self.get_logger().info(
                f'direction error: mean {np.mean(angs):.1f} deg, max {max(angs):.1f} deg')


def main():
    rclpy.init()
    node = HitStaticBall()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
