#!/usr/bin/env python3
"""M3: return balls fired by the ball machine (ground-truth ball state, no vision yet).

  in : /model/ball/odometry (nav_msgs/Odometry, Gazebo ground truth)   /ball/paddle_hit
  out: stroke_plan (tt_msgs/StrokePlan) -> stroke_executor

When a ball leaves the machine (x < -1 m, moving toward the robot) the node predicts its flight
including the bounce (return_planner.plan_return), publishes one StrokePlan, then checks the
result: timing of the paddle hit and where the returned ball lands.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tt_msgs.msg import StrokePlan

from tt_motion.kinematics import Chain, matrix_to_quat
from tt_motion.return_planner import ReturnParams, plan_return
from tt_motion.stroke import paddle_rotation_for

READY = [0.0, 0.0, -0.058, 0.132, 0.351, -2.048, -0.631, 1.803, 1.559]


class ReturnBall(Node):
    def __init__(self):
        super().__init__('return_ball')
        for name, default in (
                ('robot_description', ''), ('tip_link', 'paddle'), ('q_ready', READY),
                ('trigger_min_x', -1.55), ('target_x', -0.7), ('target_y', 0.0), ('min_lead', 0.6),
                ('trigger_x', -1.0), ('trigger_vx', 2.0), ('cooldown', 1.5),
                ('x_min', 1.0), ('x_max', 1.4), ('y_min', -0.7), ('y_max', 0.7),
                ('z_min', 0.85), ('z_max', 1.15), ('timing_limit', 0.010),
                ('spin_from_odometry', False)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.chain = Chain(g('robot_description'), tip=g('tip_link'))
        self.q_ready = np.array(g('q_ready'), float)
        self.params = ReturnParams(
            box_x=(g('x_min'), g('x_max')), box_y=(g('y_min'), g('y_max')),
            box_z=(g('z_min'), g('z_max')), min_lead=float(g('min_lead')),
            target=(float(g('target_x')), float(g('target_y'))))
        self.plan_pub = self.create_publisher(StrokePlan, 'stroke_plan', 10)
        self.create_subscription(Odometry, '/model/ball/odometry', self.on_ball, 10)
        self.create_subscription(Odometry, '/ball/paddle_hit', self.on_paddle_hit, 10)
        self.armed_after = 0.0
        self.plan = None
        self.plan_hit_abs = None
        self.hit_done = False
        self.prev = None
        self.prev_vz = None
        self.landed = False
        self.shots = self.planned = self.hits = self.returned = 0
        self.get_logger().info('return_ball ready: waiting for balls from the machine')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def state_of(msg):
        p, v, w = msg.pose.pose.position, msg.twist.twist.linear, msg.twist.twist.angular
        return (np.array([p.x, p.y, p.z]), np.array([v.x, v.y, v.z]), np.array([w.x, w.y, w.z]))

    def on_ball(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = stamp if stamp > 0.0 else self.now()
        p, v_odom, w = self.state_of(msg)
        if self.prev is None:
            self.get_logger().info(
                f'ball odometry received (t={t:.2f}, pos {np.round(p, 2).tolist()}); '
                'waiting for a shot')
        # Velocity from position differences: the odometry twist may be in the body frame.
        v = v_odom
        if self.prev is not None and t - self.prev[0] > 0.004:
            v = (p - self.prev[1]) / (t - self.prev[0])
            self.prev = (t, p, v)
        elif self.prev is None:
            self.prev = (t, p, v)
            return
        else:
            return
        if self.plan is not None:
            self.watch_landing(t, p, v)
            self.prev_vz = v[2]
        elif (t >= self.armed_after and self.get_parameter('trigger_min_x').value < p[0]
              < self.get_parameter('trigger_x').value
              and v[0] > self.get_parameter('trigger_vx').value):
            self.get_logger().info(
                f'ball launched: pos {np.round(p, 2).tolist()}, v {np.round(v, 2).tolist()} '
                f'(odometry twist says {np.round(v_odom, 2).tolist()})')
            self.on_incoming(t, p, v, w)

    def on_incoming(self, t, p, v, w):
        self.shots += 1
        spin = w if self.get_parameter('spin_from_odometry').value else np.zeros(3)
        started = self.now()
        plan = plan_return(p, v, spin, self.params)
        latency = self.now() - started
        self.armed_after = t + float(self.get_parameter('cooldown').value)
        if isinstance(plan, str):
            self.get_logger().warn(
                f'shot {self.shots}: cannot return ({plan}); ball v={np.round(v, 2).tolist()}')
            return
        hit_abs = t + plan.t_hit
        remaining = hit_abs - self.now()
        msg = StrokePlan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hit_time.sec = int(hit_abs)
        msg.hit_time.nanosec = int(round((hit_abs - int(hit_abs)) * 1e9))
        msg.paddle_pose.position.x, msg.paddle_pose.position.y, msg.paddle_pose.position.z = \
            map(float, plan.paddle_pos)
        # The orientation must be set explicitly: an all-zero quaternion is not the message default
        # (w defaults to 1 = identity), so the executor would not derive it from the velocity.
        q = matrix_to_quat(paddle_rotation_for(self.chain, self.q_ready, plan.paddle_normal))
        (msg.paddle_pose.orientation.x, msg.paddle_pose.orientation.y,
         msg.paddle_pose.orientation.z, msg.paddle_pose.orientation.w) = map(float, q)
        v_p = plan.paddle_speed * plan.paddle_normal
        msg.paddle_twist.linear.x, msg.paddle_twist.linear.y, msg.paddle_twist.linear.z = \
            map(float, v_p)
        msg.target_landing.x, msg.target_landing.y = map(float, self.params.target)
        msg.target_landing.z = 0.76
        msg.stroke_type = StrokePlan.TOPSPIN_DRIVE
        self.plan_pub.publish(msg)
        self.plan, self.plan_hit_abs, self.hit_done, self.planned = plan, hit_abs, False, self.planned + 1
        self.landed, self.prev_vz = False, None
        self.get_logger().info(
            f'shot {self.shots}: ball v={np.round(v, 2).tolist()}; hit at '
            f'{np.round(plan.ball_pos, 2).tolist()} in {remaining:.2f} s (planning took '
            f'{1000 * latency:.0f} ms), paddle {plan.paddle_speed:.2f} m/s along '
            f'{np.round(plan.paddle_normal, 2).tolist()}, expected landing '
            f'{np.round(plan.landing[:2], 2).tolist()}')
        if remaining < 0.3:
            self.get_logger().warn('very little time left: the arm may not make it')

    def on_paddle_hit(self, msg):
        if self.plan is None or self.hit_done:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        speed = float(np.linalg.norm(v))
        want = self.plan.v_out
        cosang = float(np.dot(v, want) / (speed * np.linalg.norm(want) + 1e-9))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
        dt = t - self.plan_hit_abs
        self.hit_done = True
        self.hits += 1
        limit = float(self.get_parameter('timing_limit').value)
        self.get_logger().info(
            f'HIT {self.hits}/{self.planned}: timing error {1000 * dt:+.1f} ms '
            f'({"OK" if abs(dt) <= limit else "late/early"}), out speed {speed:.2f} m/s '
            f'(planned {np.linalg.norm(want):.2f}), direction error {ang:.1f} deg')

    def watch_landing(self, t, p, v):
        """First table bounce after the hit = where the returned ball lands."""
        if self.hit_done and not self.landed and self.prev_vz is not None \
                and self.prev_vz < 0.0 <= v[2] and p[2] < 0.82:
            self.landed = True
            on_table = abs(p[0]) <= 1.37 and abs(p[1]) <= 0.7625 and p[0] < 0.0
            err = math.hypot(p[0] - self.params.target[0], p[1] - self.params.target[1])
            self.returned += int(on_table)
            self.get_logger().info(
                f'RESULT: landed at ({p[0]:.2f}, {p[1]:.2f}), target {self.params.target}, '
                f'error {100 * err:.0f} cm, {"ON opponent half" if on_table else "MISSED the table"}'
                f' | returned {self.returned}/{self.shots} shots ({self.hits} hit)')
            self.plan = None
        elif t > self.plan_hit_abs + 2.0:
            what = 'paddle never touched the ball' if not self.hit_done \
                else 'returned ball never bounced on the table'
            self.get_logger().warn(f'MISS: {what} | returned {self.returned}/{self.shots}')
            self.plan = None


def main():
    rclpy.init()
    node = ReturnBall()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
