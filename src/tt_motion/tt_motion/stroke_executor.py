#!/usr/bin/env python3
"""Turns a StrokePlan into a joint trajectory and sends it to the arm controller.

  in : stroke_plan (tt_msgs/StrokePlan)   hit time (sim clock), paddle pose and velocity at impact
       /joint_states
  out: /panda_arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory)

The trajectory header stamp is the (sim) time the plan was received and every point time is
relative to it, so the paddle reaches the hit pose at the absolute time StrokePlan.hit_time; the
controller interpolates against the sim clock. Kinematics come from the URDF (robot_description
parameter), see kinematics.py and stroke.py.
"""
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tt_msgs.msg import StrokePlan

from tt_motion.kinematics import Chain, quat_to_matrix
from tt_motion.stroke import StrokeError, StrokeParams, build_stroke, paddle_rotation_for

READY = [0.0, -0.058, 0.132, 0.351, -2.048, -0.631, 1.803, 1.559]  # rail_joint + panda_joint1..7, as in tt_robot.urdf.xacro


class StrokeExecutor(Node):
    def __init__(self):
        super().__init__('stroke_executor')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('tip_link', 'paddle')
        self.declare_parameter('q_ready', READY)
        self.declare_parameter('vel_scale', 1.0)
        # gz_ros2_control follows position commands with a first-order lag (time constant
        # 1 / (position_proportional_gain * update_rate) = 10 ms at gain 0.1, 1 kHz). Send the
        # trajectory this much earlier so the paddle arrives at the planned time.
        self.declare_parameter('tracking_lag', 0.010)
        self.declare_parameter('rail_joint', 'rail_joint')
        self.declare_parameter('rail_weight', 0.1)   # < 1: the rail carries the translation
        self.declare_parameter('d_back', 0.20)
        self.declare_parameter('d_follow', 0.15)
        self.declare_parameter('trajectory_topic', '/panda_arm_controller/joint_trajectory')

        urdf = self.get_parameter('robot_description').value
        if not urdf:
            raise RuntimeError('robot_description parameter is empty')
        self.chain = Chain(urdf, tip=self.get_parameter('tip_link').value)
        self.chain.set_weight_by_name(
            self.get_parameter('rail_joint').value, self.get_parameter('rail_weight').value)
        self.q_ready = np.array(self.get_parameter('q_ready').value, float)
        self.params = StrokeParams(
            d_back=self.get_parameter('d_back').value,
            d_follow=self.get_parameter('d_follow').value,
            vel_scale=self.get_parameter('vel_scale').value)

        self.q = None
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.create_subscription(StrokePlan, 'stroke_plan', self.on_plan, 10)
        self.pub = self.create_publisher(
            JointTrajectory, self.get_parameter('trajectory_topic').value, 10)
        self.get_logger().info(
            f'ready: vel_scale={self.params.vel_scale}, {self.chain.n} joints '
            f'({", ".join(self.chain.joint_names)}), waiting for stroke_plan')

    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in self.chain.joint_names):
            self.q = np.array([pos[n] for n in self.chain.joint_names])

    def on_plan(self, plan):
        if self.q is None:
            self.get_logger().error('no joint state yet, plan ignored')
            return
        now = self.get_clock().now()
        hit_abs = plan.hit_time.sec + plan.hit_time.nanosec * 1e-9
        t_hit = hit_abs - now.nanoseconds * 1e-9

        pos = np.array([plan.paddle_pose.position.x, plan.paddle_pose.position.y,
                        plan.paddle_pose.position.z])
        vel = np.array([plan.paddle_twist.linear.x, plan.paddle_twist.linear.y,
                        plan.paddle_twist.linear.z])
        o = plan.paddle_pose.orientation
        quat = np.array([o.x, o.y, o.z, o.w])
        if np.linalg.norm(quat) < 1e-6:  # no orientation given: paddle normal along the velocity
            rot = paddle_rotation_for(self.chain, self.q_ready, vel / np.linalg.norm(vel))
        else:
            rot = quat_to_matrix(quat)

        lag = float(self.get_parameter('tracking_lag').value)
        try:
            stroke = build_stroke(
                self.chain, self.q, self.q_ready, pos, rot, vel, t_hit - lag, self.params)
        except StrokeError as err:
            self.get_logger().error(f'stroke rejected: {err}')
            return

        req = float(np.linalg.norm(vel))
        msg = JointTrajectory()
        msg.header.stamp = now.to_msg()
        msg.joint_names = list(self.chain.joint_names)
        for t, q, qd in zip(stroke.times, stroke.q, stroke.qd):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in q]
            p.velocities = [float(v) for v in qd]
            sec = int(t)
            p.time_from_start = Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))
            msg.points.append(p)
        self.pub.publish(msg)
        self.get_logger().info(
            f'stroke sent: hit in {t_hit:.2f} s (commanded {lag * 1000:.0f} ms early, min lead {stroke.min_lead:.2f} s), paddle speed '
            f'{stroke.hit_speed:.2f} m/s (requested {req:.2f}), peak joint speed '
            f'{100 * stroke.max_joint_vel_ratio:.0f}% of limit, {len(msg.points)} points')
        if stroke.hit_speed < 0.99 * req:
            self.get_logger().warn(
                f'hit speed limited by joint velocity limits (vel_scale={self.params.vel_scale})')


def main():
    rclpy.init()
    node = StrokeExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
