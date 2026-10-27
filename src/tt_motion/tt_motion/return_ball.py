#!/usr/bin/env python3
"""M3: return balls fired by the ball machine (ground-truth ball state, no vision yet).

  in : /model/ball/odometry (nav_msgs/Odometry, Gazebo ground truth)   /ball/paddle_hit
  out: stroke_plan (tt_msgs/StrokePlan) -> stroke_executor

When a ball leaves the machine (x < -1 m, moving toward the robot) the node predicts its flight
including the bounce (return_planner.plan_return), publishes one StrokePlan, then checks the
result: timing of the paddle hit and where the returned ball lands.
"""
import math
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
from tt_msgs.msg import StrokePlan

from tt_motion.kinematics import Chain, matrix_to_quat
from tt_motion.return_planner import ReturnParams, contact, plan_return
from tt_motion.shot_selector import NAMES, select_shot
from tt_motion.stroke import StrokeError, StrokeParams, build_stroke, paddle_rotation_for

READY = [0.0, 0.0, -0.058, 0.132, 0.351, -2.048, -0.631, 1.803, 1.559]


class ReturnBall(Node):
    def __init__(self):
        super().__init__('return_ball')
        for name, default in (
                ('robot_description', ''), ('tip_link', 'paddle'), ('q_ready', READY),
                ('trigger_min_x', -1.55), ('contact_shift', 0.035), ('target_x', -0.7), ('target_y', 0.0), ('min_lead', 0.6),
                ('trigger_x', -1.0), ('trigger_vx', 2.0), ('cooldown', 0.5),
                ('x_min', 1.2), ('x_max', 1.5), ('y_min', -0.8), ('y_max', 0.8),
                ('z_min', 0.85), ('z_max', 1.15), ('timing_limit', 0.010),
                ('spin_from_odometry', False), ('strategy', 'rules'),
                ('aim_gain', 0.7), ('aim_bias_max', 0.35),
                # same values as the stroke_executor: used to pre-check that a hit point is executable
                ('d_back', 0.08), ('d_follow', 0.10), ('min_move_time', 0.2), ('prep_vel_frac', 0.9),
                ('vel_scale', 1.0), ('rail_weight', 0.1), ('tracking_lag', 0.040),
                ('check_margin', 0.12)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.chain = Chain(g('robot_description'), tip=g('tip_link'))
        self.q_ready = np.array(g('q_ready'), float)
        self.stroke_chain = Chain(g('robot_description'), tip=g('tip_link'))
        for j in ('rail_joint', 'rail_y_joint'):
            self.stroke_chain.set_weight_by_name(j, float(g('rail_weight')))
        self.stroke_params = StrokeParams(
            d_back=float(g('d_back')), d_follow=float(g('d_follow')), vel_scale=float(g('vel_scale')),
            min_move_time=float(g('min_move_time')), prep_vel_frac=float(g('prep_vel_frac')))
        self.q = None
        self.params = ReturnParams(
            box_x=(g('x_min'), g('x_max')), box_y=(g('y_min'), g('y_max')),
            box_z=(g('z_min'), g('z_max')), min_lead=float(g('min_lead')),
            target=(float(g('target_x')), float(g('target_y'))))
        self.plan_pub = self.create_publisher(StrokePlan, 'stroke_plan', 10)
        self.create_subscription(Odometry, '/model/ball/odometry', self.on_ball, 10)
        self.create_subscription(Odometry, '/ball/paddle_hit', self.on_paddle_hit, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 50)
        self.ball_hist = deque(maxlen=400)    # (sim time, ball position)
        self.paddle_log = deque(maxlen=800)   # (sim time, paddle origin, paddle normal) from FK
        self.armed_after = 0.0
        self.plan = None
        self.plan_hit_abs = None
        self.hit_done = False
        self.prev = None
        self.prev_vz = None
        self.landed = False
        self.shots = self.planned = self.hits = self.returned = self.net_fails = 0
        self.errors = []
        self.aim_bias = np.zeros(2)  # learned landing correction [m] (x, y)
        self.net_clear = None
        self.last_pos = None
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
        self.ball_hist.append((t, p.copy()))
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
        launched = (t >= self.armed_after and self.get_parameter('trigger_min_x').value < p[0]
                    < self.get_parameter('trigger_x').value
                    and v[0] > self.get_parameter('trigger_vx').value)
        if launched:
            if self.plan is not None:  # next shot already on its way: drop the old one
                self.plan = None
            self.get_logger().info(
                f'ball launched: pos {np.round(p, 2).tolist()}, v {np.round(v, 2).tolist()} '
                f'(odometry twist says {np.round(v_odom, 2).tolist()})')
            self.on_incoming(t, p, v, w)
        elif self.plan is not None:
            self.watch_landing(t, p, v)
            self.prev_vz = v[2]

    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in self.chain.joint_names):
            self.q = np.array([pos[n] for n in self.chain.joint_names])
            tf = self.chain.fk(np.array([pos[n] for n in self.chain.joint_names]))
            self.paddle_log.append((self.now(), tf[:3, 3].copy(), tf[:3, 2].copy()))

    def diagnose(self, t_contact):
        """Where was the paddle (joint states + FK) at the planned time and at the actual contact?"""
        if len(self.paddle_log) < 3:
            return
        plan = self.plan
        for label, t in (('planned time', self.plan_hit_abs), ('contact', t_contact)):
            i = min(range(len(self.paddle_log)), key=lambda k: abs(self.paddle_log[k][0] - t))
            lo, hi = max(i - 5, 0), min(i + 5, len(self.paddle_log) - 1)
            ta, pa, _ = self.paddle_log[lo]
            tb, pb, _ = self.paddle_log[hi]
            vel = (pb - pa) / max(tb - ta, 1e-6)
            _, pos, nrm = self.paddle_log[i]
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(nrm, plan.paddle_normal))))))
            self.get_logger().info(
                f'DIAG {label}: paddle pos error {1000 * np.linalg.norm(pos - plan.paddle_pos):.0f} mm '
                f'({np.round(1000 * (pos - plan.paddle_pos), 0).tolist()}), normal error {tilt:.1f} deg, '
                f'speed {np.linalg.norm(vel):.2f} m/s (planned {plan.paddle_speed:.2f}), '
                f'along plan normal {float(np.dot(vel, plan.paddle_normal)):.2f} m/s')

    def diagnose_ball(self, t_contact, v_out):
        """Real ball state just before the impact vs the prediction, and what the contact model says."""
        early = [e for e in self.ball_hist if t_contact - 0.020 <= e[0] <= t_contact - 0.004]
        if len(early) < 2:
            return
        v_act = (early[-1][1] - early[0][1]) / max(early[-1][0] - early[0][0], 1e-6)
        plan = self.plan
        model_out, _ = contact(v_act, plan.w_in, plan.paddle_normal, plan.paddle_speed, self.params, None)
        i = min(range(len(self.paddle_log)), key=lambda k: abs(self.paddle_log[k][0] - t_contact)) \
            if self.paddle_log else None
        geom = ''
        if i is not None:
            _, ppos, pnrm = self.paddle_log[i]
            d = early[-1][1] - ppos
            h = float(np.dot(d, pnrm))
            geom = f', ball {1000 * h:.0f} mm in front of the paddle, {1000 * np.linalg.norm(d - h * pnrm):.0f} mm off its axis'
        self.get_logger().info(
            f'DIAG ball before impact: v {np.round(v_act, 2).tolist()} (predicted {np.round(plan.v_in, 2).tolist()}){geom}; '
            f'contact model with the real v gives out {np.round(model_out, 2).tolist()}, actual out {np.round(v_out, 2).tolist()}')

    def stroke_pose(self, plan):
        """Paddle pose to command for a plan: orientation from the normal, and the contact shift.

        The hand sits beside the disc on its +x' side and sticks out in front of the face: let the
        ball meet the disc on the opposite side (timing jitter moves the contact point along the
        ball's tangential velocity, mostly up = toward the hand)."""
        rot = paddle_rotation_for(self.chain, self.q_ready, plan.paddle_normal)
        pos = plan.paddle_pos + float(self.get_parameter('contact_shift').value) * rot[:, 0]
        return pos, rot

    def check_stroke(self, plan, t_obs):
        """None if the arm can execute this hit (reach, joint speeds, lead time, table), else why not."""
        if self.q is None:
            return None
        pos, rot = self.stroke_pose(plan)
        lag = float(self.get_parameter('tracking_lag').value)
        lead = t_obs + plan.t_hit - self.now() - lag - float(self.get_parameter('check_margin').value)
        try:
            build_stroke(self.stroke_chain, self.q, self.q_ready, pos, rot,
                         plan.paddle_speed * plan.paddle_normal, lead, self.stroke_params)
        except StrokeError as err:
            return str(err).split(' (')[0][:90]
        return None

    def on_incoming(self, t, p, v, w):
        self.shots += 1
        spin = w if self.get_parameter('spin_from_odometry').value else np.zeros(3)
        started = time.monotonic()
        # strategy 'rules' (M4): target/stroke chosen from the predicted hit; 'fixed' (M3): target_x/y
        selector = select_shot if self.get_parameter('strategy').value == 'rules' else None
        plan = plan_return(p, v, spin, self.params, selector=selector,
                           aim_offset=tuple(self.aim_bias), check=lambda pl: self.check_stroke(pl, t))
        latency = time.monotonic() - started
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
        # The orientation must be set explicitly: an all-zero quaternion is not the message default
        # (w defaults to 1 = identity), so the executor would not derive it from the velocity.
        plan.paddle_pos, rot = self.stroke_pose(plan)
        msg.paddle_pose.position.x, msg.paddle_pose.position.y, msg.paddle_pose.position.z = \
            map(float, plan.paddle_pos)
        q = matrix_to_quat(rot)
        (msg.paddle_pose.orientation.x, msg.paddle_pose.orientation.y,
         msg.paddle_pose.orientation.z, msg.paddle_pose.orientation.w) = map(float, q)
        v_p = plan.paddle_speed * plan.paddle_normal
        msg.paddle_twist.linear.x, msg.paddle_twist.linear.y, msg.paddle_twist.linear.z = \
            map(float, v_p)
        msg.target_landing.x, msg.target_landing.y = map(float, plan.target)
        msg.target_landing.z = 0.76
        msg.stroke_type = int(plan.stroke_type)
        self.plan_pub.publish(msg)
        self.plan, self.plan_hit_abs, self.hit_done, self.planned = plan, hit_abs, False, self.planned + 1
        self.landed, self.prev_vz = False, None
        self.net_clear, self.last_pos = None, None
        self.get_logger().info(
            f'shot {self.shots}: ball v={np.round(v, 2).tolist()}; hit at '
            f'{np.round(plan.ball_pos, 2).tolist()} in {remaining:.2f} s (planning took '
            f'{1000 * latency:.0f} ms wall time), paddle {plan.paddle_speed:.2f} m/s along '
            f'{np.round(plan.paddle_normal, 2).tolist()}, expected landing '
            f'{np.round(plan.landing[:2], 2).tolist()}')
        choice = plan.info.get('choice')
        self.get_logger().info(
            f'SHOT {self.shots}: {NAMES.get(plan.stroke_type, "?")} to target '
            f'({plan.target[0]:.2f}, {plan.target[1]:.2f}), launch elevation {plan.elevation_deg:.0f} deg'
            + (f' ({choice.reason})' if choice else ' (fixed target)' if not plan.info.get('fallback')
               else ' (selected shot not solvable: fixed target)'))
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
        self.diagnose(t)
        self.diagnose_ball(t, v)
        limit = float(self.get_parameter('timing_limit').value)
        self.get_logger().info(
            f'HIT {self.hits}/{self.planned}: timing error {1000 * dt:+.1f} ms '
            f'({"OK" if abs(dt) <= limit else "late/early"}), out speed {speed:.2f} m/s '
            f'(planned {np.linalg.norm(want):.2f}), direction error {ang:.1f} deg')

    def watch_landing(self, t, p, v):
        """First table bounce after the hit = where the returned ball lands (and net clearance)."""
        if self.hit_done and not self.landed and self.last_pos is not None \
                and self.last_pos[0] > 0.0 >= p[0]:
            f = self.last_pos[0] / (self.last_pos[0] - p[0])
            z = self.last_pos[2] + f * (p[2] - self.last_pos[2])
            self.net_clear = z - 0.02 - self.params.net_top
        self.last_pos = p.copy()
        if self.hit_done and not self.landed and self.prev_vz is not None \
                and self.prev_vz < 0.0 <= v[2] and p[2] < 0.82 and v[0] < 0.0:
            self.landed = True
            on_table = abs(p[0]) <= 1.37 and abs(p[1]) <= 0.7625 and p[0] < 0.0
            target = self.plan.target
            err = math.hypot(p[0] - target[0], p[1] - target[1])
            self.returned += int(on_table)
            self.errors.append(err)
            net_ok = self.net_clear is not None and self.net_clear > 0.0
            self.net_fails += int(not net_ok)
            # integral correction of the systematic landing error (contact model vs. real paddle)
            gain, lim = float(self.get_parameter('aim_gain').value), float(self.get_parameter('aim_bias_max').value)
            self.aim_bias = np.clip(self.aim_bias + gain * (np.array(target) - p[:2]), -lim, lim)
            self.get_logger().info(
                f'RESULT: landed at ({p[0]:.2f}, {p[1]:.2f}), target ({target[0]:.2f}, {target[1]:.2f}), '
                f'error {100 * err:.0f} cm, {"ON opponent half" if on_table else "MISSED the table"}'
                f', net {"cleared" if net_ok else "NOT cleared"}'
                f' | returned {self.returned}/{self.shots} shots ({self.hits} hit) | '
                f'{sum(e <= 0.15 for e in self.errors)}/{len(self.errors)} within 15 cm, '
                f'aim bias ({self.aim_bias[0]:+.2f}, {self.aim_bias[1]:+.2f}), '
                f'mean {100 * float(np.mean(self.errors)):.0f} cm, net fails {self.net_fails}')
            self.plan = None
        elif t > self.plan_hit_abs + 2.0:
            what = 'paddle never touched the ball' if not self.hit_done \
                else 'returned ball never bounced on the table'
            self.get_logger().warn(f'MISS: {what} | returned {self.returned}/{self.shots}')
            if not self.hit_done:
                self.diagnose(self.plan_hit_abs)
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
