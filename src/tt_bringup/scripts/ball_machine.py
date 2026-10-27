#!/usr/bin/env python3
"""Ball machine: fires balls at a chosen landing point with a chosen speed and spin.

The launch direction is found by integrating the same ball model as the BallPhysics plugin
(drag + Magnus + table bounce), so the ball lands where requested. Launch messages go to
/ball/launch (geometry_msgs/Twist: linear = velocity, angular = spin, world frame).

Also usable without ROS for checking:  ball_machine.py --solve 9 0.8 0 [spin_top spin_side]
"""
import math
import random
import sys

# Same constants as tt_gazebo/ball_dynamics.hpp and the table_tennis.sdf world.
MASS, RADIUS, RHO, CD, LIFT_SLOPE, LIFT_MAX = 0.0027, 0.02, 1.2, 0.40, 1.0, 0.6
TABLE_TOP, NET_TOP = 0.76, 0.76 + 0.1525
GRAVITY = 9.81
LAUNCH_POS = (-1.8, 0.0, 1.0)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)


def aero_accel(v, w):
    speed = _norm(v)
    if speed < 1e-6:
        return (0.0, 0.0, 0.0)
    area = math.pi * RADIUS ** 2
    k = -0.5 * RHO * CD * area * speed
    f = [k * v[0], k * v[1], k * v[2]]
    wxv = _cross(w, v)
    n = _norm(wxv)
    if n > 1e-9:
        cl = min(LIFT_SLOPE * RADIUS * _norm(w) / speed, LIFT_MAX)
        s = 0.5 * RHO * cl * area * speed * speed / n
        f = [f[i] + s * wxv[i] for i in range(3)]
    return (f[0] / MASS, f[1] / MASS, f[2] / MASS - GRAVITY)


def spin_vector(phi, spin_top, spin_side):
    """Topspin axis is horizontal and perpendicular to the travel direction (phi = azimuth)."""
    return (-math.sin(phi) * spin_top, math.cos(phi) * spin_top, spin_side)


def first_bounce(theta, phi, speed, spin_top, spin_side, dt=5e-4):
    """Returns (x, y, net_clearance) at the first table contact, or None if it never lands."""
    v = [speed * math.cos(theta) * math.cos(phi), speed * math.cos(theta) * math.sin(phi),
         speed * math.sin(theta)]
    w = spin_vector(phi, spin_top, spin_side)
    p = list(LAUNCH_POS)
    clearance = None
    for _ in range(int(3.0 / dt)):
        a = aero_accel(v, w)
        v = [v[i] + a[i] * dt for i in range(3)]
        px = p[0]
        p = [p[i] + v[i] * dt for i in range(3)]
        if px < 0.0 <= p[0]:
            clearance = p[2] - RADIUS - NET_TOP
        if v[2] < 0.0 and p[2] - RADIUS <= TABLE_TOP:
            return p[0], p[1], clearance
        if p[2] < 0.0:
            return None
    return None


def solve(speed, tx, ty, spin_top=0.0, spin_side=0.0, net_margin=0.01):
    """Finds (theta, phi) landing the first bounce at (tx, ty). Returns (theta, phi, err) or None."""
    phi0 = math.atan2(ty - LAUNCH_POS[1], tx - LAUNCH_POS[0])
    best = None
    for theta0 in (0.02, 0.1, 0.2, 0.3, 0.45, 0.6, -0.05, -0.15):
        th, ph = theta0, phi0
        for _ in range(30):
            r = first_bounce(th, ph, speed, spin_top, spin_side)
            if r is None:
                break
            ex, ey = r[0] - tx, r[1] - ty
            if math.hypot(ex, ey) < 2e-3:
                break
            h = 1e-3
            r1 = first_bounce(th + h, ph, speed, spin_top, spin_side)
            r2 = first_bounce(th, ph + h, speed, spin_top, spin_side)
            if r1 is None or r2 is None:
                break
            a, c = (r1[0] - r[0]) / h, (r1[1] - r[1]) / h
            b, d = (r2[0] - r[0]) / h, (r2[1] - r[1]) / h
            det = a * d - b * c
            if abs(det) < 1e-9:
                break
            th -= max(-0.1, min(0.1, (d * ex - b * ey) / det))
            ph -= max(-0.1, min(0.1, (-c * ex + a * ey) / det))
        else:
            r = first_bounce(th, ph, speed, spin_top, spin_side)
        r = first_bounce(th, ph, speed, spin_top, spin_side)
        if r is None:
            continue
        err = math.hypot(r[0] - tx, r[1] - ty)
        if err < 0.01 and r[2] is not None and r[2] > net_margin:
            if best is None or th < best[0]:
                best = (th, ph, err)
    return best


def launch_velocity(speed, theta, phi, spin_top, spin_side):
    v = (speed * math.cos(theta) * math.cos(phi), speed * math.cos(theta) * math.sin(phi),
         speed * math.sin(theta))
    return v, spin_vector(phi, spin_top, spin_side)


def main_ros():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    class BallMachine(Node):
        def __init__(self):
            super().__init__('ball_machine')
            d = self.declare_parameter
            d('speed', 9.0)          # [m/s]
            d('speed_jitter', 0.0)   # +- random speed [m/s]
            d('spin_max', 200.0)     # [rad/s] random topspin/backspin in -max..max, sidespin +-max/2
            d('x_min', 0.6)          # random landing zone on the robot half [m] (x_min..x_max)
            d('x_max', 1.25)
            d('y_max', 0.70)         # landing y is uniform in -y_max..y_max (table half width 0.76)
            d('miss_percent', 0.0)   # % of shots aimed at a point OFF the table
            d('period', 2.5)         # [s] between shots
            d('start_delay', 5.0)    # [s] wait for the simulator
            d('count', 0)            # 0 = fire forever
            self.fired = 0
            self.last_fire = None   # sim time of the previous shot
            self.pub = self.create_publisher(Twist, '/ball/launch', 10)
            self.timer = self.create_timer(self.get_parameter('start_delay').value, self.first)

        def first(self):
            self.timer.cancel()
            self.timer = self.create_timer(self.get_parameter('period').value, self.fire)
            self.fire()

        def pick_target(self):
            g = lambda n: self.get_parameter(n).value
            if random.uniform(0.0, 100.0) < g('miss_percent'):
                # off the table: past the far end, or beyond a side edge
                if random.random() < 0.5:
                    return random.uniform(1.5, 1.9), random.uniform(-0.9, 0.9), True
                side = random.choice((-1.0, 1.0))
                return (random.uniform(g('x_min'), g('x_max')),
                        side * random.uniform(0.9, 1.2), True)
            return (random.uniform(g('x_min'), g('x_max')),
                    random.uniform(-g('y_max'), g('y_max')), False)

        def fire(self):
            g = lambda n: self.get_parameter(n).value
            # There is one ball: a launch teleports it back to the machine. Timers on sim time fire in
            # a burst after a simulator stall, which would cut the previous ball's flight short.
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.last_fire is not None and now - self.last_fire < 0.8 * g('period'):
                self.get_logger().info(
                    f'skipped a shot: only {now - self.last_fire:.1f} s of sim time since the last one')
                return
            self.last_fire = now
            speed = g('speed') + random.uniform(-1, 1) * g('speed_jitter')
            for _ in range(6):
                tx, ty, miss = self.pick_target()
                top = random.uniform(-1, 1) * g('spin_max')
                side = random.uniform(-1, 1) * g('spin_max') / 2.0
                sol = solve(speed, tx, ty, top, side)
                if sol is not None:
                    break
            else:
                self.get_logger().warn(
                    f'no launch found for speed={speed:.1f} spin_max={g("spin_max")}; '
                    'try a lower x_min, lower spin_max or other speed')
                return
            v, w = launch_velocity(speed, sol[0], sol[1], top, side)
            m = Twist()
            m.linear.x, m.linear.y, m.linear.z = v
            m.angular.x, m.angular.y, m.angular.z = w
            self.pub.publish(m)
            self.fired += 1
            self.get_logger().info(
                f'shot {self.fired}: speed={speed:.1f} elev={math.degrees(sol[0]):.1f}deg '
                f'azim={math.degrees(sol[1]):.1f}deg top={top:.0f} side={side:.0f} rad/s -> aim ({tx:.2f},{ty:.2f}) '
                f'{"OFF TABLE" if miss else "on table"}')
            if g('count') and self.fired >= g('count'):
                self.timer.cancel()

    rclpy.init()
    node = BallMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--solve':
        a = [float(x) for x in sys.argv[2:]] + [0.0] * 5
        s = solve(a[0], a[1], a[2], a[3], a[4])
        if s is None:
            print('no solution')
        else:
            v, w = launch_velocity(a[0], s[0], s[1], a[3], a[4])
            print('elev %.2f deg azim %.2f deg err %.1f mm' % (
                math.degrees(s[0]), math.degrees(s[1]), s[2] * 1000))
            print('tools/fire_ball.sh %.3f %.3f %.3f %.1f %.1f %.1f' % (*v, *w))
            print('landing', first_bounce(s[0], s[1], a[0], a[3], a[4])[:2])
    else:
        main_ros()
