#!/usr/bin/env python3
"""RViz2 helper: draws the table, net and the ball (from /model/ball/odometry)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


def box(mid, pos, size, rgba):
    m = Marker()
    m.header.frame_id = 'world'
    m.ns = 'scene'
    m.id = mid
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = size
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    return m


class SceneMarkers(Node):
    def __init__(self):
        super().__init__('scene_markers')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_pub = self.create_publisher(MarkerArray, 'tt_viz/scene', latched)
        self.ball_pub = self.create_publisher(Marker, 'tt_viz/ball', 10)
        self.create_subscription(Odometry, '/model/ball/odometry', self.on_ball, 10)

        scene = MarkerArray()
        # ITTF table: 2.74 x 1.525 m, top surface at z = 0.76 m, 30 mm thick.
        scene.markers.append(box(0, (0.0, 0.0, 0.745), (2.74, 1.525, 0.03), (0.05, 0.25, 0.5, 1.0)))
        # Net: 1.83 m wide, 152.5 mm high, at x = 0.
        scene.markers.append(box(1, (0.0, 0.0, 0.836), (0.01, 1.83, 0.1525), (0.9, 0.9, 0.9, 0.8)))
        self.static_pub.publish(scene)

    def on_ball(self, msg: Odometry):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = msg.header.stamp
        m.ns = 'ball'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = msg.pose.pose
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0
        self.ball_pub.publish(m)


def main():
    rclpy.init()
    node = SceneMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
