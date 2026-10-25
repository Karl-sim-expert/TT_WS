#!/usr/bin/env python3
"""Publishes the table and the net as MoveIt collision objects (world frame).

The rail is part of the URDF already. Dimensions follow the Gazebo world (ITTF table).
"""
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

TABLE_TOP = 0.76


def box(name, size, center):
    obj = CollisionObject()
    obj.header.frame_id = 'world'
    obj.id = name
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(v) for v in size]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in center)
    pose.orientation.w = 1.0
    obj.primitives.append(prim)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


class SceneObjects(Node):
    def __init__(self):
        super().__init__('scene_objects')
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.count = 0
        # Repeated for a while: move_group may not be listening yet when we start.
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [
            box('table', (2.74, 1.525, 0.03), (0.0, 0.0, TABLE_TOP - 0.015)),
            box('net', (0.01, 1.83, 0.1525), (0.0, 0.0, TABLE_TOP + 0.07625)),
        ]
        self.pub.publish(scene)
        self.count += 1
        if self.count >= 10:
            self.timer.cancel()
            self.get_logger().info('table and net published to the planning scene')


def main():
    rclpy.init()
    node = SceneObjects()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
