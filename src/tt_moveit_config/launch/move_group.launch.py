"""MoveIt 2 move_group for the Panda table tennis arm (sim time), plus table/net collision objects.

Start the simulation first (ros2 launch tt_bringup sim.launch.py), then in another terminal:
  ros2 launch tt_moveit_config move_group.launch.py            # move_group + scene objects
  ros2 launch tt_moveit_config move_group.launch.py rviz:=true # + RViz2 with the MotionPlanning panel
Or run it together with the simulation:  ros2 launch tt_bringup sim.launch.py moveit:=true
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    share = get_package_share_directory('tt_moveit_config')
    description = get_package_share_directory('tt_description')

    robot_description = {'robot_description': ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(description, 'urdf', 'tt_robot.urdf.xacro'),
    ]), value_type=str)}
    with open(os.path.join(share, 'config', 'tt_robot.srdf')) as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    kinematics = {'robot_description_kinematics': load_yaml(
        os.path.join(share, 'config', 'kinematics.yaml'))}
    joint_limits = {'robot_description_planning': load_yaml(
        os.path.join(share, 'config', 'joint_limits.yaml'))}
    ompl = {'move_group': load_yaml(os.path.join(share, 'config', 'ompl_planning.yaml'))}
    controllers = load_yaml(os.path.join(share, 'config', 'moveit_controllers.yaml'))
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }
    scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    common = [robot_description, robot_description_semantic, kinematics, joint_limits,
              {'use_sim_time': True}]

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=common + [ompl, controllers, trajectory_execution, scene_monitor])

    scene_objects = Node(
        package='tt_moveit_config', executable='scene_objects.py', output='screen',
        parameters=[{'use_sim_time': True}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(share, 'rviz', 'moveit.rviz')],
        parameters=common + [ompl])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        move_group, scene_objects, rviz,
    ])
