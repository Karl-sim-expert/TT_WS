"""M2 demo: simulation + stroke executor + a test that hits a parked (motionless) ball.

  ros2 launch tt_bringup static_hit.launch.py
  ros2 launch tt_bringup static_hit.launch.py speed:=3.0 elevation_deg:=30 repeat:=5
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('tt_bringup')
    description = get_package_share_directory('tt_description')

    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(description, 'urdf', 'tt_robot.urdf.xacro'),
    ]), value_type=str)

    def num(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments=[('rviz', LaunchConfiguration('rviz'))])

    executor = Node(
        package='tt_motion', executable='stroke_executor', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True,
                     'vel_scale': num('vel_scale')}])

    test = Node(
        package='tt_motion', executable='hit_static_ball', output='screen',
        parameters=[{
            'robot_description': robot_description, 'use_sim_time': True,
            'ball_x': num('ball_x'), 'ball_y': num('ball_y'), 'ball_z': num('ball_z'),
            'speed': num('speed'), 'elevation_deg': num('elevation_deg'),
            'yaw_deg': num('yaw_deg'), 'lead': num('lead'),
            'random_pos': ParameterValue(LaunchConfiguration('random_pos'), value_type=bool),
            'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
            'x_min': num('x_min'), 'x_max': num('x_max'), 'y_min': num('y_min'),
            'y_max': num('y_max'), 'z_min': num('z_min'), 'z_max': num('z_max'),
            'repeat': ParameterValue(LaunchConfiguration('repeat'), value_type=int)}])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        # Multiplier on all joint velocity limits (rail 4 m/s, Panda 2.4-2.9 rad/s). 1.0 = as built;
        # >1 is a sim-only 'virtual arm' for stronger hits.
        DeclareLaunchArgument('vel_scale', default_value='1.0'),
        DeclareLaunchArgument('ball_x', default_value='1.15'),
        DeclareLaunchArgument('ball_y', default_value='0.0'),
        DeclareLaunchArgument('ball_z', default_value='0.95'),
        DeclareLaunchArgument('speed', default_value='5.0'),          # requested ball speed [m/s]
        DeclareLaunchArgument('elevation_deg', default_value='25.0'),
        DeclareLaunchArgument('yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('lead', default_value='4.0'),           # s from plan to impact
        DeclareLaunchArgument('repeat', default_value='1'),
        # random_pos:=true samples the parked ball in the box below on every repeat (seed 0 = new
        # random each launch). The default box is where the 5 m/s hit is reachable.
        DeclareLaunchArgument('random_pos', default_value='false'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('x_min', default_value='1.0'),
        DeclareLaunchArgument('x_max', default_value='1.4'),
        DeclareLaunchArgument('y_min', default_value='-0.2'),
        DeclareLaunchArgument('y_max', default_value='0.2'),
        DeclareLaunchArgument('z_min', default_value='0.85'),
        DeclareLaunchArgument('z_max', default_value='1.15'),
        sim, executor, test,
    ])
