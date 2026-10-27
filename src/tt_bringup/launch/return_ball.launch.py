"""M3/M4 demo: ball machine fires, the robot predicts the ball and returns it (ground-truth ball state).
M4: the stroke type and the landing target are chosen from the incoming ball (strategy:=rules);
strategy:=fixed returns every ball to target_x/target_y as in M3.

  ros2 launch tt_bringup return_ball.launch.py
  ros2 launch tt_bringup return_ball.launch.py machine_speed:=6.0 machine_period:=5.0 target_x:=-0.7
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
        launch_arguments=[
            ('rviz', LaunchConfiguration('rviz')), ('ball_machine', 'true'),
            ('machine_speed', LaunchConfiguration('machine_speed')),
            ('machine_spin_max', LaunchConfiguration('machine_spin_max')),
            ('machine_period', LaunchConfiguration('machine_period')),
            ('machine_x_min', LaunchConfiguration('machine_x_min')),
            ('machine_miss_percent', '0.0')])

    executor = Node(
        package='tt_motion', executable='stroke_executor', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True,
                     'vel_scale': num('vel_scale'),
                     # the ball arrives within ~0.7 s: shorter backswing and faster preparation than M2
                     'min_move_time': 0.2, 'prep_vel_frac': 0.9,
                     'd_back': 0.08, 'd_follow': 0.10, 'go_ready_on_start': True,
                     'tracking_lag': 0.040}])

    returner = Node(
        package='tt_motion', executable='return_ball', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True, 'target_x': num('target_x'),
                     'target_y': num('target_y'), 'min_lead': num('min_lead'),
                     'strategy': LaunchConfiguration('strategy')}])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('vel_scale', default_value='1.0'),
        # Ball machine. 6 m/s is the slowest speed with a net-clearing shot (flight ~0.65 s).
        DeclareLaunchArgument('machine_speed', default_value='6.0'),
        DeclareLaunchArgument('machine_spin_max', default_value='0.0'),   # start without spin
        DeclareLaunchArgument('machine_period', default_value='5.0'),
        DeclareLaunchArgument('machine_x_min', default_value='0.9'),
        # 'rules' = M4 stroke/target selection from the incoming ball, 'fixed' = always target_x/target_y
        DeclareLaunchArgument('strategy', default_value='rules'),
        # Fixed target for strategy:=fixed (opponent half, x < 0)
        DeclareLaunchArgument('target_x', default_value='-0.7'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('min_lead', default_value='0.6'),           # s after launch
        sim, executor, returner,
    ])
