import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory('tt_gazebo')
    description_share = get_package_share_directory('tt_description')
    world = os.path.join(gazebo_share, 'worlds', 'table_tennis.sdf')

    # gz_args: '-r' starts running; add '-s' for server-only (no GUI) runs.
    gz_args = LaunchConfiguration('gz_args')

    robot_description = {'robot_description': ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(description_share, 'urdf', 'tt_robot.urdf.xacro'),
    ]), value_type=str)}

    # ros_gz_sim picks the right executable (Fortress: `ign gazebo`, newer: `gz sim`).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [world, ' ', gz_args])])

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}], output='screen')

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'tt_robot'])

    spawn_ball = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-file', os.path.join(gazebo_share, 'models', 'ball', 'model.sdf'),
                   '-name', 'ball', '-x', '-1.5', '-y', '0', '-z', '1.0'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera_a/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_b/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ])

    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'], output='screen')
    load_arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'panda_arm_controller'], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='-r'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_share, 'models')),
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH', os.path.join(gazebo_share, 'models')),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        spawn_ball,
        bridge,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(
            target_action=load_jsb, on_exit=[load_arm])),
    ])
