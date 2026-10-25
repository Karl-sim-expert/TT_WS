import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory('tt_gazebo')
    description_share = get_package_share_directory('tt_description')
    world = os.path.join(gazebo_share, 'worlds', 'table_tennis.sdf')

    # Parent dir of the Panda description share dir: lets Gazebo resolve the
    # model://moveit_resources_panda_description/... mesh URIs.
    panda_parent = os.path.dirname(
        get_package_share_directory('moveit_resources_panda_description'))
    resource_path = os.pathsep.join([os.path.join(gazebo_share, 'models'), panda_parent])

    # gz_args: '-r' starts running; add '-s' for server-only (no GUI) runs.
    # ogre2 crashes under WSLg, so default to the ogre (1.x) render engine.
    gz_args = LaunchConfiguration('gz_args')
    cameras = LaunchConfiguration('cameras')
    rviz = LaunchConfiguration('rviz')

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
                   '-name', 'ball', '-x', '-0.8', '-y', '0', '-z', '1.2'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/ball/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ])

    # Lightweight viewer: rviz:=true (pair with gz_args:="-r -s --render-engine ogre"
    # to run Gazebo without its GUI).
    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(
            get_package_share_directory('tt_bringup'), 'rviz', 'tt.rviz')],
        parameters=[{'use_sim_time': True}])
    scene_markers = Node(
        package='tt_bringup', executable='scene_markers.py', output='screen',
        condition=IfCondition(rviz), parameters=[{'use_sim_time': True}])

    # Cameras are opt-in (cameras:=true): two 120 Hz cameras are expensive to render.
    camera_nodes = []
    for name, y in (('camera_a', '0.4'), ('camera_b', '-0.4')):
        camera_nodes.append(Node(
            package='ros_gz_sim', executable='create', output='screen',
            condition=IfCondition(cameras),
            arguments=['-file', os.path.join(gazebo_share, 'models', name, 'model.sdf'),
                       '-name', name + '_rig', '-x', '-2.5', '-y', y, '-z', '1.4',
                       '-P', '0.12']))
        camera_nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', output='screen',
            condition=IfCondition(cameras),
            arguments=[f'/{name}/image_raw@sensor_msgs/msg/Image[gz.msgs.Image']))

    load_jsb = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster'])
    load_arm = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['panda_arm_controller'])

    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='-r --render-engine ogre'),
        DeclareLaunchArgument('cameras', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        spawn_ball,
        bridge,
        *camera_nodes,
        rviz_node,
        scene_markers,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(
            target_action=load_jsb, on_exit=[load_arm])),
    ])
