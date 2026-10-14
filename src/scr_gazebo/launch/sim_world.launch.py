"""Start Gazebo with the TurtleBot3 world and our scr_burger robot.

Replaces `turtlebot3_gazebo turtlebot3_world.launch.py` so that we don't have to patch
the upstream TurtleBot3 sources. Differences from upstream:
  - Render engine is a launch argument (default: ogre). Ogre2 cannot run on our UTM VM
    (OpenGL 2.1 via virgl), see docs/debugging/001-slam-map-quality.md.
  - Spawns models/scr_burger (3-row LiDAR workaround) instead of turtlebot3_burger.
  - Does not depend on the TURTLEBOT3_MODEL environment variable.
  - cmd_vel_topic argument: "UGV02 mode" (scripts/ugv02_sim.sh) renames the ROS side of the
    Gazebo cmd_vel bridge to /sim/cmd_vel, so the robot only moves through our ugv02_bridge
    + scripts/fake_esp32.py, exactly like the real UGV02.
World, meshes, URDF and bridge config are reused from turtlebot3_gazebo.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('scr_gazebo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    render_engine = LaunchConfiguration('render_engine')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre',
                              description='gz render engine (ogre2 crashes on our UTM VM)'),
        DeclareLaunchArgument('world',
                              default_value=os.path.join(tb3, 'worlds', 'turtlebot3_world.world')),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel',
                              description='ROS topic Gazebo listens to for velocity. '
                                          '/sim/cmd_vel = UGV02 mode (only fake_esp32 drives the robot)'),
    ]

    # Resource paths must be set before Gazebo starts.
    #   turtlebot3_gazebo/models : world assets + turtlebot3_common meshes used by scr_burger
    #   scr_gazebo/models        : our robot model
    resource_paths = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
    ]

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v2 --render-engine ', render_engine, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-g -v2 --render-engine ', render_engine],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gui),
    )

    with open(os.path.join(tb3, 'urdf', 'turtlebot3_burger.urdf'), 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'burger',
            '-file', os.path.join(pkg, 'models', 'scr_burger', 'model.sdf'),
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
        ],
        output='screen',
    )

    # Same topics as upstream burger (clock, joint_states, odom, tf, cmd_vel[TwistStamped], imu, scan).
    def make_bridge(context):
        config = os.path.join(tb3, 'params', 'turtlebot3_burger_bridge.yaml')
        cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)
        if cmd_vel_topic not in ('cmd_vel', '/cmd_vel'):
            # UGV02 mode: same config, only the ROS name of the cmd_vel entry changes.
            with open(config, 'r') as f:
                entries = yaml.safe_load(f)
            hits = [e for e in entries if e.get('ros_topic_name') in ('cmd_vel', '/cmd_vel')]
            if len(hits) != 1:
                raise RuntimeError(f'expected exactly one cmd_vel entry in {config}, found {len(hits)}')
            hits[0]['ros_topic_name'] = cmd_vel_topic
            config = os.path.join(tempfile.gettempdir(), 'scr_gz_bridge_ugv02_mode.yaml')
            with open(config, 'w') as f:
                yaml.safe_dump(entries, f)
        return [Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', 'config_file:=' + config],
            output='screen',
        )]

    bridge = OpaqueFunction(function=make_bridge)

    return LaunchDescription(
        declare_args + resource_paths + [gz_server, gz_client, robot_state_publisher, spawn_robot, bridge]
    )
