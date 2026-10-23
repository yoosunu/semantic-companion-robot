"""SLAM 모드: 지도 없이 시작해서 SLAM Toolbox가 그리는 지도 위에서 Nav2가 주행한다.

robot up slam 이 nav2 창에서 실행 (scripts/robot_nav.sh --slam).
  - slam_toolbox (online async): /scan + odom → /map, TF map→odom
  - Nav2 navigation (map_server·AMCL 없음): planner, controller(MPPI) ...
  - RViz (Nav2 기본 화면)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("nav2_bringup")
    slam = get_package_share_directory("slam_toolbox")
    # 내 RViz 화면 설정이 있으면 그걸 쓰고 (config/nav2.rviz — RViz에서 File > Save Config As로 저장), 없으면 Nav2 기본 화면
    my_rviz = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "config", "nav2.rviz"))
    rviz_config = my_rviz if os.path.exists(my_rviz) else os.path.join(bringup, "rviz", "nav2_default_view.rviz")
    nav_params = LaunchConfiguration("nav_params")
    slam_params = LaunchConfiguration("slam_params")

    return LaunchDescription([
        DeclareLaunchArgument("nav_params", description="Nav2 설정 (원본 + my_nav2.yaml)"),
        DeclareLaunchArgument("slam_params", description="SLAM Toolbox 설정 (원본 + my_slam.yaml)"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam, "launch", "online_async_launch.py")),
            launch_arguments={"use_sim_time": "true", "slam_params_file": slam_params}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "navigation_launch.py")),
            launch_arguments={"use_sim_time": "true", "params_file": nav_params, "autostart": "true"}.items(),
        ),
        Node(
            package="rviz2", executable="rviz2", output="log",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
        ),
    ])
