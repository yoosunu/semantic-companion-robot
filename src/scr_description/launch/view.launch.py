"""[VM] 로봇 모델 눈으로 확인: robot_state_publisher + RViz.

  ros2 launch scr_description view.launch.py
RViz 에서: Fixed Frame = base_footprint, Add → RobotModel (Description Topic: /robot_description), Add → TF
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory("scr_description"), "urdf", "ugv02.urdf.xacro")
    description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="rviz2", executable="rviz2", output="log"),
    ])
