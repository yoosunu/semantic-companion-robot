"""[Pi] 실물 로봇 켜기: 로봇 모델(TF) + 내 브리지 + 라이다. 이 위에 SLAM·Nav2 를 시뮬처럼 얹는다.

  ros2 launch scr_bringup robot.launch.py ugv_port:=/dev/ttyXXX
  ros2 launch scr_bringup robot.launch.py ugv_port:=/dev/ttyXXX use_lidar:=false   # 바퀴만 시험

  켜지는 것                    내는 것
  robot_state_publisher       TF base_footprint → base_link → laser  (scr_description 의 URDF)
  ugv02_bridge (내 코드)       /odom, TF odom → base_footprint, /battery_state  ← /cmd_vel
  rplidar (RPLIDAR C1)         /scan (frame: laser)

ugv_port 는 일부러 기본값이 없다: UGV02 가 USB 로 붙는지(/dev/ttyUSB* 또는 /dev/ttyACM*) GPIO UART(/dev/ttyAMA0) 로
붙는지 도착 날 확인해서 직접 넣는다 (docs/ugv02-protocol.md day-1). 라이다와 UGV02 가 둘 다 USB 면 이름이
꽂는 순서에 따라 바뀔 수 있음 → 나중에 udev 규칙으로 고정 이름(/dev/ugv02, /dev/lidar) 부여.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory("scr_description"), "urdf", "ugv02.urdf.xacro")
    description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    ugv_port = LaunchConfiguration("ugv_port")
    lidar_port = LaunchConfiguration("lidar_port")
    track_width = LaunchConfiguration("track_width")

    return LaunchDescription([
        DeclareLaunchArgument("ugv_port", description="UGV02(ESP32) 시리얼 포트 — 도착 날 확인"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0", description="RPLIDAR C1 포트"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("track_width", default_value="0.160",
                              description="odom 계산용 유효 바퀴 간격 [m] — 회전 보정 후 바꿀 값"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),

        Node(package="scr_ugv02", executable="ugv02_bridge", output="screen",
             parameters=[{"port": ugv_port, "baud": 115200, "track_width": track_width}]),

        # RPLIDAR C1: Slamtec sllidar_ros2 (src/sllidar_ros2 submodule). 10-28 실측: Standard, 5 kHz, 10.0 Hz, 720 beams.
        #   apt rplidar_ros 2.1.0 (SDK 1.12) 는 C1 에서 "Cannot start scan: 80008002" → 사용 안 함
        Node(package="sllidar_ros2", executable="sllidar_node", output="screen",
             condition=IfCondition(LaunchConfiguration("use_lidar")),
             parameters=[{"channel_type": "serial", "serial_port": lidar_port, "serial_baudrate": 460800,
                          "frame_id": "laser", "inverted": False, "angle_compensate": True, "scan_mode": "Standard"}]),
    ])
