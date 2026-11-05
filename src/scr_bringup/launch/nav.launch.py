"""[Pi] 실물 자율주행: 로봇(브리지·라이다·모델) + SLAM + Nav2 를 한 번에. 시뮬의 `robot up slam` 과 같은 구성.

  ros2 launch scr_bringup nav.launch.py ugv_port:=/dev/ttyAMA0

  켜지는 것                 하는 일
  robot.launch.py          브리지(odom: 자이로 + 주행거리계), 라이다 /scan, 로봇 모델 TF
  slam_toolbox (async)     /map, map → odom  (설정: 패키지 기본값 + config/slam_real.yaml)
  Nav2 (navigation_launch) 경로 계획(NavFn), 따라가기(MPPI), 속도 부드럽게, 충돌 감시
  - map_server·AMCL 없음 (지도는 SLAM 이 실시간으로)

주의
- 켜고 3초 동안 로봇을 건드리지 말 것 (자이로 영점 측정). 지도 기준 = 이때 로봇의 자리·방향.
- SLAM 중에 손으로 옮기지 말 것 (옮긴 거리는 odom 이 모름 → 지도 어긋남).
- Nav2 설정 = repo 의 config/nav2_burger_sim.yaml + nav2_slam_mode.yaml + nav2_ugv02.yaml + nav2_explore.yaml 을 합친 것
  (nav2_explore = 탐색 모드: 도착 30 cm, 방향 안 봄)
  (합친 결과와 바뀐 값 목록: /tmp/nav2_ugv02_effective.yaml, 켤 때 화면에 [설정] 줄로 출력).
- 끌 때(Ctrl+C) 주행 보고서가 자동으로 만들어짐 → ~/scr_logs/reports/, ntfy 로 요약 한 줄.
  (Mac: robot-pull 로 가져오기. 설계: docs/design/logging.md)
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

REPO = os.path.expanduser("~/semantic-companion-robot")   # 설정 파일이 repo 에 있음 (설치 안 함 → 고치면 바로 반영)


def merged_params():
    """원본 → SLAM 모드 → 실물 순서로 덮어써서 한 파일로."""
    cfg = os.path.join(REPO, "config")
    merge = os.path.join(REPO, "scripts", "merge_params.py")
    step1, out = "/tmp/nav2_ugv02_step1.yaml", "/tmp/nav2_ugv02_effective.yaml"
    subprocess.run(["python3", merge, os.path.join(cfg, "nav2_burger_sim.yaml"),
                    os.path.join(cfg, "nav2_slam_mode.yaml"), step1], check=True, capture_output=True)
    step2 = "/tmp/nav2_ugv02_step2.yaml"
    r = subprocess.run(["python3", merge, step1, os.path.join(cfg, "nav2_ugv02.yaml"), step2],
                       check=True, capture_output=True, text=True)
    print(r.stdout)
    # 11-05: SLAM 중엔 탐색·투어링 모드 = 느슨한 도착 (config/nav2_explore.yaml 설명). 순회(엄격)는 지도 완성 후 따로.
    r = subprocess.run(["python3", merge, step2, os.path.join(cfg, "nav2_explore.yaml"), out],
                       check=True, capture_output=True, text=True)
    print(r.stdout)
    return out


def merged_slam_params():
    """slam_toolbox 패키지 기본값 → config/slam_real.yaml (실물용) 순서로 덮어써서 한 파일로."""
    base = os.path.join(get_package_share_directory("slam_toolbox"), "config", "mapper_params_online_async.yaml")
    out = "/tmp/slam_real_effective.yaml"
    r = subprocess.run(["python3", os.path.join(REPO, "scripts", "merge_params.py"), base,
                        os.path.join(REPO, "config", "slam_real.yaml"), out],
                       check=True, capture_output=True, text=True)
    print(r.stdout)
    return out


def report_after_shutdown(context, *args, **kwargs):
    """주행이 끝나면 보고서 + 알림. 노드들이 로그를 다 쓸 때까지 8초 기다린 뒤 실행.
    start_new_session: launch 가 끝나도, tmux 의 Ctrl+C 가 와도 이 작업은 따로 살아서 끝까지 돈다."""
    cmd = (f"mkdir -p ~/scr_logs/reports; sleep 8; "
           f"python3 {REPO}/scripts/run_report.py ~/.ros/log --pending --notify --prune 7 "
           f"--out ~/scr_logs/reports >> ~/scr_logs/report_hook.log 2>&1")
    subprocess.Popen(["bash", "-c", cmd], start_new_session=True)
    return []


def generate_launch_description():
    bringup = get_package_share_directory("scr_bringup")
    nav2 = get_package_share_directory("nav2_bringup")
    slam = get_package_share_directory("slam_toolbox")
    params = merged_params()
    slam_params = merged_slam_params()

    return LaunchDescription([
        DeclareLaunchArgument("ugv_port", description="UGV02 시리얼 포트 (실물: /dev/ttyAMA0)"),
        # 11-03 기어가기: 브리지가 모터를 직접 제어 (v5). config/nav2_ugv02.yaml 의 저속 상한과 짝.
        #   drive_mode:=pid 로 예전 방식 (그땐 nav2_ugv02.yaml 속도도 10-31 값으로 되돌려야 덜컹 안 함)
        DeclareLaunchArgument("drive_mode", default_value="pwm"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "robot.launch.py")),
            launch_arguments={"ugv_port": LaunchConfiguration("ugv_port"),
                              "drive_mode": LaunchConfiguration("drive_mode")}.items()),

        # 브리지가 자이로 영점(3 s)을 잰 뒤 odom 을 내기 시작 → SLAM·Nav2 는 5 s 뒤에
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam, "launch", "online_async_launch.py")),
                launch_arguments={"use_sim_time": "false", "slam_params_file": slam_params}.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "navigation_launch.py")),
                launch_arguments={"use_sim_time": "false", "params_file": params, "autostart": "true"}.items()),
        ]),

        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=report_after_shutdown)])),
    ])
