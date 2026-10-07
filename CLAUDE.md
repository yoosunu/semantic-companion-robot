# CLAUDE.md — 작업 맥락 (Claude가 매 세션 먼저 읽는 파일)

## 프로젝트
Semantic Companion Robot — 바퀴형 모바일 로봇 자율주행 + iPhone 기반 semantic perception.
Jasper의 대학 마지막 프로젝트. public GitHub repo이자 포트폴리오.

## 작업 구조
- Mac `~/Projects/semantic-companion-robot` = VM `~/semantic-companion-robot` (같은 폴더)
  - QEMU VirtFS(9p) → `/mnt/utm`, bindfs로 uid 501→1000(jasper) 매핑, fstab 자동 마운트
- Claude는 Mac 쪽에서 파일을 읽고 수정. ROS/Gazebo 실행은 VM에서 Jasper가 함.
- 실행 결과는 가능하면 `logs/`에 파일로 남겨 Claude가 직접 읽게 함.
- 명령 안내 시 항상 `[Mac]` / `[VM]` / `[Pi]` 표시.

## 배포 흐름
Repo: https://github.com/yoosunu/semantic-companion-robot (public)
Claude ⇄ 공유 폴더 ⇄ VM (개발/시뮬레이션) → git push → GitHub (`main` = 검증된 버전, `dev` = 작업 중)
→ Pi에서 clone/pull → colcon build → 로컬 독립 실행 (Mac/인터넷 없이 동작해야 함)

## 개발 환경
- MacBook Air M1 16GB → UTM(QEMU) → Ubuntu 24.04 ARM64 (4 cores / 6GB) → ROS 2 Jazzy → Gazebo Harmonic
- TurtleBot3 Jazzy 소스: VM `~/turtlebot3_ws` (repo 밖, 외부 의존성)
  ```bash
  source /opt/ros/jazzy/setup.bash
  source ~/turtlebot3_ws/install/setup.bash
  export TURTLEBOT3_MODEL=burger
  ```

## 알려진 환경 이슈
- Apple Silicon + UTM에서 Gazebo Ogre2 renderer segfault → `--render-engine ogre`로 해결.
  현재 `turtlebot3_gazebo`의 `empty_world.launch.py`, `turtlebot3_world.launch.py`를 직접 수정한 상태.
  → TODO: 이 repo의 자체 launch로 감싸서 원본 수정 없이 관리.
- Gazebo bridge의 `/cmd_vel`은 `geometry_msgs/msg/TwistStamped`. `Twist`로 publish하면 로봇이 안 움직임.

## 현재 상태 (2026-10-07 종료 시점)
- 시뮬레이션은 repo의 `scr_gazebo` 패키지로 실행: `ros2 launch scr_gazebo sim_world.launch.py` (robot-start가 사용).
  빌드: `./scripts/setup_workspace.sh` → ~/scr_ws (src는 repo/src 심볼릭 링크). 환경: `scripts/env.sh`.
- 디버깅 001: sim LiDAR가 전부 range_min(0.12) → 원인: VM(Ogre1+GL2.1)에서 gpu_lidar 맨 위 행 깊이 읽기 실패.
  수정: scr_burger 모델 LiDAR 수직 3줄(±0.5°), bridge가 가운데 행 사용. 정지 상태 회귀 테스트 통과.
  turtlebot3_ws launch 수정은 원복 완료.
- 계층 상태: 센서 ✅ / SLAM 맵(정지) ✅ / SLAM 맵(주행 전체) ⏳
- 다음 할 일:
  1. safe_teleop 개선 (키 누르는 동안만 이동, 입력 누적 방지, 가감속, 속도 0.20 이하) — 0.30 m/s + 펄스 방식으로 주행 중 로봇 전복
  2. 한 바퀴 주행 후 맵이 육각형 + 기둥 9개와 일치하는지 검증, map_saver로 저장
  3. 001 기록 정리 (증거 파일 docs/debugging/001/로 보존, docs/debugging/README.md 목록)
  4. SLAM 도구 결정 (Cartographer vs SLAM Toolbox) → Nav2
- 알려진 불편: tmux 안에서 robot-stop 실행 시 같이 붙여넣은 명령이 사라짐 → 개선 예정
- 열린 결정: 하드웨어 기준 문서는 SLAM Toolbox 우선, 시뮬레이션은 현재 Cartographer.

## 하드웨어 (구매 전) — 상세: `docs/hardware-baseline.md`
UGV02(내장 ESP32: 엔코더/IMU/모터 제어, Pi와 UART/JSON) · Raspberry Pi 5 8GB · RPLIDAR C1 Dev Kit(USB) · iPhone(ARKit/CoreML)

## 작업 원칙
- 목표 중심. 필요한 ROS 2 개념만 그때 설명.
- 한 번에 너무 많은 명령 주지 않기.
- 추측으로 설정 바꾸지 않기. 로그/코드/실행 순서로 원인 분리.
- 디버깅 기록 형식: 증상 → 가설 → 실험/확인 → 로그 → 원인 → 수정 → regression test/결과
- public repo: 비밀 정보(Wi-Fi, 토큰, IP) 커밋 금지. build/install/log, rosbag 커밋 금지.

## 코드 작성 분담 (2026-10-07 결정)
- 1단계 "먼저 돌아가게": 기반 기능(시뮬레이션, 설정, 스크립트, 실험 도구, 초기 노드)은 Claude와 함께 빠르게 작성.
- 2단계 "내 코드로 교체": 핵심 부분은 Jasper가 직접 작성, Claude는 리뷰·질문·테스트 설계.
  - 대상: UGV02 C++ 드라이버(hardware interface), semantic world model, task/safety manager, 카메라 거리 추정 vs LiDAR 평가.
- 1단계 코드도 토픽/메시지 인터페이스를 명확히 해서, 2단계 코드가 같은 자리에 끼워지도록 설계.
- 교체 전후를 수치로 비교해 기록 (예: 주기, 지연, 오차, 실패 처리).

## 통합 원칙 (2026-10-07)
- 한 번에 한 계층만 붙인다. 아래 계층이 합격 기준을 통과하고 증거가 남은 뒤에만 다음 계층을 올린다.
  예: 센서(/scan 값) → TF/odom → SLAM 맵 → 위치추정 → Nav2 → 인식 → semantic → 작업
- 각 계층의 합격 기준은 수치로 정하고, 점검은 스크립트로 자동화해 회귀 테스트로 재사용한다 (예: scripts/slam_check.sh).
- 문제가 생기면 이미 통과한 계층부터 재점검해서 범위를 좁힌다.

