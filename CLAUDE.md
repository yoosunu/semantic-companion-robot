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

## 현재 상태 (2026-10-11 종료 시점) — 시뮬레이션 v0.1 통과
- 계층 상태: 센서 ✅ / SLAM 맵 ✅ / 위치 추정(AMCL 2.0cm) ✅ / Nav2 자율주행(SUCCEEDED, 7.2cm) ✅ → `docs/validation/sim-v0.1.md`
- 실행: `robot-start` → `./scripts/robot_nav.sh` → `./scripts/nav_test.sh` (Nav2 설정: `config/nav2_burger_sim.yaml`)
  맵 생성: `robot-start` → `robot-slam` → `robot-drive` → `./scripts/save_map.sh <name>`
- 빌드: `./scripts/setup_workspace.sh` → ~/scr_ws (src는 repo/src 심볼릭 링크). 환경: `scripts/env.sh`.
- 결정: 시뮬레이션 튜닝은 여기서 멈춤. Burger 파라미터는 실물 UGV02로 옮겨가지 않으므로, 실물 bring-up하며 개념을 채운다.
  시뮬레이션은 회귀 테스트·실험용으로 유지.
- 하드웨어 주문 완료(2026-10-12): 엘레파츠 품목(Pi 5 포함)은 합배송으로 10/26 발송 예정. microSD(쿠팡)만 먼저 도착.
- 다음 할 일 (하드웨어 도착 전):
  1. 로봇의 임무("단순히 돌아다니는 게 아니라 ~를 수행") 정의 → semantic 계층 인터페이스 설계
  2. UGV02 JSON/UART 프로토콜 정리 → 브리지 설계(Jasper 직접 작성) + 가짜 ESP32(시리얼 에뮬레이터)로 하드웨어 없이 테스트
  3. Pi 준비: microSD에 Ubuntu 24.04 미리 굽기(Mac, Raspberry Pi Imager) + Pi 설치 스크립트(scripts/setup_pi.sh) 작성 → 도착 당일 바로 실행
  4. bring-up 계층 순서: Pi 환경 → LiDAR /scan → UGV02 모터/odom/IMU → TF → SLAM → Nav2
- iPhone: iPhone 16 (일반 모델, LiDAR 스캐너 없음) → 기하(갈 수 있는 공간)는 RPLIDAR + costmap, iPhone은 의미(YOLO) 담당.
- 보류 실험: iPhone 높이별(20/30/40cm) YOLO 탐지율 비교 → 거치 높이 결정 근거. 공간 확보 후 진행.
- 구매 기준서: docs/hardware-purchase-checklist.md (배터리 미포함 가능성, UGV02 5V 출력 전류 미표기 등)
- 후보(나중): 자율 탐색(frontier exploration) — 로봇이 스스로 돌아다니며 맵 완성. 직접 작성 후보.
- 알려진 불편: tmux 안에서 robot-stop 실행 시 같이 붙여넣은 명령이 사라짐.
- 열린 결정: SLAM은 실물에서 SLAM Toolbox 사용 예정(시뮬레이션은 Cartographer 맵 사용 중).

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

