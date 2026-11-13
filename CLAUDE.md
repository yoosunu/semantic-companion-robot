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
- 편집: Mac VS Code + Remote-SSH (`ssh scr-vm` = jasper@192.168.64.2, 키 인증). VS Code 터미널 = VM 터미널.
  GUI(Gazebo/RViz)는 SSH 터미널에 화면이 없으므로 VM 창에서 실행하거나 `DISPLAY=:0`을 붙여 VM 화면에 띄운다.

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

## 지금 (2026-11-06 새벽) — **RViz 실물 연결** + 탐색 모드(느슨한 도착) + 헛바퀴 위치 오류 원인·대책 + 방 지도. ← 새 대화는 여기부터
- RViz: UTM 네트워크 **브리지됨(en0)** → VM 192.168.123.118 (DHCP, MAC 7A:0E:93:96:80:C4 — 바뀌면 공유기 예약). Mac `~/.ssh/config` scr-vm.
  VM 창 터미널에서 `./tools/mac/rviz.sh` (`config/robot.rviz`: 지도·스캔·로봇·경로·로컬 비용·탐색 경계·충돌 상자·odom 자취). 대안 Foxglove(Mac 앱) 보류.
  RViz 닫을 때 저장 안 함 (설정 파일 덮어씀). 시계 차 +0.23 s = 라이다 지연(정상). GLSL ERROR = UTM 그래픽 한계, 무시.
- 탐색·투어링 모드 (`config/nav2_explore.yaml`, nav.launch 가 마지막에 덮어씀): 도착 0.20 m, 방향 안 봄, GoalAngleCritic 뺌.
  근거: 목표에서 "위치+방향 동시 맞추기" = 0.13 rad/s 미만 회전 요구 → 4분 헤맴 (재현됨). 0.30 은 탐색기와 충돌(즉시 도착 → 같은 목표 283번).
  순회 모드(엄격, 지도 완성 후 AMCL)는 따로 만들 것. 집 귀환 = Nav2 로 집 앞 40 cm → 전용 동작(회전 → 직진) `go_home.py` (예정).
- 실패한 실험 (되돌림): 벽 붙기 설정 묶음(min_range 0.13 + inflation 0.25/12 + CostCritic footprint) → 구간 2분+ 헤맴. 한 번에 여러 개 바꾼 실수.
- 몸 점 실측: 장착 상태 스캔 20장, 25 cm 안 점 0개 → obstacle/raytrace_min_range 0.18 → 0.11 (0.18 은 옆 6 cm 안 벽을 숨겼음).
- 충돌 감시: 정지 상자 방향별 (velocity_polygon 전진/후진/회전) — 가구 틈에서 후진까지 막혀 64초 갇혔던 것.
- **헛바퀴 위치 오류**: 카펫·턱에서 바퀴만 돌고(자이로 ~0, 브리지 자이로 루프가 PWM 85까지 올림) odom 거리(바퀴)가 쌓임 + SLAM 이
  패키지 기본값(0.5 m/0.5 rad 마다 보정) → 못 가는 가구 턱 너머에 로봇이 있다고 판단. 대책: `config/slam_real.yaml` (0.15 m / 0.2 rad),
  nav.launch 가 넘김. 11-06 탐색: 5개 목표, 완료 + 집 복귀 50 s, 벽 선명. 남은 것: 브리지 헛바퀴 감지(회전 명령 중 자이로 ~0 → 거리 안 쌓기·PWM 그만),
  SLAM max_laser_range 20 → 12 (C1), 흰 통로는 LiDAR 가 턱 위로 본 것 (진짜 빈 높이) → 바닥 턱은 ToF 몫.
- 탐색 끝 알림: `sim/explore_watch.py --name room` (탐색기 실행 + 복귀하면 지도 자동 저장 + ntfy, 3분 안에 복귀 못 하면 경고).
- 브리지 v5.3: TAP 은 직전 0.5 s 평균으로 결정 (MPPI 잡음 ± 은 쉼). 심박: Nav2 노드별 CPU (전부 7~17%씩, 원인 미상), 위치는 토픽 (SLAM /pose 는
  보정 때만 갱신 → 개선 필요). robot-pull 이 심박·지도도 가져옴.
- 지도: `~/maps/room_1106_explore` (Pi), Mac `logs/maps/`.
- 탐색기 블랙리스트: 30 s 동안 경계에 못 가까워지면 포기 (11-06 파랑 = 카펫 헛바퀴 중 포기). "All frontiers traversed/tried out" 은 포기 포함.
  블랙리스트는 탐색기 머릿속뿐 → 지도엔 여전히 흰색 (LiDAR 가 턱 위로 봄). → 부딪힌 자리를 지도에 남길 방법 필요.
- **다음 1순위 (Jasper 11-06): IMU 충돌 감지** — ToF 와 별개로 계속 쓸 것 (사방, 누가 건드려도). 기어가서 잡음이 작음:
  ax 표준편차 정지 ~25 / PWM 30 ~92 / 0.08 m/s 충돌 추정 ~2000+ (1 g ≈ 8500). ① 브리지 `/imu` 원본 발행 + 살살 부딪히기·밀기·카펫 기록 → 문턱값
  ② 브리지: 부딪힘(가속 튐 → 정지·방향·사건) / 막힘·헛바퀴(바퀴 돌고 IMU 조용 → 밀기 멈춤·거리 안 쌓음) / 건드림·들림(명령 0인데 IMU 움직임)
  ③ 부딪힌 자리를 장애물 지도에 찍기 (TurtleBot 범퍼 방식) ④ run_report 에 bump 사건. 그다음 go_home.py → AMCL 순회.
  → 11-06 1차 측정 완료 (브리지 v6 `/imu`, `tools/lab/imu_lab.py`): 톡 0.04 g(정지 잡음 8배), 부딪힘 0.09 g(주행 잡음 4배), 카펫 gy 7 °/s,
    출발 0.17 g 는 자기 동작 → 빼야 함. `docs/calibration/imu-touch-2026-11-06.md`. 다음 = 관찰 모드 감지기 v0 (기록만, 오탐 확인).

## 지금 (2026-11-05 저녁) — 부품 주문 완료 + "로봇의 하루" 목표 확정 + 도착 기준 0.10 (6/6). ← 새 대화는 여기부터
- 궁극 목표: `docs/design/robot-architecture.md` "로봇의 하루" (기상 깨우기·브리핑 / 외출 중 순회·지도 갱신 / 마중 / 취침 요약).
  첫 완성 장면 = **아침 기상**. 지휘자 = task_manager. 평소 운행 = 저장 지도 + AMCL (SLAM 은 지도 그릴 때만).
- 주문 (인수인계: 프로젝트 문서 `claude/handoff_purchase_2026-11-04.md`): 팬틸트 키트(9~12일), 삼성 25R ×3, 엘레파츠(ToF VL53L5CX
  SparkFun Qwiic + Adafruit 4397 + 점퍼선, 11-13 발송). 멀티미터 안 삼 → 서보 꽂기 전 사진 비교. Pico 안 삼 (직결 후 Pi I2C 3·5번 빔).
- Nav2: 도착 기준 0.10 m (0.08 은 MPPI 가 목표 앞에서 x·z ± 깔짝, 톡톡 52회) → 왕복 6/6, 평균 21.8 s, 최대 36 s. PoseProgressChecker 효과 확인.
  Pi 부하 CPU ~45%, 메모리 0.8/7.8 GB, 47°C. 배터리: 완충해도 11.8 V 근처 (켠 채) → 새 셀로 충전 경로 vs 셀 문제 가르기.
- 심박 로그: `nodes/system/heartbeat.py` (1분마다 v·위치·CPU·프로세스별 CPU·온도 → `~/scr_logs/<날짜>/heartbeat.jsonl`). 11-05 첫 시험.
- 다음: 심박 시험 → 방 지도 저장 + AMCL (SLAM↔AMCL CPU 비교) → 충돌 감시 후진 → keepout → IMU 충격 → ToF 노드 → 팬틸트 명령.
- 펌웨어 읽기 1·2장 함 (loop 구조, 우리 변경 3개 전후 비교: T:13 heartbeat / T:11 PID 끄기 / T:11 0 정지).

## 지금 (2026-11-04 새벽) — 로그 체계 v0 + 안전(heartbeat) + **기어가기(브리지 v5 PWM)** + Nav2 저속 왕복 4/4. ← 새 대화는 여기부터
- 원칙 (Jasper): **"조용하되 정확하게. 빠를 필요는 없다."** (집 안 로봇, 물류 로봇 아님)
- 로그: `docs/design/logging.md` (3층: 사건·심박·블랙박스). nav.launch 종료 시 Pi 가 보고서(`~/scr_logs/reports`) + ntfy 알림,
  부팅 시 밀린 것(crontab). Mac `robot-pull` → 최신 보고서 열기 (VS Code ⌘⇧V). 비밀값 Pi `~/.scr.env` (NTFY_TOPIC).
  `nodes/system/run_report.py` (Nav2 로그 → 시간표). 약점: 주행 중 위치 모름 → 2층 심박이 다음.
- 안전: 펌웨어 heartbeat 는 **T:1/T:11 만** 갱신 (T:13 은 안 됨 = 10-31 거실 사고 원인) → 브리지 v4 부터 T:1 + `T:136 500ms`.
  crash/hang 0.5~0.8 s 정지. 회귀 시험 `tools/bench/failsafe_test.py crash|hang [pwm]` → `docs/debugging/004-...`
- 기어가기: 펌웨어 PID 는 저속에서 가다서다(덜컹 ~450). 일정 PWM 은 잔잔(PWM 30 = 0.08 m/s, 덜컹 92) → 한계는 하드웨어 아님.
  브리지 v5 `drive_mode:=pwm` (nav.launch 기본): 직진 표 + 회전(자이로 폐루프) + 부드러운 출발 + v5.2 톡톡 밀기(바닥선 0.05 m/s / 0.13 rad/s 미만).
  측정 전부 `docs/calibration/lowspeed-2026-11-03.md`. 시험 `tools/bench/crawl_test.py [이름 일부]`.
- Nav2 (`config/nav2_ugv02.yaml`): vx 0.08, wz 0.25, 도착 0.08 m. 왕복 0.6 m+180°: 4/4, 오차 0.06~0.08 m, 17~19 s (한 번 110 s:
  진척 확인 20 s/10 cm 가 톡톡 1 cm 와 충돌) → **PoseProgressChecker 3 cm / 11° 로 바꿈, 아직 미시험** ← 다음 첫 할 일
  `python3 sim/shuttle.py --laps 2 --a 0,0,0 --b 0.6,0,180`
- 소음 1 m: 주행 45~55 dB (회전이 시끄러움 = 스키드 스티어 끌림).
- 내일 계획 (Jasper 11-04): **Waveshare 펌웨어 코드 같이 열어보기** (github waveshareteam/ugv_base_ros ROS_Driver: uart_ctrl.h 명령 분기,
  movtion_module.h heartBeatCtrl·PID·getLeftSpeed — 오늘 원인 찾은 곳) + 배터리 계획 (UGV02 위키: 보호회로 **없는** 셀 권장, UPS 보드가 보호,
  2200 mAh+ 4C — 지금 셀 KittenBot 2000 mAh 는 기준 미달) + 카메라 얘기
- 카메라 구상 (11-04 새벽 대화, 결정 전): 역할 = 의미 붙이기(의자·소파 지도 표시) + 사람 알아보기, 안전은 LiDAR 담당 → 실시간 불필요,
  LiDAR 가 "뭔가 있다" 할 때만 추론. 후보: Pi 카메라/USB 웹캠 + Pi YOLO (가속기 Hailo 는 나중) / 갤럭시 A51 상시 연결
  (전원은 Pi USB 금지 → 12V→5V 모듈로 따로, 데이터는 USB 테더링, 배터리 보호 85%) / iPhone 16 은 옵션(의존 금지).
  AR 대신: 로봇 자체가 AR (SLAM = 추적, TF = 카메라 위치, 지도 = 세계). 거리 = ① 라이다 (정답) ② 바닥 평면 투영(bbox 아래끝 → 바닥 z=0,
  SimCheong 지형 raycast 와 같은 원리) ③ (가능하면) ARCore/단안 깊이 → 라이다를 정답으로 거리추정 비교 가능.
- 다음 후보: 진척 확인 시험 → 기어가기로 방 탐색(지도 비교) → 심박(2층) → 충돌 감시 후진 막힘(VelocityPolygon) → IMU 턱 감지
  → 의자 다리 LiDAR 빔 수 실험(0.5/1/2 m) → 유리문 keepout. 미뤄둔 것: 핸드폰(YOLO) 얘기, 충전/도킹(opennav_docking), OLED 표시.
- tmux: 끄기는 `tmux send-keys -t <이름> C-c` (kill-session 은 브리지가 정지 못 하고 죽음 → 보고서에 bridge.died)

## 지금 (2026-10-31 밤) — 브리지 v3(자이로) + **실물 Nav2 + 스스로 탐색 성공** = Lv 7 핵심 달성. ← 새 대화는 여기부터
- 브리지 v3: 회전 = 자이로(15.65 단위/°/s, 시작 3 s 영점 + 정지 3 s 마다 갱신), 거리 = odl/odr(cm), 전압 V. 회전 오차 135 % → 2 % (`docs/calibration/odom-2026-10-31.md`)
- 실물 Nav2: `ros2 launch scr_bringup nav.launch.py ugv_port:=/dev/ttyAMA0` (로봇+SLAM+Nav2, 설정 `config/nav2_ugv02.yaml`: 0.2 m/s, 부드러운 가속, 몸 18 cm 무시, 충돌 감시 라이다 지연 0.5 s)
  덜컥거림 = 충돌 감시가 0.22 s 늦은 라이다를 버림 → `docs/debugging/003-nav2-stop-go-jerk.md`
- 탐색: `tmux new -d -s explore "bash -ic 'cd ~/semantic-companion-robot && ros2 run explore_lite explore --ros-args --params-file config/explore.yaml -p use_sim_time:=false; exec bash'"` → 73초에 완료
- tmux: kill-server 직후 new 는 같이 사라질 수 있음 → 나눠서. 로그 셀 땐 `capture-pane -J`
- 다음 후보: 지도 저장 → 저장 지도에서 위치 찾기(AMCL/SLAM localization), task_manager v0(Python), 1-F VM RViz, IMU 덜컹 감지, 배터리 실사용 시간, 충전(UPS 보드 확인)

## 지금 (2026-10-31 새벽) — odom 보정 1차 완료: 회전은 자이로, 거리는 odl/odr(cm). track_width 0.40 으로 실물 지도 벽 겹침 해소(3분). ← 새 대화는 여기부터
- 근거·숫자: `docs/calibration/odom-2026-10-31.md` (라이다 정답 측정 도구 `tools/bench/lidar_angle.py`, 단위 측정 `tools/bench/odom_probe.py`)
- 다음: ① 브리지 yaw 를 자이로로 (gz ÷ 16.4 °/s, 정지 시 바이어스 재측정, v ÷ 100, odl/odr cm) — Jasper 코드
  ② 완충(Pi 끄고 충전) 후 같은 자리·같은 시간 3종 지도 비교 (0.16 / 0.40 / 자이로) ③ 배터리 실사용 시간 측정 (자동 충전 기준)
- 실물 실행(지금은 수동): `tmux new -d -s robot "bash -ic 'ros2 launch scr_bringup robot.launch.py ugv_port:=/dev/ttyAMA0 track_width:=0.40; exec bash'"`
  → `tmux new -d -s slam "bash -ic 'ros2 launch slam_toolbox online_async_launch.py; exec bash'"` → wander 도 같은 방식. 끄기 `tmux send-keys -t <이름> C-c`
- Mac 으로 지도 가져오기: `scp 'jasper@scr-pi.local:~/maps/room_10*' ~/Projects/semantic-companion-robot/logs/maps/` (원격 {} 묶음 표기는 안 됨)

## 지금 (2026-10-29 새벽) — 실물 1~4단계 ✅ + 조립 완료(배터리 전원으로 Pi·라이다 동작). 다음: Day-1 2·3·6·7, 브리지 v1 파싱. ← 새 대화는 여기부터 읽기
- 실물 순서 (하나씩 붙이기, 바퀴는 띄우고, Pi는 처음엔 27W 어댑터 전원만):
  1) ✅ 10-28 Pi: Active Cooler(팬 동작 확인) · SSH · `dev` clone · `setup_pi.sh` 완료 · dialout · `ros2 pkg list | grep scr_` 4개
     막힌 3가지(OUI 필터 누락 / cloud-init instance-id·캐시 / apt noble-updates 없음) → `docs/debugging/002-pi-first-boot.md`
  2) ✅ 10-28 라이다: `/dev/ttyUSB0`(CP210x) → **sllidar_ros2**(submodule, apt rplidar_ros 는 C1 스캔 시작 실패) → `/scan` 10.0 Hz, 720빔.
     Pi USB 2.0(검은) **아래 칸 고정**. 방향(Day-1 7번)은 로봇에 단 뒤. +23°·10 cm 에 상시 물체(케이블 추정) — 장착 후 자기 몸 필터 검토.
  3) ✅ 10-28 UGV02 혼자 전원: 18650×3 (KittenBot 2000mAh, 홀더 표시대로, 가운데 칸만 반대) → 12.6V 2A 충전기로 UPS 깨움(충전기 빨강=충전 중)
     → 전원 버튼 → OLED `AP: UGV / ST: OFF / MAC / V: 12.46` → 폰 Wi-Fi `UGV`(12345678) → `192.168.4.1` CONTROL PANEL(전압 12.54, D-pad, STOP).
     켜진 채 충전 동작 확인(Day-1 4번).
     웹 패널 주행 ✅: 공중 → 바닥. 6륜 모두 돌고 손 떼면 멈춤. 제자리 회전이 눈으로 보기에 정확(미끄러짐 작아 보임 — odom 실측 전 인상).
     **방향: OLED 화면 쪽 = 뒤(엉덩이)** (웹 ▲ 기준) → URDF·라이다 장착 방향의 기준. 회전 부호(◀ = 반시계?)는 아직 기록 안 함 → 4단계에서 T:13으로.
     생각보다 빠르고 힘이 셈 → 내 코드에선 속도 상한을 낮게 시작 (브리지/Nav2 max vel).
     공식 위키 http://www.waveshare.com/wiki/UGV02 — 거꾸로 넣은 셀로 충전 금지(폭발), 10V 아래면 충전, 보호회로 없는 셀.
  4) ✅ 10-29 UGV02↔Pi: **GPIO UART `/dev/ttyAMA0`** (USB 아님). 연결 = 차체 40핀(암) → Jetson Nano Adapter (B) → 무지개 10핀 F-F → Pi GPIO 1~10.
     Pi 설정: `serial-getty@ttyAMA0` mask, cmdline.txt 에서 `console=serial0,115200` 제거(백업 cmdline.txt.bak-uart) → setup_pi.sh 5b.
     **Pi 전원도 이 케이블로 UGV02 배터리에서 받음 → 27W 어댑터 동시 연결 절대 금지.** 끌 때: Pi `sudo poweroff` 먼저 → UGV02 전원.
     피드백(T:1001) 기본 ~20Hz 스트림 → `docs/ugv02-protocol.md` "Real feedback". 어댑터 방향 180° 틀리면 5V가 TX/RX로 감 — 검은선=GND 위치로 확인.
  4.5) ✅ 10-29 조립: 마운트 판(날개 아래로) 아래에 Pi 매달기(M2.5 황동 스페이서), 판은 차체 창의 긴 황동 스페이서 3개로 받침(어댑터 때문에 4번째 불가).
     라이다는 판 위, **16248(M2.5×8+3, 수나사 3mm) 스페이서 + 15685(KM2.5×8)** 로 고정 — C1 바닥 나사 깊이 ≤4mm(데이터시트). 19024(+6)·15685 직결은 초과.
     라이다 케이블은 판 날개 밑으로 묶어 바퀴에서 뗌, Pi USB2 아래 칸. 배터리 전원에서 `throttled=0x0`, `/scan` 10.0 Hz ✅ (800mA 기동 문제 없음).
  5) 체크리스트: `docs/ugv02-protocol.md` Day-1 1~8 (피드백 형식, 좌회전 부호, 3 s 정지, 충전, 포트, URDF 실측, 라이다 방향, 라이다 드라이버)
- 10-29 05시 첫 실물 SLAM 맛보기: Pi 에서 `nohup` 으로 robot.launch.py + `slam_toolbox online_async_launch.py`, `sim/wander.py --laser-yaw 3.1416` (혼자 헤매기, 1분마다 ~/maps 저장).
  라이다 방향 실측(`wander.py --check`): 로봇 앞 = 라이다 180°, 좌우 정상 → URDF `laser_yaw=${pi}` (XML 주석에 `--` 쓰면 xacro 깨짐 — 겪음).
  1차: 사무용 의자 바닥 다리(라이다 높이 아래) 못 봄 → STOP 0.55 m. 2차: ~6.5분 돌다 SSH 끊김 → **10-31 확인: 배터리 방전으로 전원 차단** (journalctl -b -2 가 종료 과정 없이 05:35 에서 끊김, Wi-Fi 끊김·저전압 로그 없음). 10-28부터 충전 없이 사용 + Pi·라이다·모터 동시. → 주행 전 OLED V 확인, 충전은 깨어 있을 때.
  지도: Pi `~/maps/room_try1`, `~/maps/room_1029_0529` (.pgm/.yaml). 다음날 확인: `journalctl -b -1 | tail -50`, 전압(피드백 v), 지도 Mac 으로 scp.
  TODO: 실물용 `robot` 명령(지금은 nohup 수동), wander → Nav2+explore.
  결과(room_1029_0529): 방 윤곽은 잡혔지만 **같은 벽이 여러 각도로 겹침 = 회전(yaw) 누적 오차**. 원인 추정: track_width 0.160(TB3 값, UGV02 미보정·스키드 미끄러짐), L/R 단위 미확인, wander 의 잦은 제자리 회전.
  다음: odom 측정(360° 회전 → track_width, 1 m 직진 → 단위) → 필요하면 브리지 yaw 를 IMU gz 로 → 같은 방 재시도 비교.
  배터리: 보호회로 없는 셀 → 자는 동안 충전 안 함(Jasper 결정), 깨어 있을 때 충전.
- Pi 접속: Mac에서 `ssh jasper@scr-pi.local` (avahi 설치됨). 안 되면 핑 스캔 후 `arp -a | grep -i 98:fe:54` (이 Pi의 OUI).
  sudo는 비밀번호 필요(user-data `sudo: null`). 긴 작업은 `tmux new -s <이름>` 안에서 (SSH 끊겨도 계속).
  화면 없음(micro-HDMI 케이블 없음, 랜선 어려움) → Pi 안 로그가 필요하면: Mac `sudo dd if=/dev/rdiskNs2 ... count=2048` + `debugfs -c` (002 참고).
- 실물 관찰: UGV02 케이스에 확장 데크(2층 판) + 양옆 알루미늄 레일 → 1층 Pi, 2층 라이다 구조. 차체 위 40핀 단자에 `GND TX RX SCL SDA 5V` 인쇄
  (GPIO 연결 시 5V가 GPIO에 들어가지 않게 핀 배치 사진 확인 후 전원). 배터리: 18650×3 증정 + 12.6V 2A 충전기, 차체 DC 잭으로 장착 채 충전.
- Pi 다음 시작: `cd ~/semantic-companion-robot && git pull && bash setup/setup_pi.sh` (sllidar_ros2 submodule 받아 ~/scr_ws 빌드) → 확인 후 `rm -rf ~/test_ws`.
- 정리 대상: rosdep이 Pi에 `ros-jazzy-turtlebot3-gazebo` 설치(어느 package.xml 의존성인지 찾아 Pi에서 제외).
- 남은 준비성 항목: 1-F Pi↔VM 통신(UTM VM 네트워크가 Mac 안쪽 망이면 Pi 토픽을 VM RViz 에서 못 봄 → 첫 확인: VM `ip -4 addr`, Mac `ipconfig getifaddr en0`)
  / odom 측정 도구(1 m 직진·360° 회전, 11/15 게이트) / 1-B 브리지 T:131 재전송.
- 다음 내 노드: task_manager (Python 결정) — 설계 v0 `docs/design/task_manager.md`. 실물 Nav2가 돈 다음.
- 진행 방식: Jasper가 말하면 먼저 듣고 방향을 정한 뒤 진행. 명령은 [Mac]/[VM]/[Pi] 표시, **한 번에 한 동작씩**(긴 목록 금지 — 중간에 질문할 수 있게).
  물리 작업은 영상 공유가 없으므로 위치·방향·확인 방법까지 세세하게. 배선은 전원 넣기 전 사진으로 확인.
  **tmux 는 안에 들어가게 하지 말 것**: 붙여넣은 다음 줄이 사라지고, 나올 때 꺼지는 일이 반복됨 → 항상 한 줄 `tmux new -d -s <이름> "bash -ic '<명령>; exec bash'"`,
  상태 확인은 `tmux capture-pane -pt <이름> | tail -20`. (Pi Wi-Fi 절전으로 첫 SSH/scp 타임아웃 가능 → ping 후 재시도, TODO: setup_pi.sh 에서 절전 끄기)

## 현재 상태 (2026-10-14) — 시뮬 v0.1 통과, 브리지 v0 통과. 전체 계획·일정·목표 Lv: `docs/roadmap.md`
- 계층 상태: 센서 ✅ / SLAM 맵 ✅ / 위치 추정(AMCL 2.0cm) ✅ / Nav2 자율주행(SUCCEEDED, 7.2cm) ✅ → `docs/validation/sim-v0.1.md`
- 실행: **`robot` 하나** (도움말 출력). `robot up` / `robot go` / `robot edit` → `robot reload` / `robot status` / `robot down`. 구현: `sim/robot`, 사용법: `docs/sim-playbook.md`
  맵 생성(옛 방식, robot 명령에 아직 없음): `sim/robot_slam.sh` → `robot drive` → `./run/save_map.sh <name>`
- 빌드: `./setup/setup_workspace.sh` → ~/scr_ws (src는 repo/src 심볼릭 링크). 환경: `sim/env.sh`.
- 결정: 시뮬레이션 튜닝은 여기서 멈춤. Burger 파라미터는 실물 UGV02로 옮겨가지 않으므로, 실물 bring-up하며 개념을 채운다.
  시뮬레이션은 회귀 테스트·실험용으로 유지.
- 하드웨어 주문 완료(2026-10-12): 엘레파츠 품목(Pi 5 포함)은 합배송으로 10/26 발송 예정. microSD(쿠팡)만 먼저 도착.
- 다음 할 일 (하드웨어 도착 전):
  1. ✅ 로봇의 임무 정의 (2026-10-13) → 아래 "로봇 임무" 섹션
  2. **(진행 중)** `ugv02_bridge` (src/scr_ugv02): v0 ✅ 2026-10-14 (/cmd_vel→T:13 JSON, teleop으로 TB3 주행 확인, 브리지 죽으면 3 s 폭주 측정)
     v2 ✅ 2026-10-18 (Jasper 작성): 명령 끊김 3 s → 0.505 s, 종료 시 즉시 정지.
     v1 ✅ 2026-10-20 (Jasper 작성, odom 적분): /odom·TF·/battery_state. UGV02 모드는 Gazebo odom/tf를 /sim/* 로 숨김.
     미션 L1 ✅ (Nav2 3 m, 도착 4.9 cm). Nav2와 쓸 땐 브리지 `-p use_sim_time:=true` 필수. 미션 L2 ✅ (기둥 중심까지 0.388 m). 다음: L2b(yaw_goal_tolerance만 0.05로) → L3(기둥 회피).
     열린 질문: 도착 방향 L1 11°·L2 12.7° — 둘 다 yaw_goal_tolerance 0.25 rad(14.3°) 바로 안쪽 → L2b로 확인, 도착 후 AMCL 14.8 cm → A/B(일반 vs UGV02 모드)로 내 odom 탓인지 확인. 프로토콜: docs/ugv02-protocol.md (피드백 형식은 Waveshare 앱 근거 + [가정])
     실행(10-22~): `robot up` = UGV02 모드만. tmux 창 world/serial/esp32/bridge/nav2/cmd. Nav2 손잡이: `config/my_nav2.yaml`(원본 위에 덮어씀).
     옛 alias(robot-start/ugv/stop/drive)와 ugv02_sim.sh·play.sh 삭제. 일반(비 UGV02) 시뮬 모드는 더 이상 안 씀.
     Nav2 controller = MPPI (10-22 결정, 대안 RPP): docs/decisions/001-nav2-controller.md. 다음: SLAM + Nav2 동시(L6).
     폭주 테스트는 teleop 대신 `ros2 topic pub -r 10 /cmd_vel ...` (teleop은 포커스 잃으면 스스로 정지)
  3. Pi 준비: microSD에 Ubuntu 24.04 미리 굽기(Mac, Raspberry Pi Imager) + Pi 설치 스크립트(setup/setup_pi.sh) 작성 → 도착 당일 바로 실행
  4. bring-up 계층 순서: Pi 환경 → LiDAR /scan → UGV02 모터/odom/IMU → TF → SLAM → Nav2
- iPhone: iPhone 16 (일반 모델, LiDAR 스캐너 없음) → 기하(갈 수 있는 공간)는 RPLIDAR + costmap, iPhone은 의미(YOLO) 담당.
- 보류 실험: iPhone 높이별(20/30/40cm) YOLO 탐지율 비교 → 거치 높이 결정 근거. 공간 확보 후 진행.
- 구매 기준서: docs/hardware-purchase-checklist.md (배터리 미포함 가능성, UGV02 5V 출력 전류 미표기 등)
- 후보(나중): 자율 탐색(frontier exploration) — 로봇이 스스로 돌아다니며 맵 완성. 직접 작성 후보.
- tmux 안에서 `robot down`: tmux run-shell로 넘겨서 처리 (옛 robot-stop 문제 해결).
- 열린 결정: SLAM은 실물에서 SLAM Toolbox 사용 예정(시뮬레이션은 Cartographer 맵 사용 중).

## 로봇 임무 (2026-10-13 결정) — 전체 단계: `docs/roadmap.md`
- 면접 문장: "SLAM으로 맵을 그리고, Nav2로 이동하면서, 비전과 직접 짠 알고리즘으로 어디로 갈지 스스로 판단합니다.
  그리고 단순히 돌아다니는 게 아니라, **배터리를 스스로 관리하며 사람 손 없이 집(방)에서 상시 운용**됩니다."
- 구성: 상시 운용(Pi = 리눅스 서버, systemd) + 방 SLAM + 시맨틱 맵(물건 위치를 계속 최신으로 기억) + 자동 충전 귀환 + LLM 인터페이스.
- **자동 충전은 필수 목표** (Jasper: 손이 가면 매력 없음). 단계:
  1) 브리지가 /battery_state 발행 2) task_manager가 저전압 판단 → 귀환 3) Nav2로 독 앞까지
  4) 정밀 접근(Nav2 opennav_docking + AprilTag 등) 5) 실제 충전(접점/무선 — 방식 미정, 실측 후 결정)
  - 안전: 리튬 3S 무인 충전은 화재 위험. 핵심은 충전 제어(균형·과충전·온도), 접점 방식이 아님. 검증된 보호회로 사용.
  - 하드웨어 도착 후 확인: UGV02 충전 포트 사양, 켜진 채 충전 가능 여부, 전압 피드백 필드, 실측 배터리 소모.
- LLM 원칙: LLM은 자연어 → 정해진 형식의 명령 번역만. /cmd_vel 직접 제어 금지. task_manager가 검증(시맨틱 맵에 있나, 배터리 충분한가) 후 Nav2에 전달.
  "end-to-end는 믿을 수 없으니 task_manager가 통제한다"가 포트폴리오 핵심 설계 철학. 순서: 시맨틱 맵 → LLM.
- 연습용 simple_mover P제어/액션화는 보류 (필요해지면).

## 하드웨어 (구매 전) — 상세: `docs/hardware-baseline.md`
UGV02(내장 ESP32: 엔코더/IMU/모터 제어, Pi와 UART/JSON) · Raspberry Pi 5 8GB · RPLIDAR C1 Dev Kit(USB) · iPhone(ARKit/CoreML)

## 작업 원칙
- **학습 방식 = 미션 사다리** (docs/roadmap.md): 작은 행동 미션 → 숫자 기준 하나 → 끝나면 세 줄 기록. 결과를 먼저 보고 막히는 곳을 판다. 붙일 땐 새 미지수 하나씩.
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
- 각 계층의 합격 기준은 수치로 정하고, 점검은 스크립트로 자동화해 회귀 테스트로 재사용한다 (예: sim/slam_check.sh).
- 문제가 생기면 이미 통과한 계층부터 재점검해서 범위를 좁힌다.

