# 001 — SLAM 맵 품질 검증

- 상태: **진행 중**
- 시작: 2026-10-07

## 증상
로봇을 거의 직진시키는데도 Cartographer 맵이 Gazebo 월드 구조와 직관적으로 맞지 않아 보임.
SLAM pipeline은 동작(operational)하지만 map quality는 미검증.

## 정상 기준 (turtlebot3_world)
- 육각형 외벽 + 3×3 원형 기둥이 그대로 나타나야 함
- 시뮬레이션 LiDAR 최대 거리 약 3.5m → 먼 벽은 가까이 가야 그려짐 (정상)
- `map` 원점 = 로봇 시작 위치, RViz/Gazebo 시점은 독립 → 모양·비율로 비교
- 비정상 신호: 벽 겹침(ghosting), 회전 시 부채꼴 번짐, 직선 벽 휨, 기둥 중복, RViz 로봇 위치가 실제와 점점 벌어짐

## 가설
- H1: 비교 기준 문제 — 띄운 월드가 다르거나(empty_world), 시점·원점 차이를 오해
- H2: `use_sim_time` 불일치로 TF/scan 타임스탬프 어긋남
- H3: teleop 입력 특성 — `LINEAR_SPEED = 0.30`이 Burger 최대(약 0.22 m/s) 초과, 0.4s 펄스로 급가감속 (기록만, 값 미변경)

## 실험 / 확인
- [x] robot_start.sh → `turtlebot3_world` 사용 확인. robot_slam.sh → `use_sim_time:=True` 전달 → **H1 기각**
- [x] Gazebo RTF ≈ 95% (0.95x) — sim time 사용 시 문제 없음
- [x] `use_sim_time`: cartographer_node / occupancy_grid_node / robot_state_publisher / rviz2 모두 True → **H2 기각**
  (ros_gz_bridge는 False — 메시지 stamp는 Gazebo에서 오므로 현재 영향 없음, 기록만)
- [x] 정지 상태 점검 (`scripts/slam_check.sh`, logs/slam_check_20261007_174735.txt)
- [x] Gazebo 원본 토픽(`gz topic -e -t /scan`)에서도 전부 0.12 → **ros_gz_bridge 무관, Gazebo 센서 단계 문제**
- [x] 렌더러 조건 비교 (`scripts/experiments/lidar_render_test.sh`, 공식 예제 `gpu_lidar_sensor.sdf`, 서버 단독)
- [x] TurtleBot3 burger 모델의 LiDAR 정의(model.sdf) 확인 — 센서 원점 = 하우징 메시 원점 (-0.032 0 0.171)

## 로그
정지 상태, 클린 재시작 후:
```text
/scan: range_min=0.12 range_max=3.5, beams=360, 전부 0.120 (inf 0, nan 0)
/map: resolution 0.05, 18x18 (= 0.9m x 0.9m)
map->odom: yaw -87.4° (이전 실행 -68.2°)  ← 정지 상태인데 임의 회전
odom->base_footprint: 0 (정지, 정상)
```

## 새 가설
- **H4: LiDAR 센서 데이터 자체가 비정상.** 모든 빔이 range_min(0.12)으로 고정 → 원통형 스캔.
  - 맵이 0.9m로 작은 것, 정지 상태에서 map->odom 회전이 임의로 정해지는 것(원형 스캔은 회전 불변)을 모두 설명.
  - 유력 원인: Ogre2 segfault 회피용 `--render-engine ogre`로 바꾼 뒤 gpu_lidar 깊이값이 깨짐.

### 렌더러 실험 결과 (logs/lidar_render_test_20261007_175545.txt)
VM OpenGL: `virgl (ANGLE, Apple M1, Metal)`, **OpenGL version 2.1** (Mesa 25.2.8). gz sim 8.15.0.

| 조건 | 결과 |
|---|---|
| 1. ogre | **정상.** 10240 beams, finite 7440 / inf 2800, 0.08~10m, 서로 다른 값 6432개 (종료 시 segfault는 kill 시점, 결과와 무관) |
| 2. ogre2 | 서버 조기 종료 (exit 1) |
| 3. ogre2 + LIBGL_ALWAYS_SOFTWARE | ogre2 GL3Plus 초기화 중 segfault (Mesa driCreateNewScreen3) |
| 4. ogre2 + headless | ogre2 리소스 초기화 중 segfault |

해석:
- **"Ogre1이 gpu_lidar를 깨뜨린다"(H4의 유력 원인) 기각.** Ogre1에서도 일반 gpu_lidar는 정상 거리값을 낸다.
- ogre2는 이 VM에서 어떤 방식으로도 못 씀 (GL 2.1 환경) → Ogre1 유지가 유일한 선택지.
- 따라서 문제는 렌더러 일반이 아니라 **burger 모델의 LiDAR 구성과 Ogre1의 조합**.

## 새 가설
- **H5: LiDAR가 로봇 자기 몸체(LDS 하우징 visual 등)를 보고 있다.**
  전 빔이 정확히 range_min(0.12)으로 고정 = 모든 방향에서 최소 거리보다 가까운 물체에 막힘.
  Ogre1/Ogre2의 자기 가림(self-occlusion) 처리 차이로 Ogre1에서만 드러날 가능성.

### 자기 가림 실험 결과 (logs/lidar_selfocclusion_test_20261007_181015.txt)
- 원본 /scan, 하우징 visual 제거(/scan_novis), 센서 +6cm(/scan_up) **모두 360빔 전부 0.12, 서로 다른 값 1개**
- → **H5 기각.** 하우징은 원인 아님.
- 단서: burger LiDAR는 gaussian noise(stddev 0.01)가 켜져 있는데 값이 정확히 0.12 하나뿐
  → 원시 거리값이 range_min 미만(0 추정)으로 계산된 뒤 min으로 clamp된 것. **거리 계산 자체가 실패.**

## 새 가설
- **H6: 360° 수평 시야각이 이 환경(Ogre1 + OpenGL 2.1 virgl)의 gpu_lidar에서 실패한다.**
  예제(정상)는 약 160°, burger(실패)는 360°. 넓은 시야각은 여러 카메라 렌더링을 이어 붙이는 경로를 탐.
  검증: `scripts/experiments/lidar_param_test.sh` (fov180 / fov90 / nonoise / min008 각각 한 요인만 변경)

### 파라미터 실험 결과 (logs/lidar_param_test_20261007_181214.txt)
| 변형 | 결과 |
|---|---|
| fov180 / fov90 | 전부 0.12 |
| nonoise | 전부 0.12 |
| min008 | 전부 **0.08** (출력이 range_min을 그대로 따라감) |
- → **H6 기각.** 시야각·노이즈·최소거리 설정은 원인 아님.
- 원시 깊이값이 ~0으로 읽히고 range_min으로 clamp되는 것이 재확인됨.

## 새 가설
- **H7: TurtleBot3 월드/launch 실행 방식이 원인** (예제는 서버 단독 실행에서 정상)
- **H8: 수직 1줄(렌더 높이 1px)이 Ogre1 + GL 2.1에서 깊이 읽기 실패** (예제 LiDAR는 수직 16줄)
  검증: `scripts/experiments/lidar_world_test.sh` — burger LiDAR(stock / vert16)를 예제 월드에 spawn

### 월드 교차 실험 결과 (logs/lidar_world_test_20261007_181432.txt)
정상 동작이 확인된 예제 월드(ogre, 서버 단독)에 burger LiDAR를 넣음:
| 대상 | 결과 |
|---|---|
| /lidar (예제 자체, 대조군) | 정상, 0.08~10m, 서로 다른 값 6482개 |
| /scan_stock (burger 그대로, 수직 1줄) | **전부 0.12** |
| /scan_vert16 (burger + 수직 16줄) | **정상**, 0.12~3.34m, 서로 다른 값 4411개 |
- 같은 월드·같은 렌더러에서 수직 줄 수만 다르고 결과가 갈림.
- → **H7 기각** (월드/launch 무관), **H8 채택.**

### 다중 행 확인 실험 (logs/lidar_rows_probe_20261007_182159.txt, logs/lidar_rows_probe2_20261007_182442.txt)
- 수직 2줄: 절반가량 inf, 서로 다른 값 30개 → 불안정. 수직 3줄: 전부 유효.
- **gz ranges 배열은 행 우선(row-major)**: index = row × 360 + col
- v3wide(행 각도 -0.6 / -0.3 / 0.0 rad, 노이즈 제거) 각 행 평균:
  - row0(-0.6): 0.527m — 센서 높이 약 0.33m 기준 바닥까지 0.33/sin(0.6)=0.59m와 부합
  - row1(-0.3): 1.141m — 0.33/sin(0.3)=1.12m와 부합
  - row2(0.0): **전부 0.12** ← 수평인데도 실패
- v3(±0.5°)에서도 맨 위 행(row2, +0.5°)만 실패, row0·row1은 정상.
- **ros_gz_bridge는 가운데 행(row1)을 ROS LaserScan(360빔)으로 넘긴다** (v3wide: row1과 mean|diff| 0.0002)
- 정확도 (v3를 (-2.0, 0.5)에 배치, 가운데 행 = 0°):
  - 예상: 가장 가까운 기둥 두 개가 약 0.9m, -29°(331°) / +34° 방향
  - 측정: **0.908m @ 331°, 0.958m @ 31°** ✓
  - 그 외: 0.469m @ 89° = 0.5m 왼쪽의 다른 테스트 로봇 ✓, 0.969m @ 267° = 1.0m 아래의 원래 로봇 ✓
  - 359° 한 빔만 0.12 (경계 아티팩트로 보임, 회귀 테스트에서 재확인)

## 원인
**이 VM 환경(Ogre1 + OpenGL 2.1 virgl)의 gpu_lidar는 렌더 결과의 맨 위 행 깊이값을 읽지 못한다(~0 → range_min으로 clamp).**
수직 각도와 무관하게 '맨 위 행'이 실패하며, 수직 1줄짜리 2D LiDAR(burger)는 그 한 줄이 곧 맨 위 행이라 전체가 실패한다.
→ 360빔 전부 0.12 → SLAM이 반지름 12cm 원통만 보게 되어 맵이 0.9m로 작고, 원형 스캔이라 회전이 임의로 정해짐.
SLAM/Cartographer 설정 문제가 아니었음.

## 수정
- 원본 `turtlebot3_ws`는 수정하지 않고, repo의 `scr_gazebo` 패키지에 burger 모델 사본을 둔다.
- 사본의 LiDAR만 **수직 3줄(±0.5°)** 로 변경. ros_gz_bridge가 가운데 행(0°)을 넘기므로 ROS `/scan`은 그대로 360빔 2D 스캔.
  → 별도 추출 노드 불필요 (처음 계획했던 C++ 추출 노드는 실험 결과에 따라 만들지 않음).
- 자체 launch(`scr_gazebo/launch/sim_world.launch.py`)에서 Ogre1 렌더러 지정 → `turtlebot3_ws` launch 파일 수정 원복 가능.
- 실제 로봇(RPLIDAR C1)은 렌더링을 거치지 않으므로 이 문제와 무관. 이 수정은 시뮬레이션 전용.

## Regression test / 결과

### 1) 정지 상태 (logs/slam_check_20261007_183159.txt, `ros2 launch scr_gazebo sim_world.launch.py` + Cartographer)
| 항목 | 수정 전 | 수정 후 |
|---|---|---|
| /scan 유효 빔 | 360개 전부 0.12 | 327개 유효(0.12~3.41m, 평균 1.37m), 33개 inf(3.5m 밖) |
| /map 크기 | 18×18 (0.9m×0.9m) | 91×113 (4.55m×5.65m) |
| 정지 상태 map→odom yaw | -68.2° / -87.4° (임의) | **0.28°** |
| RViz | 로봇 주변 작은 회색 사각형 | 육각형 외벽 일부 + 기둥 3열이 월드 배치대로 표시 |

### 2) 주행 후 전체 맵
- [ ] 월드 한 바퀴 주행 후 맵이 육각형 + 3×3 기둥과 일치하는지
