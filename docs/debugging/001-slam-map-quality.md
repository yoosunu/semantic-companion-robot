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
- [ ] robot_start.sh / robot_slam.sh 내용 확인 (어떤 월드, 어떤 launch 인자)
- [ ] Gazebo RTF 확인
- [ ] `ros2 param get /cartographer_node use_sim_time`, `/cartographer_occupancy_grid_node` 동일 확인

## 로그

## 원인

## 수정

## Regression test / 결과
