# 시뮬 플레이북 — 혼자 띄우고, 만지고, "왜 안 되지?"를 푸는 법

## 0. 외울 것: `robot` 하나

`robot` 만 치면 할 수 있는 일 목록이 나온다. 처음 한 번만: `bash ~/semantic-companion-robot/scripts/robot setup` → `source ~/.bashrc`

`robot up` 하면 tmux 창 6개가 뜬다. **층마다 창 하나**:

| 창 | 이름 | 하는 일 | 재시작 |
|---|---|---|---|
| 0 | world | Gazebo (UGV02 모드: /sim/* 로 숨김) | 거의 안 함 |
| 1 | serial | 가상 시리얼 케이블 (socat) | 안 함 |
| 2 | esp32 | 가짜 ESP32 펌웨어 | 거의 안 함 |
| 3 | bridge | 내 ugv02_bridge | 브리지 코드 고쳤을 때 (빌드 후) |
| 4 | nav2 | Nav2 + RViz (`robot edit` 의 값 적용) | `robot reload` |
| 5 | cmd | 내가 명령 치는 곳 | — |

- 한 층만 손으로 재시작: 그 창에서 `Ctrl+C` → `↑` → `Enter`
- 창 이동: `Ctrl+b` 다음 숫자 / `Ctrl+b w` (목록에서 고르기)
- 화면 나가기 (계속 실행): `Ctrl+b d` → 돌아오기: `robot show`

## 1. 움직이기

| 방법 | 어떻게 |
|---|---|
| RViz 클릭 | 위 툴바 **"Nav2 Goal"** → 지도 위를 클릭한 채 끌기 (클릭 = 위치, 끄는 방향 = 도착 방향) |
| 왕복 | `robot go` (옵션: `--laps 3`, `--a 0,0,0 --b 2.5,0,180`) |
| 숫자 판정 1회 | `robot test 2.0 0.1 90 2.0 0.5` (x y 방향 [기둥x 기둥y], 정답 위치 비교, logs/에 저장) |
| 키보드 | `robot drive` |

map 좌표: 출발점 = (0, 0). 기둥 9개 중심 x ∈ {0.9, 2.0, 3.1}, y ∈ {-0.6, 0.5, 1.6}.

## 2. Nav2 값 바꾸기 — 3가지

| 방법 | 언제 | 유지 |
|---|---|---|
| `ros2 param set /controller_server goal_checker.yaw_goal_tolerance 0.05` | 빨리 찔러보기 | Nav2 재시작하면 사라짐 |
| `ros2 run rqt_reconfigure rqt_reconfigure` (GUI) | 여러 값을 둘러보며 | 위와 같음 |
| `robot edit` → 주석(#) 풀고 저장 → `robot reload` | 값이 정해졌을 때, 기록용 | 파일에 남음 (git) |

- 바꾼 값 확인: `ros2 param get /controller_server goal_checker.yaw_goal_tolerance`
- 노드의 전체 값 목록: `ros2 param list /controller_server`
- rqt_reconfigure가 없으면: `sudo apt install ros-jazzy-rqt-reconfigure`
- 규칙: **한 번에 하나, 바꾸기 전에 예측 한 줄**

## 3. 로봇이 꼬였을 때

- 위치를 잃음 (RViz에서 레이저와 지도가 어긋남): RViz **"2D Pose Estimate"** → 로봇이 실제로 있는 곳을 클릭한 채 방향으로 끌기
- 어디 박혀서 못 나옴: Gazebo에서 로봇을 끌어다 빈 곳에 놓기 → 위의 2D Pose Estimate
- 그래도 안 되면: `robot reload` → 그래도 안 되면 `robot down` → `robot up`

## 4. "왜 안 되지?" — 증상에서 출발

| 증상 | 확인 | 흔한 원인 |
|---|---|---|
| 뭐가 문제인지 모르겠음 | **`robot status`** (층별 ✅/❌) | — |
| 값을 바꿨는데 변화가 없음 | `ros2 param get <노드> <이름>` | 이름 오타(nav2 창 맨 위 ⚠ 확인), 다른 노드가 더 낮게 자름 (아래) |
| 속도를 올렸는데 안 빨라짐 | `FollowPath.max_speed_xy`, `velocity_smoother max_velocity` | 상한이 3군데: DWB max_vel_x / max_speed_xy, smoother max_velocity, fake_esp32 바퀴 0.5 m/s |
| 로봇이 아예 안 움직임 | `ros2 topic hz /cmd_vel` → `ros2 topic info /cmd_vel -v` (누가 보내나) | bridge 창이 죽음, Nav2가 아직 덜 뜸 (nav2 창 로그) |
| Nav2가 목표를 거절 / ABORTED | nav2 창 로그에서 `planner_server`, `controller_server` 줄 | 목표가 장애물·비싼 구역 안, 경로 없음 |
| /odom이 없음 | `ros2 topic hz /odom` | bridge가 안 떴음 (UGV02 모드에선 브리지가 /odom을 만든다) |
| 누가 누구에게 보내는지 모르겠음 | `ros2 node list`, `ros2 topic list`, `rqt_graph` | — |
| 로그를 다시 보고 싶음 | `logs/nav_launch_*.txt`, `logs/shuttle_*.csv` | — |

## 5. 속도 명령이 지나가는 길 (값을 바꿀 때 어디가 막는지)

```
controller_server (DWB: max_vel_x, max_speed_xy, max_vel_theta)
  → /cmd_vel_nav → velocity_smoother (max_velocity, max_accel)
  → /cmd_vel_smoothed → collision_monitor (가까운 장애물이면 감속/정지)
  → /cmd_vel → 내 bridge → fake_esp32 (바퀴 ±0.5 m/s) → Gazebo
```
가장 낮은 상한이 이긴다. (토픽 이름은 `ros2 topic info -v`로 직접 확인해볼 것)
