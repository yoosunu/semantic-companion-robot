# Roadmap — Semantic Companion Robot

> 길을 잃으면 이 파일을 본다. 원칙: 아래 계층이 **수치로 합격**해야 위 계층을 올린다 (CLAUDE.md "통합 원칙").
> 마지막 갱신: 2026-10-13

## 미션 (2026-10-13 결정)
SLAM으로 맵을 그리고, Nav2로 이동하면서, 비전과 직접 짠 알고리즘으로 어디로 갈지 스스로 판단한다.
단순히 돌아다니는 게 아니라 **배터리를 스스로 관리하며 사람 손 없이 방에서 상시 운용**된다.
LLM은 자연어 → 정해진 명령 형식 번역만 하고, 실행 여부는 직접 작성한 `task_manager`가 검증해 결정한다.

## 단계

| 단계 | 내용 | 끝났다는 기준 | 상태 |
|---|---|---|---|
| 0. 시뮬 | TB3 Burger 시뮬에서 SLAM·Nav2 검증 | AMCL 2.0 cm, Nav2 도착 7.2 cm (`docs/validation/sim-v0.1.md`) | ✅ |
| 1. 하드웨어 오기 전 (~10/26) | `ugv02_bridge`를 가짜 UGV02(mock)로 개발, Pi SD 준비 | mock 상대로 명령·odom·타임아웃·배터리 동작 | **진행 중** |
| 2. 실물 기동 | Pi → LiDAR `/scan` → 브리지로 실제 주행 → odom 정확도 → 방 SLAM → 방 Nav2 (M2~M5) | 계층별 수치 합격 + 회귀 스크립트 | |
| 3. 자동 귀환 | `/battery_state` → `task_manager` 저전압 판단 → 독 앞 이동 | 저전압 시 사람 개입 없이 독 앞 도착 | |
| 4. 시맨틱 | iPhone YOLO 방향 + LiDAR 거리 → 물건 위치 맵. 카메라 거리 vs LiDAR 비교 (시그니처) | 물건 위치 오차 수치 | |
| 5. 자동 충전 | 정밀 도킹(opennav_docking + AprilTag 등) → 실제 충전 | 스스로 충전 후 재출발 | |
| 6. LLM | 자연어 → `task_manager` 검증 → 실행 | "책상 가봐" 동작, 잘못된 명령 거부 | |
| 7. 상시 운용 | systemd 자동 실행/재시작, 장시간 무인 테스트, 문서 정리 | 며칠 연속 무인 운용 기록 | |

## 1단계 세부 (현재)

| # | 할 일 | 담당 | 상태 |
|---|---|---|---|
| 1-1 | UGV02 프로토콜 정리 (`docs/ugv02-protocol.md`) | Claude | |
| 1-2 | 브리지 v0: `/cmd_vel` → T:13 JSON 문자열 로그 출력 (시리얼 없음) | **Jasper** (simple_mover 복사 후 수정) | |
| 1-3 | mock UGV02: 가상 시리얼로 JSON 받고 가짜 바퀴/IMU/전압 응답 | Claude | |
| 1-4 | 브리지 v1: 실제 시리얼(가상 포트)로 mock과 통신, `/odom` 발행 | 함께 | |
| 1-5 | 브리지 v2: `/cmd_vel` 0.5 s 타임아웃 정지, 종료 시 정지, `/battery_state` | **Jasper** | |
| 1-6 | Pi SD 굽기 + `scripts/setup_pi.sh` | 함께 | |

## 시뮬(TB3)과의 관계
- 브리지 개발에는 TB3/Gazebo가 필요 없다. 브리지의 상대는 UGV02(실물) 또는 mock이다.
- Gazebo는 이미 ROS로 말하므로(ros_gz_bridge가 브리지 역할) 시뮬에는 우리 브리지가 끼지 않는다.
- 시뮬은 Nav2/task_manager 로직의 회귀 테스트·실험용으로 유지.

## 노드별 소유 (점점 Jasper 비중 증가)
| 노드 | Claude | Jasper |
|---|---|---|
| simple_mover (연습, 보류) | 작성 | 리팩터링 |
| ugv02_bridge | 뼈대·시리얼·mock | 변환·안전 로직 |
| task_manager | 뼈대 | 판단 로직 대부분 |
| semantic_mapper | 리뷰·테스트 설계 | 거의 전부 |
