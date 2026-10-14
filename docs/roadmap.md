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

## 1단계 세부 (현재) — "TB3를 UGV02라고 가정하고 내 브리지로 움직이기"

```
teleop / Nav2 ── /cmd_vel ──→ ugv02_bridge (Jasper) ──JSON over 가상 시리얼──→ fake_esp32 ──→ Gazebo TB3 가 움직임
                 /odom   ←──      (번역기)          ←──JSON 피드백(바퀴·IMU·전압)──     ←── Gazebo
```
- fake_esp32: UGV02 펌웨어인 척하는 프로그램. JSON을 받아 Gazebo 로봇을 움직이고, Gazebo 상태를 UGV02 형식 JSON으로 돌려준다.
- 브리지 입장에서는 실제 UGV02와 구분되지 않는다 → 10/26엔 포트 경로만 바꾼다.
- 차이(정직하게): TB3 최고속도 0.22 m/s vs UGV02 0.5 m/s, 물리 특성 다름. 프로토콜·안전 로직 검증용이지 튜닝용 아님.

| # | 할 일 | 담당 | 상태 |
|---|---|---|---|
| 1-1 | UGV02 프로토콜 정리 (`docs/ugv02-protocol.md`) | Claude | ✅ (피드백 형식은 [가정]) |
| 1-2 | fake_esp32 + 가상 시리얼 + 시뮬 "UGV02 모드" 실행 스크립트 | Claude | ✅ (2026-10-14) |
| 1-3 | 브리지 v0: `/cmd_vel` → T:13 JSON → 시리얼. **합격: teleop으로 TB3가 내 브리지를 거쳐 움직인다** | **Jasper** (시리얼 헬퍼는 Claude 제공, `src/scr_ugv02`) | ✅ (2026-10-14) |
| 1-4 | 브리지 v1: 피드백 JSON → `/odom` (+TF). 합격: RViz에서 내 odom으로 로봇이 따라 움직임 | 함께 | |
| 1-5 | 브리지 v2: `/cmd_vel` 0.5 s 타임아웃 정지, 종료 시 정지, `/battery_state`. 합격: teleop 끊으면 0.5 s 안에 정지 | **Jasper** | |
| 1-6 | 내 브리지를 거쳐 시뮬 SLAM·Nav2 재실행 (sim v0.1 회귀) | 함께 | |
| 1-7 | Pi SD 굽기 + `scripts/setup_pi.sh` | 함께 | |

## 시뮬(TB3)과의 관계
- 평소 시뮬: Gazebo가 ROS로 직접 말함(ros_gz_bridge). 우리 브리지 없음.
- UGV02 모드 시뮬: Gazebo 토픽 이름을 바꿔 숨기고, fake_esp32가 그 사이를 맡는다 → 전체 스택이 내 브리지를 통과.

## 노드별 소유 (점점 Jasper 비중 증가)
| 노드 | Claude | Jasper |
|---|---|---|
| simple_mover (연습, 보류) | 작성 | 리팩터링 |
| ugv02_bridge | 뼈대·시리얼·mock | 변환·안전 로직 |
| task_manager | 뼈대 | 판단 로직 대부분 |
| semantic_mapper | 리뷰·테스트 설계 | 거의 전부 |
