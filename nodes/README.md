# nodes — 로봇에서 실제로 도는 코드

이 폴더만 보면 NERI 가 무엇을 하는지 알 수 있음. 나머지 폴더는 켜기(run), 설치(setup), 실험·측정(tools), 시뮬 기록(sim).
지금은 `python3 경로` 로 실행 (ROS 정식 패키지로 바꾸는 건 다음 단계).

| 폴더 | 파일 | 줄 | 하는 일 | 켜는 곳 |
|---|---|---|---|---|
| perception | `face_perception.py` | 139 | 카메라 → YuNet 얼굴 검출 → 추적 → `/people` | `run/look_test.sh` |
| | `face_track.py` | 92 | 추적기 (순수 계산, 방향 공간에서 잇기) | face_perception 이 import |
| | `lidar_motion.py` | 70 | LiDAR "비어 있던 곳에 새로 들어옴" + 들어온 방향 (잠 깨우기, 순수 계산) | executive 가 import |
| | `imu_touch.py` | 60 | IMU 톡톡 감지 + 맞은 방향, 자기 흔들림 빼기 (순수 계산) | executive 가 import |
| navigation | `scr_explore.py` | 966 | 경계 탐색 + 바퀴 주인(nav2/crawl/whisker) 상태 머신 | `run/run_explore.sh` |
| | `whisker.py` | 1125 | 충돌·끼임 감지, 흉터, 탈출 사다리 | `nav.launch.py`, run_explore |
| behavior | `executive.py` | 100 | **사령관**: 모드(SLEEP·IDLE·DOZE·COMPANION·TASK) → `/neri/profile` 설정표. 잠 깊이 vs 자극 세기(LiDAR·톡톡) | `run/look_test.sh` |
| | `executive_brain.py` | 130 | 사령관의 판단 (순수 계산): 전환 규칙·설정표·예약 미루기, 시간은 dur() 한 곳 | executive 가 import |
| | `look_at_person.py` | 123 | 고개 노드: `/people`·`/scan`·`/odom` → brain → `/pantilt/cmd` | `run/look_test.sh` |
| | `look_brain.py` | 300 | 고개의 마음 (순수 계산): 쳐다볼 이유·딴짓·꾸벅 모양(A·B, 3번째 잠)·깨는 모양·클립(까꿍·시선 따라가기·움찔·따라 끄덕) | look_at_person 이 import |
| | `status_display.py` | 100 | OLED 4줄 상태 (탐색기 없으면 1번 줄 = 사령관 모드) | `nav.launch.py`, `run/look_test.sh` |
| system | `heartbeat.py` | 178 | 1분마다 로봇 상태 한 줄 (로그 2층) | `run/run_explore.sh` |
| | `run_report.py` | 476 | ROS 로그 → 주행 시간표·보고서 | `nav.launch.py` 끝날 때, `run/stop_all.sh` |
| | `merge_params.py` | 53 | Nav2 원본 설정 + 내 설정 합치기 | `nav.launch.py` |

C++ 은 `src/scr_ugv02/src/ugv02_bridge.cpp` (하드웨어 브리지, 737줄) 하나. 메시지 정의는 `src/scr_interfaces/`.
순수 계산 파일(face_track, look_brain, scr_explore·whisker 의 판단부)은 `tests/*_sim.py` 로 ROS 없이 시험.

## 누가 누구에게 (11-14 사령관 v1)
```
face_perception ──/people──▶ look ──/pantilt/cmd──▶ 브리지
        ▲                    │ ▲
        │         /neri/event │ │ /neri/profile (head)
        │                    ▼ │
        └─ /neri/profile ◀── executive ◀── /scan (잠 깨우기), /neri/request (sleep·wake·task:·done:)
              (face_hz)          └──▶ status_display (OLED)
```
