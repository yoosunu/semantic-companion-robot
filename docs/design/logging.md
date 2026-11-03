# 로그 설계 (초안 v0, 2026-11-03)

> 목적: 로봇이 몇 시간~하루 혼자 돌아도, **문제가 났다는 걸 바로 알고, 나중에 원인을 찾을 수 있게.**
> 참고한 것: Sentry 의 세 가지 — 이벤트(무슨 일), breadcrumbs(직전 맥락), 알림(심각한 것만).

## 원칙

1. **바뀔 때만 적는다.** 같은 상태가 이어지는 동안은 한 줄도 안 쓴다. (예: "정지" 1줄, "해제" 1줄 — 그 사이 2분은 0줄)
2. **사람은 날로그를 읽지 않는다.** 스크립트가 시간표로 바꾸고, 사람·agent 는 시간표에서 시작해 의심 가는 줄만 연다.
3. **알림은 "알게만" 한다.** 분석은 나중에, Pi 에 남은 로그로.
4. **비밀은 `.env`** (알림 주제 이름, healthchecks URL). 공개 저장소에 절대 커밋 안 함. 메시지 본문에 IP·비밀번호 금지.

## 세 층

| 층 | 무엇 | 언제 쓰나 | 크기 |
|---|---|---|---|
| 1. 사건 (events) | 상태가 바뀔 때 한 줄 | 항상 | 작음 |
| 2. 심박 (heartbeat) | 1분에 한 줄 요약 | 항상 | 하루 1,440줄 |
| 3. 블랙박스 | 최근 2~3분 센서 기록을 덮어쓰며 유지, **ERROR 때만 파일로 저장** | 사고 때만 | 평소 0 |

### 1층 — 사건 한 줄의 모양 (JSON Lines)

```json
{"t": 1793458778.83, "lvl": "WARN", "src": "collision", "ev": "stop", "x": 4.62, "y": 0.29, "v": 10.9, "msg": "PolygonStop"}
```

| 필드 | 뜻 |
|---|---|
| `t` | 시각 (epoch 초) |
| `lvl` | INFO / WARN / ERROR |
| `src` | 출처: run, nav, collision, recovery, explore, bridge, power, imu |
| `ev` | 사건 이름 (아래 목록) |
| `x, y` | map 좌표 (모르면 생략) |
| `v` | 배터리 전압 |
| `msg` | 사람이 읽을 짧은 설명, 근거 로그 한 조각 |

### 1층 — 사건 목록

| src | ev | lvl | 비고 |
|---|---|---|---|
| run | start / end | INFO | 무슨 모드(explore, nav, …), 시작 전압 |
| nav | goal_sent / goal_ok | INFO | 목표 좌표 |
| nav | goal_failed | WARN | 실패 이유 (plan 실패 / 진척 없음 / 취소) |
| nav | no_progress | WARN | controller "Failed to make progress" |
| nav | plan_failed | WARN | 같은 목표 연속 실패는 **1줄 + 횟수** 로 묶음 |
| collision | stop / slow / clear | WARN / INFO / INFO | stop 이 **30초 넘게** 이어지면 ERROR 로 승격 |
| recovery | start / ok / failed | INFO / INFO / WARN | spin, backup, wait |
| explore | done | INFO | "All frontiers traversed" |
| explore | goal_blacklisted | INFO | |
| imu | tilt / bump | WARN | 턱·기울기 감지 (구현 예정) |
| bridge | start / stop | INFO | 정상 시작·종료 |
| bridge | died | ERROR | odom 이 2초 넘게 끊김 |
| power | low / critical | WARN / ERROR | **30초 평균** 10.5 V / 10.2 V (셀당 3.5 / 3.4). low 해제는 10.7 V 위로 올라와야 (되돌림 여유). 임시값 — 셀 상태 확인 후 재조정 |

### 2층 — 심박 (1분)

```json
{"t": 1793458800, "x": 4.62, "y": 0.29, "v": 10.9, "mode": "explore", "moving": false}
```
→ healthchecks.io ping 도 여기서 같이 (아래).

### 3층 — 블랙박스

- 대상: `/scan`, `/odom`, `/cmd_vel`, `/tf`, IMU (브리지가 내보내게 되면)
- 방식: rosbag2 녹화를 최근 2~3분만 유지 → ERROR 사건이 나면 그 구간을 `logs/blackbox/<시각>/` 에 저장
- 블랙박스 파일은 커밋하지 않음 (`.gitignore` 의 `*.mcap`, `rosbag*/`)

## 알림

| 경로 | 무엇 | 언제 켜나 |
|---|---|---|
| **ntfy** (폰 → 애플워치) | ERROR 사건: bridge died, collision stop 30초+, power critical. 조용한 알림: explore done, run end | 지금부터 |
| **healthchecks.io** (소식 끊김 알림) | 심박이 N분 끊기면 대신 알림 | **자율 주행 단계부터.** 지금은 꺼 둠 — 수동 실행이 많아서 끊김 경고가 쏟아지면 안 됨. 켤 때도 "주행 중"(run start ~ end)에만 ping |

## 저장과 보관

- Pi: `~/scr_logs/<날짜>/events.jsonl`, `heartbeat.jsonl`, `blackbox/`
- 주행 후 Mac 으로 가져와 `run_report.py` 실행 → `logs/reports/<날짜>_<run>.md` (시간표)
- 보관: **7일**. 더 오래된 날짜 폴더는 자동 삭제 (블랙박스 포함)

## run_report.py 가 만드는 것

1. 요약: 시간, 거리, 시작·끝 전압, 사건 수 (수준별)
2. 시간표: 구간별 "어디서 무엇을" (오늘 거실 표 같은 것)
3. 의심 구간: 같은 자리 60초+ / ERROR 앞뒤 — 근거 로그 줄 번호와 함께
4. 지금은 Nav2 원본 로그(`~/.ros/log`)에서도 뽑을 수 있게 (1층이 생기기 전 과도기)

## 구현 순서 (제안)

1. `run_report.py` — 지금 있는 `~/.ros/log` 에서 시간표 만들기 (거실 로그로 검증)
2. 1층 기록 노드 `event_logger` — Nav2 토픽·로그를 구독해 사건으로 변환
3. ntfy 알림 (ERROR 만)
4. 2층 심박
5. 3층 블랙박스
6. healthchecks.io (자율 주행 단계에서)

## 정해야 할 것

- [x] 전압 low / critical 기준 (임시)
- [x] 로그 보관 기간 → 7일
- [ ] 1층 목록에서 빼거나 더할 것 — run_report 로 실제 시간표를 본 뒤에

## 덤: UGV02 화면 (OLED)

`{"T":3,"lineNum":0..3,"Text":"..."}` 로 4줄 표시 가능 → 모드·전압·마지막 사건 표시 후보. 비밀값은 표시 금지.

## 비밀값 위치

Pi `~/.scr.env` (`NTFY_TOPIC=...`, chmod 600) — 저장소 밖.
