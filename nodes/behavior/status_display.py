#!/usr/bin/env python3
"""status_display — 로봇 상태를 한 곳에 모아 UGV02 OLED(4줄) 글자로 (11-10 Jasper: "상태 머신을 디스플레이에, 나중에 대화에도").

  입력  /robot/state   (탐색기: 목적·바퀴 주인·하는 일·남은 경계·포기 수)
        /whisker/state (위스커: drive / stop / back / trapped / help / pause, 탈출 사다리 칸)
        /battery_state (브리지: 전압)
        /neri/profile  (사령관: 지금 모드 — 탐색기가 안 돌 때 1번 줄에, 11-14)
  출력  /robot/display (String, 4줄 "\\n" 로 이음) → 브리지가 바뀐 줄만 {"T":3,"lineNum":i,"Text":...} 로 OLED 에 씀
        (시리얼에 쓰는 건 브리지 하나뿐 — 서로 부딪히지 않게)

  화면 예:  EXPLORE  O5 X3      목적 · 남은 경계 · 포기
            NAV2   VIEW         바퀴 주인 · 하는 일
            ESCAPE 2/4          위스커
            11.8V  04:12        전압 · 탐색 시간
  OLED 는 영어·숫자만 (한글 미지원 가능성), 한 줄 20자. 바뀔 때만, 최대 0.5 s 에 한 번.
"""
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

W = 20
STALE_S = 5.0


def clean(t):
    t = "".join(c for c in t if 32 <= ord(c) < 127 and c not in '"\\\\')
    return t[:W]


def lines(robot, whisker, volt, now, neri=None):
    if robot and now - robot["at"] < STALE_S:
        r = robot["d"]
        mode = {"done": "DONE", "home": "HOME"}.get(r.get("phase"), (r.get("mode") or "idle").upper())
        l0 = f"{mode:<8} O{r.get('open', 0)} X{r.get('gave_up', 0)}"
        l1 = f"{(r.get('owner') or 'none').upper():<6} {r.get('doing', '')}"
        t = int(r.get("t", 0))
        clock = f"{t // 60:02d}:{t % 60:02d}"
    else:
        l0, l1, clock = (neri or {}).get("mode", "IDLE"), (neri or {}).get("why", ""), ""
    st = (whisker or {}).get("state", "drive")
    k, n = (whisker or {}).get("try", 0), (whisker or {}).get("rungs", 4)
    l2 = {"drive": "TOUCH OK", "pause": "PUSHED", "stop": "BUMP", "back": f"ESCAPE {k}/{n}" if k else "BACK OFF",
          "trapped": f"ESCAPE {k}/{n} ..", "help": "HELP! STUCK"}.get(st, st.upper())
    l3 = (f"{volt:.1f}V" if volt else "--.-V") + (f"  {clock}" if clock else "")
    return [clean(x) for x in (l0, l1, l2, l3)]


class StatusDisplay(Node):
    def __init__(self):
        super().__init__("status_display")
        self.robot, self.whisker, self.volt, self.neri = None, None, None, None
        self.last, self.last_t = None, 0.0
        self.create_subscription(String, "robot/state", self.on_robot, 10)
        self.create_subscription(String, "whisker/state", self.on_whisker, 10)
        self.create_subscription(BatteryState, "battery_state", lambda m: setattr(self, "volt", m.voltage), 10)
        self.create_subscription(String, "neri/profile", self.on_neri, 10)
        self.pub = self.create_publisher(String, "robot/display", 10)
        self.create_timer(0.5, self.tick)

    def on_robot(self, m):
        try:
            self.robot = {"d": json.loads(m.data), "at": time.time()}
        except ValueError:
            pass

    def on_neri(self, m):
        try:
            self.neri = json.loads(m.data)
        except ValueError:
            pass

    def on_whisker(self, m):
        try:
            self.whisker = json.loads(m.data)
        except ValueError:
            pass

    def tick(self):
        now = time.time()
        ls = lines(self.robot, self.whisker, self.volt, now, self.neri)
        if ls != self.last or now - self.last_t > 2.0:      # 바뀌었을 때 + 2 s 마다 (브리지가 5 s 마다 4줄 다시 씀)
            self.last, self.last_t = ls, now
            self.pub.publish(String(data="\n".join(ls)))


def main():
    rclpy.init()
    n = StatusDisplay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
