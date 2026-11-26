#!/usr/bin/env python3
"""world_model — 네리가 '알고 있는' 객체의 주인 (11-27, Jasper 와). 판단은 world_objects.py (순수 계산, tests/world_objects_sim.py).

  왜 (Jasper 11-27): 의자를 그려 놓고 끝이면 '본 척'. 한 번 비전 + LiDAR 로 알았으면 LiDAR 가 매 스캔 그 지지점을 붙잡고 있어야
    움직이면 움직였다, 아니면 아니다를 앎. floor_hazard 에 다 맡기면 너무 커짐 → 데이터 주인을 나눔:
      floor_hazard = 비전으로 모양 추정 (다리 잇기·원) → 확인된 지지점을 여기에 맡김
      world_model  = 객체 기억 + 매 스캔 추적 → 사건
  첫 버전 (11-27 합의): 추적 + 사건 로그만. costmap 은 아직 floor_hazard 가 그림 (moved 를 받으면 그 가구 모양을 옮김). 쳐다보기 없음.

  입력  /scan                  매 장 → odom 좌표로 → WorldObjects.step (odom = 연속. map 은 SLAM 보정으로 튐)
        /semantic/adopt        String JSON {"key", "name", "anchors": [[x, y], ...] (map), "corners": [...] (map)} — 맡기기
                               {"key", "drop": true} — 지우기 (floor_hazard 가 가구를 뺌)
        /semantic/clear        전부 지움
  출력  /semantic/objects      String JSON, 1 Hz: [{"id", "key", "name", "state", "x", "y" (map), "th", "anchors" (map)}]
        /semantic/object_event String JSON: {"type": moving|moved|hidden|found|missing, "id", "key", "name", ...(map 좌표)}
  로그  OBJ #1 chair 맡음 / moved 0.55 m (x,y → x,y) 회전 … / missing … / 1 분마다 상태 요약
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_objects as wo  # noqa: E402


def tf2d(t):
    q = t.transform.rotation
    return t.transform.translation.x, t.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def apply(T, pts):
    x, y, th = T
    p = np.asarray(pts, float).reshape(-1, 2)
    c, s = math.cos(th), math.sin(th)
    return np.column_stack([x + p[:, 0] * c - p[:, 1] * s, y + p[:, 0] * s + p[:, 1] * c])


def inverse(T):
    x, y, th = T
    c, s = math.cos(th), math.sin(th)
    return -(x * c + y * s), -(-x * s + y * c), -th


class WorldModel(Node):
    def __init__(self):
        super().__init__("world_model")
        p = self.declare_parameter
        self.odom, self.map = p("odom_frame", "odom").value, p("map_frame", "map").value
        self.W = wo.WorldObjects(gate=p("gate", 0.10).value, hold_s=p("hold_s", 1.0).value, move_m=p("move_m", 0.10).value)
        self.key2id = {}
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "/semantic/adopt", self.on_adopt, 10)
        self.create_subscription(String, "/semantic/clear", self.on_clear, 10)
        self.pub_obj = self.create_publisher(String, "/semantic/objects", 1)
        self.pub_ev = self.create_publisher(String, "/semantic/object_event", 10)
        self.create_timer(1.0, self.publish_objects)
        self.n_scan, self.ms, self.sum_t = 0, 0.0, time.time()
        self.get_logger().info(f"OBJ 준비: 추적 좌표 {self.odom}, 출력 {self.map}, 문턱 {self.W.gate * 100:.0f} cm, 놓침 참기 {self.W.hold_s} s, "
                               f"움직임 {self.W.move_m * 100:.0f} cm")

    def lookup(self, target, src, stamp=None):
        try:
            if stamp is not None:
                return tf2d(self.tf.lookup_transform(target, src, stamp, timeout=Duration(seconds=0.05)))
            return tf2d(self.tf.lookup_transform(target, src, Time()))
        except Exception:                                               # noqa: BLE001
            if stamp is not None:
                return self.lookup(target, src)
            return None

    def to_map(self, pts):
        T = self.lookup(self.map, self.odom)
        return None if T is None else apply(T, pts)

    # ---------- 맡기기 ----------
    def on_adopt(self, m):
        j = json.loads(m.data)
        key = str(j.get("key"))
        if j.get("drop"):
            oid = self.key2id.pop(key, None)
            if oid in self.W.objs:
                self.get_logger().info(f"OBJ #{oid} {self.W.objs[oid].name} 뺌 (floor_hazard 가 가구를 지움)")
                self.W.drop(oid)
            return
        T = self.lookup(self.odom, self.map)                            # map → odom
        if T is None or not j.get("anchors"):
            return
        anchors = apply(T, j["anchors"])
        corners = apply(T, j["corners"]) if j.get("corners") else None
        o, new = self.W.adopt(j["name"], anchors, corners, time.time())
        old = self.key2id.get(key)
        self.key2id[key] = o.id
        if new:
            self.get_logger().info(f"OBJ #{o.id} {o.name} 맡음: 닻 {len(o.local)} 개 "
                                   f"{np.round(np.asarray(j['anchors']), 2).tolist()} (map)" + (f" — key {key} 가 #{old} 였음" if old else ""))

    def on_clear(self, _):
        n = len(self.W.objs)
        self.W.objs.clear()
        self.key2id.clear()
        self.get_logger().info(f"OBJ 전부 지움: {n} 개")

    # ---------- 매 스캔 ----------
    def on_scan(self, m):
        if not self.W.objs:
            return
        T = self.lookup(self.odom, m.header.frame_id, Time.from_msg(m.header.stamp))
        if T is None:
            return
        t0 = time.time()
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        pts = apply(T, np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]))
        events = self.W.step(pts, np.array(T[:2]), time.time())
        self.ms += (time.time() - t0) * 1000
        self.n_scan += 1
        for kind, o, info in events:
            self.event(kind, o, info)

    def event(self, kind, o, info):
        key = next((k for k, v in self.key2id.items() if v == o.id), None)
        ev = {"type": kind, "id": o.id, "key": key, "name": o.name}
        T = self.lookup(self.map, self.odom)
        for k in ("from", "to"):
            if k in info and T is not None:
                ev[k] = apply(T, [info[k]])[0].round(3).tolist()
        if "to" in info and "from" in info:
            ev["delta"] = (np.asarray(ev.get("to", info["to"])) - np.asarray(ev.get("from", info["from"]))).round(3).tolist()
        for k in ("dist", "turn", "since", "after"):
            if k in info:
                ev[k] = info[k]
        self.pub_ev.publish(String(data=json.dumps(ev, ensure_ascii=False)))
        word = {"moving": "움직이기 시작", "moved": "옮겨짐", "hidden": "가려짐", "found": "다시 보임", "missing": "없음 의심"}[kind]
        extra = ""
        if kind == "moved":
            extra = f" {info['dist']:.2f} m, 회전 {info.get('turn', 0):+.0f}° ({ev.get('from')} → {ev.get('to')}, map)"
        elif kind in ("missing", "found"):
            extra = f" ({info.get('since', info.get('after'))} s)"
        self.get_logger().info(f"OBJ #{o.id} {o.name} {word}{extra} | 닻 찾음 {o.last['found']} 가려짐 {o.last['hidden']} 안보임 {o.last['clear']}")

    def publish_objects(self):
        T = self.lookup(self.map, self.odom)
        if T is None:
            return
        out = []
        for o in self.W.objs.values():
            c = apply(T, [o.p])[0]
            out.append({"id": o.id, "key": next((k for k, v in self.key2id.items() if v == o.id), None), "name": o.name, "state": o.state,
                        "x": round(float(c[0]), 3), "y": round(float(c[1]), 3), "th": round(math.degrees(o.th + T[2]), 1),
                        "anchors": apply(T, o.anchors()).round(3).tolist()})
        self.pub_obj.publish(String(data=json.dumps(out, ensure_ascii=False)))
        if time.time() - self.sum_t >= 60 and self.W.objs:
            self.sum_t = time.time()
            self.get_logger().info(f"OBJ 요약: {len(out)} 개 [" + ", ".join(f"#{x['id']} {x['name']} {x['state']} ({x['x']:+.2f},{x['y']:+.2f})" for x in out)
                                   + f"] | 스캔 {self.n_scan} 장, 평균 {self.ms / max(1, self.n_scan):.1f} ms")
            self.n_scan, self.ms = 0, 0.0


def main():
    rclpy.init()
    rclpy.spin(WorldModel())


if __name__ == "__main__":
    main()
