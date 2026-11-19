#!/usr/bin/env python3
"""floor_hazard — 센서 사각 바닥 위험물 → costmap (11-19). 판단은 hazards.py (순수 계산, tests/hazard_sim.py).

  흐름: glance_node (눈) → semantic/seen (이름 + 윤곽·발자국, odom 좌표)
        → 여기 (판사): 바닥형 이름 + 점수 ≥ on + 그 모양 안에 LiDAR 점 ≤ scan_max → 표시
        → /semantic/cloud (map 좌표 점, costmap semantic_layer 가 받음. LiDAR 가 안 지움 = 위스커 층과 같은 설정)
  왜 (11-19 Jasper): 발닦개는 LiDAR 높이 아래·ToF 엔 너무 얇음·자이로는 헛돈다는 것만 앎 → 애초에 안 밟는 게 최선 → 비전만 가능.
        센서가 볼 수 있는 건 (의자 다리, 의자에 걸린 수건) 센서가 피함 → 이름표만.
  입력: semantic/seen (String JSON), /scan (LaserScan), /semantic/clear (아무 String → 전부 지움 + 비용지도 초기화)
        지우기: ros2 topic pub --once /semantic/clear std_msgs/msg/String "{data: all}"
  출력: /semantic/cloud (PointCloud2, frame), /semantic/hazards (String JSON 목록, 1 Hz — 나중에 탐색기가 봄)
  파라미터: frame (map; SLAM 없이 시험할 땐 odom), on 0.5, scan_max 2, rate_hz 1.0
  2단계 (11-20): floor/blobs (바닥 감지, 초당 2 번) 의 가장 가까운 점을 표시 근처에서 모아 "다가갈수록 끌려오나" →
        끌려오면 떠 있는 것 (발자국이 엉뚱) → 표시 빼고 비용지도 초기화. 안 끌려오면 "바닥 확인" (hazards.floating)
  지금 안 하는 것: 자동 지우기 (치운 발닦개), 탐색기 연결 (3단계)
"""
import json
import math
import os
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    from nav2_msgs.srv import ClearEntireCostmap
except ImportError:                                                     # Nav2 없이 시험할 때
    ClearEntireCostmap = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hazards as hz  # noqa: E402

MARK_Z = 0.05


def tf2d(t):
    """geometry_msgs/TransformStamped → (x, y, yaw)"""
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.transform.translation.x, t.transform.translation.y, yaw


def apply(T, pts):
    x, y, th = T
    pts = np.asarray(pts, float).reshape(-1, 2)
    c, s = math.cos(th), math.sin(th)
    return np.column_stack([x + pts[:, 0] * c - pts[:, 1] * s, y + pts[:, 0] * s + pts[:, 1] * c])


class FloorHazard(Node):
    def __init__(self):
        super().__init__("floor_hazard")
        p = self.declare_parameter
        self.frame = p("frame", "map").value
        self.remove_floating = p("remove_floating", False).value   # 11-20: 진짜 발닦개를 "떠 있음" 으로 두 번 뺌 → 원인 볼 때까지 관찰 모드
        self.H = hz.HazardMap(on=p("on", 0.5).value, scan_max=p("scan_max", 2).value)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.scan = None
        self.create_subscription(String, "semantic/seen", self.on_seen, 10)
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m), qos_profile_sensor_data)
        self.create_subscription(String, "/semantic/clear", self.on_clear, 10)
        self.create_subscription(String, "floor/blobs", self.on_blobs, 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/semantic/cloud", 1)
        self.pub_state = self.create_publisher(String, "/semantic/hazards", 1)
        self.create_timer(1.0 / p("rate_hz", 1.0).value, self.tick)
        # 지우기: 점 구름만 비우면 costmap 층 (clearing: false) 에 칸이 남음 → 비용지도 전체 초기화 (LiDAR·위스커는 다음 주기에 다시 채움)
        self.clear_cli = [self.create_client(ClearEntireCostmap, s) for s in (
            "/global_costmap/clear_entirely_global_costmap", "/local_costmap/clear_entirely_local_costmap")] if ClearEntireCostmap else []
        self.get_logger().info(f"HAZARD 준비: 좌표 {self.frame}, 바닥형 {sorted(self.H.types)}, 점수 ≥ {self.H.on}, LiDAR 점 ≤ {self.H.scan_max}")

    def lookup(self, src):
        try:
            return tf2d(self.tf.lookup_transform(self.frame, src, Time()))
        except Exception as e:                                          # noqa: BLE001
            self.get_logger().warn(f"HAZARD TF {self.frame}←{src} 없음: {e}", throttle_duration_sec=5.0)
            return None

    def scan_points(self):
        m = self.scan
        if m is None:
            return None
        T = self.lookup(m.header.frame_id)
        if T is None:
            return None
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        return apply(T, np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]))

    def on_seen(self, msg):
        j = json.loads(msg.data)
        objs = []
        if j.get("name") and not j["name"].endswith("?"):
            objs.append(j)
        objs += j.get("extras", [])
        other = sorted({o["name"] for o in objs if o["name"] not in self.H.types})
        if other:                                                       # 11-20: rug 를 조용히 버려서 원인 찾기 어려웠음 → 남김
            self.get_logger().info(f"HAZARD #{j.get('id')} 바닥형 아님 (이름표만): {', '.join(other)}")
        objs = [o for o in objs if o["name"] in self.H.types]
        if not objs:
            return
        To, Tb, pts = self.lookup("odom"), self.lookup("base_footprint"), self.scan_points()
        if To is None or Tb is None or pts is None:
            self.get_logger().warn(f"HAZARD #{j.get('id')} 건너뜀: TF·스캔 없음")
            return
        now = time.time()
        for o in objs:
            out = apply(To, o.get("outline") or np.zeros((0, 2)))
            foot = apply(To, o.get("footprint") or np.zeros((0, 2)))
            poly = hz.shape_for(o["name"], out, foot)
            res, why = self.H.judge(o["name"], o["conf"], poly, pts)
            if res == "mark":
                dist = math.hypot(*(poly.mean(0) - np.array(Tb[:2])))
                full = not o.get("cut", False)
                how = self.H.add(o["name"], o["conf"], poly, now, dist=dist, full=full)
                c = poly.mean(0)
                word = {"new": "새로", "replaced": "모양 바꿈", "joined": "조각 합침", "kept": "확인만"}[how]
                self.get_logger().info(f"HAZARD #{j.get('id')} {o['name']} {o['conf']:.2f} {'통째' if full else '잘림'} → 표시 ({word}) "
                                       f"중심 ({c[0]:+.2f}, {c[1]:+.2f}) 넓이 {hz.area(poly):.2f} m² 거리 {dist:.2f} m | {why} | 지금 {len(self.H.items)} 개")
            else:
                self.get_logger().info(f"HAZARD #{j.get('id')} {o['name']} {o['conf']:.2f} → 안 찍음 | {why}")

    def on_blobs(self, msg):
        """바닥 감지 덩어리 → 표시 근처면 모아서 떠 있나 판정 (2단계)"""
        if not self.H.items:
            return
        j = json.loads(msg.data)
        To = self.lookup("odom")
        if To is None or not j.get("blobs"):
            return
        X, Y, th = j["pose"]
        r = apply(To, [(X, Y)])[0]
        lag = round(self.get_clock().now().nanoseconds * 1e-9 - float(j.get("t", 0.0)), 2)   # 사진 시각 → 지금 (자세 시각 어긋남 확인용)
        c, s = math.cos(th), math.sin(th)
        pts, infos = [], []
        for b in j["blobs"]:
            if b.get("edge"):
                continue        # 11-20 실물: 다가가면 발닦개 앞쪽이 화면 아래로 빠짐 → "가장 가까운 점" = 화면 아래 끝선 = 로봇과 같이 움직임
                                #   → 진짜 발닦개를 "떠 있음 (25 cm 에 16 cm 끌려옴)" 으로 뺐음. 화면 끝에 걸친 덩어리는 기록 안 함
            p = apply(To, [(X + b["x"] * c - b["y"] * s, Y + b["x"] * s + b["y"] * c)])[0]
            pts.append(p)
            infos.append({"t": j.get("t"), "lag": lag, "near": b.get("near"), "head": j.get("head"), "area": b.get("area"),
                          "r": [round(r[0], 3), round(r[1], 3)], "p": [round(p[0], 3), round(p[1], 3)]})
        if pts:
            for res, it in self.H.observe_frame(r[0], r[1], pts, infos, remove=self.remove_floating):
                rec = " ; ".join(f"r({o['r'][0]:+.2f},{o['r'][1]:+.2f}) p({o['p'][0]:+.2f},{o['p'][1]:+.2f}) near {o['near']} "
                                 f"고개 {o['head']} 면적 {o['area']} 늦음 {o['lag']}s" for o in it.get("raw", [])[-8:] if o)
                if res == "floating":
                    act = "표시 뺌" if self.remove_floating else "관찰 모드라 표시 유지"
                    self.get_logger().warn(f"HAZARD {it['name']} 떠 있음? → {act} (다가간 {it['move']:.2f} m 에 {it['pull']:.2f} m 끌려옴) "
                                           f"| 지금 {len(self.H.items)} 개 | 기록: {rec}")
                    if self.remove_floating:
                        self.clear_costmaps()
                else:
                    self.get_logger().info(f"HAZARD {it['name']} 바닥 확인 (다가간 {it['move']:.2f} m 에 {it['pull']:+.2f} m) | 기록: {rec}")

    def clear_costmaps(self):
        for cl in self.clear_cli:
            if cl.service_is_ready():
                cl.call_async(ClearEntireCostmap.Request())

    def on_clear(self, _):
        self.get_logger().info(f"HAZARD 지움: {self.H.clear()} 개 + 비용지도 초기화")
        self.tick()
        self.clear_costmaps()

    def tick(self):
        pts = self.H.cloud_points()
        m = PointCloud2()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.height, m.width = 1, len(pts)
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
        m.is_bigendian, m.point_step, m.is_dense = False, 12, True
        m.row_step = 12 * len(pts)
        m.data = b"".join(struct.pack("<fff", float(x), float(y), MARK_Z) for x, y in pts)
        self.pub_cloud.publish(m)
        self.pub_state.publish(String(data=json.dumps(self.H.state())))


def main():
    rclpy.init()
    rclpy.spin(FloorHazard())


if __name__ == "__main__":
    main()
