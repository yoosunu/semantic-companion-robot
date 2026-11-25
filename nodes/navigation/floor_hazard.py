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
  밑이 뜬 가구 (11-25, furniture.py, tests/furniture_sim.py): semantic/seen 의 bed (새 가구 ≥ furn_on 0.5, 이미 있는 자리 ≥ furn_keep_on 0.3;
        glance 가 bearing·cam·robot·foot_dense 실어 줌)
        + 그 순간 /scan → FurnitureMap (다리 잇기) → 사각형을 같은 /semantic/cloud 에. 로그 FURN
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
import furniture as fu  # noqa: E402
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
        self.H = hz.HazardMap(on=p("on", 0.4).value, scan_max=p("scan_max", 2).value)   # 11-20: 0.5 → 0.4 (바닥형은 전부 YOLOE 단어, 따라보기와 같게)
        self.chair_method = p("chair_method", "column").value            # 11-26 의자: column = 1차 (기둥) 만 (기본, 2차 걷어냄 — furniture.Chair 설명),
                                                                         #   auto = 1차 → 없으면 2차, contact = 2차만 (숫자 "2" 는 ROS 가 정수로 읽어 타입 오류)
        self.F = fu.FurnitureMap(on=p("furn_on", 0.5).value, keep_on=p("furn_keep_on", 0.3).value,   # 이중 문턱: 새 가구 0.5, 기존 자리 0.3
                                 chair_kw={"use1": self.chair_method != "contact", "use2": self.chair_method in ("auto", "contact")})
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.scan = None
        self.create_subscription(String, "semantic/seen", self.on_seen, 10)
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m), qos_profile_sensor_data)
        self.create_subscription(String, "/semantic/clear", self.on_clear, 10)
        self.create_subscription(String, "floor/blobs", self.on_blobs, 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/semantic/cloud", 1)
        self.pub_state = self.create_publisher(String, "/semantic/hazards", 1)
        self.pub_furn = self.create_publisher(String, "/semantic/furniture", 1)   # 11-25 가구 도형 (map) → 탐색기 "가구 밑" 경계
        self.create_timer(1.0 / p("rate_hz", 1.0).value, self.tick)
        # 지우기: 점 구름만 비우면 costmap 층 (clearing: false) 에 칸이 남음 → 비용지도 전체 초기화 (LiDAR·위스커는 다음 주기에 다시 채움)
        self.clear_cli = [self.create_client(ClearEntireCostmap, s) for s in (
            "/global_costmap/clear_entirely_global_costmap", "/local_costmap/clear_entirely_local_costmap")] if ClearEntireCostmap else []
        self.drift = hz.DriftWatch(dist=p("drift_m", 0.05).value, gap=p("drift_gap_s", 5.0).value)   # 11-25 SLAM 보정 → 지역 costmap 옛 표시
        self.get_logger().info(f"HAZARD 준비: 의자 {dict(column='1차 (기둥) 만', auto='1차 → 없으면 2차', contact='2차만 (1차 끔)').get(self.chair_method, self.chair_method)}, 좌표 {self.frame}, 바닥형 {sorted(self.H.types)}, 점수 ≥ {self.H.on}, LiDAR 점 ≤ {self.H.scan_max}")

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
        furn = [o for o in objs if o["name"] in fu.FURNITURE and o.get("bearing") is not None]
        other = sorted({o["name"] for o in objs if o["name"] not in self.H.types and o["name"] not in fu.FURNITURE})
        if other:                                                       # 11-20: rug 를 조용히 버려서 원인 찾기 어려웠음 → 남김
            self.get_logger().info(f"HAZARD #{j.get('id')} 바닥형 아님 (이름표만): {', '.join(other)}")
        objs = [o for o in objs if o["name"] in self.H.types]
        if not objs and not furn:
            return
        To, Tb, pts = self.lookup("odom"), self.lookup("base_footprint"), self.scan_points()
        if To is None or Tb is None or pts is None:
            self.get_logger().warn(f"HAZARD #{j.get('id')} 건너뜀: TF·스캔 없음")
            return
        lag = round(time.time() - float(j["t_img"]), 2) if j.get("t_img") else None   # 11-26 사진 → 여기 (glance 처리 + 전달)
        for o in furn:
            self.on_furniture(j.get("id"), o, To, Tb, pts, lag)
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

    def on_furniture(self, jid, o, To, Tb, pts, lag=None):
        """밑이 뜬 가구 한 장 → FurnitureMap (odom 값을 map 으로, 방향엔 odom→map 회전만 더함)"""
        foot = apply(To, o.get("foot_dense") or o.get("footprint") or np.zeros((0, 2)))   # 촘촘한 발자국 (가는 다리 안 놓침)
        cam = apply(To, [o["cam"]])[0]
        bear = [None if b is None else b + To[2] for b in o["bearing"]]
        before = {id(it): (it.name, (it.shape() or {}).get("corners")) for it in self.F.items}
        F, found, how = self.F.observe(o["name"], o["conf"], cam, bear, foot, pts, np.array(Tb[:2]))
        after = {id(it): (it.shape() or {}).get("corners") for it in self.F.items}
        old = [f"{nm} 사라짐/옮김" for k, (nm, c) in before.items() if k not in after and c is not None]
        old += [f"{nm} 줄어듦/밀림" for k, (nm, c) in before.items() if k in after and fu.stale(c, after[k])]
        if old:                                                         # 11-26 Jasper: 의자를 밀었는데 옛 원이 남음 → costmap 다시 그리기
            self.get_logger().info(f"FURN #{jid} 옛 자리 지움: {', '.join(old)} → 비용지도 다시 그리기")
            self.clear_costmaps()
        if F is None:
            self.get_logger().info(f"FURN #{jid} {o['name']} {o['conf']:.2f} → {how}")
            return
        if F not in self.F.items:                                       # 11-26: 기둥 못 찾은 새 의자 (안 남김)
            self.get_logger().info(f"FURN #{jid} {o['name']} {o['conf']:.2f} 새 의자 안 남김 | 1차 실패: {getattr(F, 'miss', '')}")
            return
        r = F.shape()
        legs = ", ".join(f"({l[0]:+.2f},{l[1]:+.2f})x{l[2]}" for l in F.legs)
        cutw = "".join(w for w, b in zip(("왼", "오"), o["bearing"]) if b is None)
        word = "새 가구" if how == "new" else ("같은 가구" + (" (낮은 점수라 기존 자리로 받음)" if "낮은" in how else ""))
        head = (f"FURN #{jid} {o['name']} {o['conf']:.2f} {word}" + (f" (사진→반영 {lag:.2f} s)" if lag is not None else "") + f" | 이번 다리 {len(found)} 개"
                + (f" | 잘림 {cutw}" if cutw else "") + f" | 다리 {len(F.legs)}: {legs}")
        miss = getattr(F, "miss", "")                                   # 11-25 의자 1차 (기둥) 실패 이유
        if r is None:
            self.get_logger().info(head + " | 영역 없음 (앞줄 다리 2 개를 한 장에서 같이 봐야 함 / 의자는 기둥 1 개 또는 접점)"
                                   + (f" | 1차 실패: {miss}" if miss else ""))
        elif r.get("round"):                                            # 11-25 의자: 1차 = 기둥 원, 2차 = 접점 + 끝 방향
            P, a = r["corners"], r.get("alt")
            alt = (f" | 2차 비교: {a['why']['lo']} x {a['corners'][:, 0].min():+.2f}~{a['corners'][:, 0].max():+.2f} "
                   f"y {a['corners'][:, 1].min():+.2f}~{a['corners'][:, 1].max():+.2f}") if a else ""
            self.get_logger().info(head + f" | {r.get('method', 1)}차 중심 ({r['center'][0]:+.2f}, {r['center'][1]:+.2f}) "
                                   f"크기 {r['radius']:.2f} | x {P[:, 0].min():+.2f}~{P[:, 0].max():+.2f} y {P[:, 1].min():+.2f}~{P[:, 1].max():+.2f}"
                                   f" | {r['why']['lo']}" + (f" | 1차 실패: {miss}" if miss and r.get("method") == 2 else "") + alt)
        else:
            self.get_logger().info(head + f" | 길이 {r['hi'] - r['lo']:.2f} 깊이 {r['depth']:.2f} 간격 {r['spacing']:.2f} | "
                                   f"끝: {r['why']['lo']} / {r['why']['hi']} | 깊이: {r['why']['depth']} | "
                                   f"꼭짓점 {np.round(r['corners'], 2).tolist()}")

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

    def clear_costmaps(self, local_only=False):
        for cl in self.clear_cli[1:] if local_only else self.clear_cli:
            if cl.service_is_ready():
                fut = cl.call_async(ClearEntireCostmap.Request())
                fut.add_done_callback(lambda _: self.publish_cloud())  # 지운 직후 우리 점 구름 바로 다시 (1 초 빈틈 줄임)

    def check_drift(self):
        """11-25 map↔odom 이 한 칸 넘게 바뀌면 지역 costmap 만 지움 (전역은 map 좌표라 괜찮음)."""
        To, Tb = self.lookup("odom"), self.lookup("base_footprint")
        if To is None or Tb is None:
            return
        x0, y0, th = To                                                 # 로봇의 odom 좌표 = To⁻¹ · Tb
        c, s = math.cos(-th), math.sin(-th)
        p = ((Tb[0] - x0) * c - (Tb[1] - y0) * s, (Tb[0] - x0) * s + (Tb[1] - y0) * c)
        d = self.drift.update(To, p, time.monotonic())
        if d is not None:
            self.get_logger().info(f"HAZARD 지역 costmap 지움: map↔odom 어긋남 {d * 100:.1f} cm "
                                   f"(odom {To[0]:+.3f},{To[1]:+.3f},{math.degrees(To[2]):+.1f}°)")
            self.clear_costmaps(local_only=True)

    def on_clear(self, _):
        self.get_logger().info(f"HAZARD 지움: {self.H.clear()} 개, 가구 {self.F.clear()} 개 + 비용지도 초기화")
        self.tick()
        self.clear_costmaps()

    def tick(self):
        self.publish_cloud()
        self.check_drift()
        st = self.H.state()
        To = self.lookup("odom") if st else None
        if To is not None:                                              # 11-20 따라보기용: glance_node 는 odom 좌표 → 같이 실어 줌
            x0, y0, th = To
            c, s = math.cos(-th), math.sin(-th)
            for h in st:
                dx, dy = h["cx"] - x0, h["cy"] - y0
                h["ox"], h["oy"] = round(dx * c - dy * s, 3), round(dx * s + dy * c, 3)
        self.pub_state.publish(String(data=json.dumps(st)))

    def publish_cloud(self):
        pts = np.vstack([self.H.cloud_points().reshape(-1, 2), self.F.cloud_points().reshape(-1, 2)])
        m = PointCloud2()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.height, m.width = 1, len(pts)
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
        m.is_bigendian, m.point_step, m.is_dense = False, 12, True
        m.row_step = 12 * len(pts)
        m.data = b"".join(struct.pack("<fff", float(x), float(y), MARK_Z) for x, y in pts)
        self.pub_cloud.publish(m)
        self.pub_furn.publish(String(data=json.dumps([{"name": F.name, "corners": np.round(r["corners"], 3).tolist()}
                                                      for F, r in self.F.shapes() if r is not None])))


def main():
    rclpy.init()
    rclpy.spin(FloorHazard())


if __name__ == "__main__":
    main()
