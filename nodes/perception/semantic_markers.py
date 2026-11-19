#!/usr/bin/env python3
"""semantic_markers — 따라보기 결과를 RViz 이름표로 (11-20 Jasper: "라벨링한 거 이름표처럼 띄우면 재밌을 듯").

  입력  semantic/label  String JSON {"name","conf","why","views","wx","wy" (odom), "floor"} ← glance_node 따라보기가 끝날 때 하나씩
  출력  /semantic/markers  visualization_msgs/MarkerArray (1 Hz, 프레임 odom — RViz 고정 프레임이 map 이어도 TF 로 따라옴)
  색    확신 = 초록, 바닥 위험물 이름 (발닦개 등) = 빨강, "?" 또는 "이름?" = 회색
  같은 자리 (merge 0.30 m 안) 는 하나로: 새 결과가 확신이거나 예전 것이 "?" 면 바꿈 (확신을 "?" 로 덮지 않음)
  기억은 켜져 있는 동안만 (디스크 저장 안 함 — 지도 이름표 층은 나중에)
"""
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

GREEN, RED, GRAY = (0.2, 0.85, 0.3), (0.95, 0.25, 0.25), (0.6, 0.6, 0.6)


def color_of(lab):
    if lab["name"].endswith("?") or lab["name"] == "?":
        return GRAY
    return RED if lab.get("floor") else GREEN


def merge(labels, new, r=0.30):
    """같은 자리면 바꿀지 판단해 목록 갱신 → "new" / "replaced" / "kept" """
    sure = not new["name"].endswith("?")
    for i, old in enumerate(labels):
        if math.hypot(old["wx"] - new["wx"], old["wy"] - new["wy"]) < r:
            if sure or old["name"].endswith("?"):
                labels[i] = new
                return "replaced"
            return "kept"
    labels.append(new)
    return "new"


class Markers(Node):
    def __init__(self):
        super().__init__("semantic_markers")
        self.labels = []
        self.create_subscription(String, "semantic/label", self.on_label, 10)
        self.pub = self.create_publisher(MarkerArray, "/semantic/markers", 1)
        self.create_timer(1.0, self.tick)
        self.get_logger().info("LABELS 준비: semantic/label → /semantic/markers (odom)")

    def on_label(self, m):
        lab = json.loads(m.data)
        how = merge(self.labels, lab)
        self.get_logger().info(f"LABELS {lab['name']} ({lab['wx']:+.2f}, {lab['wy']:+.2f}) → {how} | 지금 {len(self.labels)} 개")
        self.tick()

    def tick(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, lab in enumerate(self.labels):
            r, g, b = color_of(lab)
            for kind, z in ((Marker.SPHERE, 0.03), (Marker.TEXT_VIEW_FACING, 0.22)):
                mk = Marker()
                mk.header.frame_id, mk.header.stamp = "odom", now
                mk.ns, mk.id, mk.type, mk.action = "semantic", i * 2 + (kind == Marker.TEXT_VIEW_FACING), kind, Marker.ADD
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(lab["wx"]), float(lab["wy"]), z
                mk.pose.orientation.w = 1.0
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 1.0
                if kind == Marker.SPHERE:
                    mk.scale.x = mk.scale.y = mk.scale.z = 0.06
                else:
                    mk.scale.z = 0.09
                    mk.text = lab["name"] if lab["name"].endswith("?") else f"{lab['name']} {lab.get('conf', 0):.2f}"
                arr.markers.append(mk)
        self.pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(Markers())


if __name__ == "__main__":
    main()
