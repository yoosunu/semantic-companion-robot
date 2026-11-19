#!/usr/bin/env python3
"""semantic_markers — 따라보기 결과를 RViz 이름표로 (11-20 Jasper: "라벨링한 거 이름표처럼 띄우면 재밌을 듯").

  입력  semantic/label  String JSON {"name","conf","why","views","wx","wy" (odom), "floor"} ← glance_node 따라보기가 끝날 때 하나씩
  출력  /semantic/markers  visualization_msgs/MarkerArray (1 Hz, 프레임 odom — RViz 고정 프레임이 map 이어도 TF 로 따라옴)
  모양  핀: 바닥 점 + 막대 + 흰 이름 + 아래 작은 점수. 색 (점·막대·점수) = 확신 초록, 바닥 위험물 (발닦개 등) 빨강, "?" 회색
  같은 자리 (merge 0.30 m 안) 는 하나로: 새 결과가 확신이거나 예전 것이 "?" 면 바꿈 (확신을 "?" 로 덮지 않음)
  기억은 켜져 있는 동안만 (디스크 저장 안 함 — 지도 이름표 층은 나중에)
"""
import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

GREEN, RED, GRAY = (0.2, 0.85, 0.3), (0.95, 0.25, 0.25), (0.6, 0.6, 0.6)
PIN_H = 0.25                                       # 핀 막대 높이 (m)


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
        """이름표 = 핀: 바닥 점 (색 = 상태) + 얇은 막대 + 흰 이름 + 아래 작은 점수 (11-20 Jasper: 사람들한테 보여주기엔 웃김 ㅋㅋ).
           RViz 글자는 공백을 'A' 폭 두 배로 그려 "trash ··· can" 처럼 벌어짐 → 공백 대신 '-'"""
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        def mk(i, kind, z, rgb, a=1.0):
            m = Marker()
            m.header.frame_id, m.header.stamp = "odom", now
            m.ns, m.id, m.type, m.action = "semantic", i, kind, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = 0.0, 0.0, z
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, a)
            return m

        for i, lab in enumerate(self.labels):
            x, y, rgb = float(lab["wx"]), float(lab["wy"]), color_of(lab)
            dot = mk(i * 4, Marker.SPHERE, 0.0, rgb)
            dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = x, y, 0.02
            dot.scale.x = dot.scale.y = dot.scale.z = 0.05
            pin = mk(i * 4 + 1, Marker.LINE_LIST, 0.0, rgb, 0.8)
            pin.scale.x = 0.008
            pin.points = [Point(x=x, y=y, z=0.02), Point(x=x, y=y, z=PIN_H)]
            name = mk(i * 4 + 2, Marker.TEXT_VIEW_FACING, 0.0, (1.0, 1.0, 1.0))
            name.pose.position.x, name.pose.position.y, name.pose.position.z = x, y, PIN_H + 0.06
            name.scale.z = 0.08
            name.text = lab["name"].replace(" ", "-")
            conf = mk(i * 4 + 3, Marker.TEXT_VIEW_FACING, 0.0, rgb)
            conf.pose.position.x, conf.pose.position.y, conf.pose.position.z = x, y, PIN_H + 0.005
            conf.scale.z = 0.05
            conf.text = "" if lab["name"].endswith("?") else f"{lab.get('conf', 0):.2f}"
            arr.markers += [dot, pin, name, conf]
        self.pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(Markers())


if __name__ == "__main__":
    main()
