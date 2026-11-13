#!/usr/bin/env python3
"""/camera/image_raw 한 장을 ~/cam_snap.png (또는 인자 경로) 로 (OpenCV·cv_bridge 없이, PNG 는 zlib 로 직접). bgr8/rgb8 만."""
import os
import struct
import sys
import time
import zlib

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init()
n = rclpy.create_node("cam_snap")
got = []
n.create_subscription(Image, "camera/image_raw", lambda m: got.append(m), qos_profile_sensor_data)
t0 = time.time()
while not got and time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.2)
if not got:
    print("사진 못 받음 (8 s) — 노드 로그 확인"); sys.exit(1)
m = got[0]
if m.encoding not in ("bgr8", "rgb8"):
    print(f"인코딩 {m.encoding} — 저장 안 함 (bgr8/rgb8 만)"); sys.exit(1)
raw = bytearray()
data = bytes(m.data)
for y in range(m.height):
    line = bytearray(data[y * m.step: y * m.step + m.width * 3])
    if m.encoding == "bgr8":
        line[0::3], line[2::3] = line[2::3], line[0::3]
    raw += b"\x00" + line                       # PNG 줄마다 필터 0


def chunk(t, d):
    return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)


path = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/cam_snap.png")
with open(path, "wb") as f:
    f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", m.width, m.height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(bytes(raw), 6)) + chunk(b"IEND", b""))
print(f"저장: {path} ({m.width}x{m.height}, {m.encoding})")
