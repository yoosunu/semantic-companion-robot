#!/usr/bin/env python3
"""[Pi] 침대 한 번 찍기 (11-24) — 마스크·상자가 바닥에 얼마나 맞게 내려오나 + LiDAR 다리와 맞나. 주행 없음.

  준비: 브리지(+LiDAR) + 카메라 640 480 만 켜기 (bash run/look_test.sh 대신 아래 [Pi] 명령). look·glance 는 끄기 (고개를 이 도구가 씀)
  쓰기: ~/yolo_venv/bin/python tools/lab/bed_snap.py [이름]       → ~/calib/bed_set/<이름>/
        ~/yolo_venv/bin/python tools/lab/bed_snap.py --redo <폴더>  → 찍은 걸로 분석만 다시 (ROS 없이)
  자세: 틸트 −15, 팬 −40 −20 0 +20 +40. 자세마다 고개 멈춘 뒤 원본 사진 + 그 순간 /scan 저장
  분석 (자세마다):
    - YOLO26-seg (COCO: bed couch dining table) + YOLOE (가구 단어) → 마스크
    - 상자 아래 변 → 바닥 (파랑), 마스크 열마다 가장 아래 픽셀 → 바닥 (빨강)
    - LiDAR 점 (회색), 작고 떨어진 덩어리 = 다리 후보 (초록 원, 지름 ≤ 8 cm)
    - 다리 후보마다 빨강 (마스크 아래) 까지 가장 가까운 거리 → 출력
  결과: snap_<자세>.png (사진 + 마스크), top_<자세>.png (위에서 본 그림, 로봇 기준 m), report.txt
  11-25: + 다리 잇기 (nodes/navigation/furniture.py) — 5 자세의 YOLO26 bed (≥ 0.5) 를 모아 사각형 → furniture.png + report 끝 줄
"""
import json
import math
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "nodes", "perception"))
sys.path.insert(0, os.path.join(HERE, "..", "..", "nodes", "navigation"))
import furniture as fu  # noqa: E402
import ground  # noqa: E402
from yolo_onnx import Yolo  # noqa: E402

POSES = [(-40, -15), (-20, -15), (0, -15), (20, -15), (40, -15)]
FURN = ("bed", "couch", "dining table", "sofa", "table")
LASER_YAW = math.pi                       # URDF laser_yaw (라이다 0° = 로봇 뒤)


def scan_to_base(sc):
    """LaserScan 저장본 → 로봇(base) 좌표 N×2"""
    r = np.asarray(sc["ranges"], float)
    a = sc["angle_min"] + np.arange(len(r)) * sc["angle_increment"]
    ok = np.isfinite(r) & (r > 0.05) & (r < 4.0)
    a, r = a[ok] + LASER_YAW, r[ok]
    return np.column_stack([ground.LASER_X + r * np.cos(a), r * np.sin(a)])


def leg_clusters(pts, gap=0.05, max_d=0.08, clear=0.15):
    """스캔 순서 그대로의 점들을 gap 으로 끊어 덩어리 → 지름 ≤ max_d, 다른 점이 clear 안에 없는 것 = 다리 후보 [(cx, cy, 지름, 점수)]
       (로봇 원점 기준으로 각도 정렬하면 라이다가 7.8 cm 앞이라 다리 가장자리와 뒷벽 점이 섞임 → 스캔 순서 유지)"""
    if len(pts) == 0:
        return []
    p = pts
    cut = np.where(np.hypot(*np.diff(p, axis=0).T) > gap)[0] + 1
    out = []
    for c in np.split(p, cut):
        d = float(np.hypot(*(c.max(0) - c.min(0))))
        if d > max_d or len(c) < 2:
            continue
        cen = c.mean(0)
        others = p[np.hypot(*(p - cen).T) > d / 2 + 0.01]
        if len(others) and np.hypot(*(others - cen).T).min() < clear:
            continue
        out.append((float(cen[0]), float(cen[1]), d, len(c)))
    return out


def ray_bearing(px, K, D, pan, tilt):
    """픽셀 하나 → 그 광선의 방향 (로봇 기준 각도 °, 왼쪽 +). 바닥에 안 닿는 광선도 됨 (거리 안 씀)"""
    und = cv2.fisheye.undistortPoints(np.array([[px]], float), K, D).reshape(2)
    R, _ = ground.cam_pose(pan, tilt)
    w = R @ np.array([und[0], und[1], 1.0])
    return math.degrees(math.atan2(w[1], w[0]))


def mask_bottom(mask, step=4):
    """마스크 열마다 가장 아래 픽셀 (step 열 간격) → N×2"""
    cols = np.where(mask.any(0))[0][::step]
    return np.array([[x, mask[:, x].nonzero()[0].max()] for x in cols], float)


def analyze(d):
    meta = json.load(open(os.path.join(d, "poses.json")))
    K, D, _ = ground.load_calib()
    m26 = Yolo(os.path.expanduser("~/models/yolo26s-seg.onnx"), size=320, conf=0.2)
    mE = Yolo(os.path.expanduser("~/models/yoloe-26s-seg-home.onnx"), size=320, conf=0.1)
    rep = []
    F, last = fu.Furniture(), None
    for p in meta["poses"]:
        bgr = cv2.imread(os.path.join(d, p["file"]))
        pan, tilt = p["pan"], p["tilt"]
        pts = scan_to_base(p["scan"])
        legs = leg_clusters(pts)
        dets = [("26",) + x for x in m26.detect(bgr, masks=True)] + [("E",) + x for x in mE.detect(bgr, masks=True)]
        dets = [x for x in dets if x[1] in FURN and len(x) == 8]
        snap = bgr.copy()
        S, O = 200, np.array([100, 400])                       # 위에서 본 그림: 1 m = 200 px, 로봇 (0,0) 을 아래 가운데쪽
        top = np.full((500, 600, 3), 255, np.uint8)
        to = lambda xy: (int(300 - xy[1] * S), int(O[1] + 0 - xy[0] * S + 0))   # x 앞 = 위, y 왼 = 왼쪽
        cv2.rectangle(top, to((0.126, 0.13)), to((-0.14, -0.13)), (120, 120, 120), 1)
        for x, y in pts:
            cv2.circle(top, to((x, y)), 1, (150, 150, 150), -1)
        for cx, cy, dia, n in legs:
            cv2.circle(top, to((cx, cy)), 8, (0, 160, 0), 2)
        line = f"[{p['file']}] pan {pan:+d} tilt {tilt:+d} | LiDAR 다리 후보 {len(legs)}: " + \
               ", ".join(f"({cx:+.2f},{cy:+.2f}) ⌀{dia*100:.0f}cm" for cx, cy, dia, _ in legs)
        rep.append(line)
        for k, (mdl, name, conf, bx, by, bw, bh, mask) in enumerate(dets):
            col = (0, 0, 255) if mdl == "26" else (255, 0, 255)
            ov = snap.copy(); ov[mask] = col; snap = cv2.addWeighted(ov, 0.35, snap, 0.65, 0)
            cv2.rectangle(snap, (int(bx), int(by)), (int(bx + bw), int(by + bh)), (255, 120, 0), 1)
            cv2.putText(snap, f"{mdl}:{name} {conf:.2f}", (int(bx) + 3, int(by) + 15), 0, 0.5, col, 1)
            box_px = np.column_stack([np.linspace(bx, bx + bw, 20), np.full(20, by + bh)])
            box_fl = ground.pixels_to_floor(box_px, K, D, pan, tilt)
            mb = mask_bottom(mask)
            mb_fl = ground.pixels_to_floor(mb, K, D, pan, tilt) if len(mb) else np.zeros((0, 2))
            for q in box_fl[np.isfinite(box_fl[:, 0])]:
                cv2.circle(top, to(q), 2, (255, 120, 0), -1)
            ok = mb_fl[np.isfinite(mb_fl[:, 0])]
            for q in ok:
                cv2.circle(top, to(q), 2, col, -1)
            # 광선 방향 범위 (거리 없이): 마스크 좌우 끝 열 → 각도
            cols = np.where(mask.any(0))[0]
            bear = [ray_bearing([c, mask[:, c].nonzero()[0].max()], K, D, pan, tilt) for c in (cols.min(), cols.max())]
            if mdl == "26" and name in fu.FURNITURE and conf >= 0.5:      # 다리 잇기 (로봇 안 움직임 → odom = 로봇 좌표)
                edge_cut = (cols.min() <= 3, cols.max() >= mask.shape[1] - 4)   # 화면 끝에 잘린 쪽은 방향 증거 아님 (None)
                F.add_view(ground.cam_pose(pan, tilt)[1][:2], [None if c else math.radians(v) for v, c in zip(bear, edge_cut)], mb_fl, pts, (0.0, 0.0))
            cut = "잘림 " + "".join(s for s, c in (("왼", cols.min() <= 3), ("오", cols.max() >= mask.shape[1] - 4), ("아래", mask[-3:].any())) if c)
            near = []
            for cx, cy, _, _ in legs:
                dd = np.hypot(*(ok - [cx, cy]).T).min() if len(ok) else float("nan")
                near.append(f"({cx:+.2f},{cy:+.2f})→{dd*100:.0f}cm")
            rep.append(f"   {mdl}:{name} {conf:.2f} 방향 {bear[0]:+.0f}°~{bear[1]:+.0f}° {cut if cut != '잘림 ' else ''}| "
                       f"마스크 아래 바닥점 {len(ok)}/{len(mb)} (x {ok[:, 0].min() if len(ok) else 0:.2f}~{ok[:, 0].max() if len(ok) else 0:.2f} m) | "
                       f"다리→마스크 아래: {', '.join(near) or '-'}")
        cv2.putText(top, f"pan {pan:+d} tilt {tilt:+d}  grid 0.5 m", (5, 20), 0, 0.5, (0, 0, 0), 1)
        for gx in np.arange(0, 2.01, 0.5):
            cv2.line(top, to((gx, 1.5)), to((gx, -1.5)), (230, 230, 230), 1)
        stem = p["file"].rsplit(".", 1)[0]
        cv2.imwrite(os.path.join(d, f"snap_{stem}.png"), snap)
        cv2.imwrite(os.path.join(d, f"top_{stem}.png"), top)
        last = pts
    r = F.shape()
    if r is None:
        rep.append(f"다리 잇기: 영역 없음 (확정 다리 {len(F.legs)} 개 — 앞줄 2 개 필요)")
    else:
        rep.append(f"다리 잇기: 다리 {[(round(l[0], 2), round(l[1], 2)) for l in F.legs]} 간격 {r['spacing']:.2f} | 길이 {r['hi'] - r['lo']:.2f} "
                   f"깊이 {r['depth']:.2f} | 꼭짓점 {np.round(r['corners'], 2).tolist()} | 끝 오른쪽: {r['why']['lo']} / 왼쪽: {r['why']['hi']} | 깊이: {r['why']['depth']}")
        S, img = 150, np.full((420, 640, 3), 255, np.uint8)
        to = lambda q: (int(400 - q[1] * S), int(380 - q[0] * S))
        for x, y in (last if last is not None else []):
            cv2.circle(img, to((x, y)), 1, (150, 150, 150), -1)
        cv2.polylines(img, [np.array([to(q) for q in r["corners"]], np.int32)], True, (0, 0, 220), 2)
        for l in F.legs:
            cv2.circle(img, to(l[:2]), 6, (0, 150, 0), -1)
        cv2.rectangle(img, to((0.126, 0.13)), to((-0.14, -0.13)), (90, 90, 90), 1)
        cv2.putText(img, "red = blocked (furniture.py), green = legs", (8, 18), 0, 0.45, (0, 0, 0), 1)
        cv2.imwrite(os.path.join(d, "furniture.png"), img)
    open(os.path.join(d, "report.txt"), "w").write("\n".join(rep) + "\n")
    print("\n".join(rep))
    print(f"끝: {d}  (snap_*.png 사진+마스크, top_*.png 위에서 본 그림: 파랑 = 상자 아래, 빨강 = YOLO26 마스크 아래, 보라 = YOLOE, 초록 원 = LiDAR 다리 후보)")


def snap(name):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from std_msgs.msg import Float32MultiArray
    out = os.path.expanduser(f"~/calib/bed_set/{name}")
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    n = rclpy.create_node("bed_snap")
    imgs, scans = [], []
    n.create_subscription(Image, "camera/image_raw", lambda m: imgs.append(m), qos_profile_sensor_data)
    n.create_subscription(LaserScan, "/scan", lambda m: scans.append(m), qos_profile_sensor_data)
    pub = n.create_publisher(Float32MultiArray, "pantilt/cmd", 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print("pantilt/cmd 듣는 노드 없음 (브리지 꺼짐?) → 그만")
        return None
    log = []
    for pan, tilt in POSES:
        for _ in range(5):
            pub.publish(Float32MultiArray(data=[float(pan), float(tilt), 400.0, 10.0]))
            rclpy.spin_once(n, timeout_sec=0.05)
        end = time.time() + 3.0
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        imgs.clear(); scans.clear()
        t1 = time.time()
        while (len(imgs) < 3 or not scans) and time.time() - t1 < 8:
            rclpy.spin_once(n, timeout_sec=0.2)
        if not imgs or not scans:
            print(f"pan {pan} 사진 {len(imgs)} 스캔 {len(scans)} → 건너뜀 (카메라·LiDAR 켜졌나?)")
            continue
        m, sc = imgs[-1], scans[-1]
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        bgr = a if m.encoding == "bgr8" else np.ascontiguousarray(a[:, :, ::-1])
        f = f"pan{pan:+d}_tilt{tilt:+d}.png"
        cv2.imwrite(os.path.join(out, f), bgr)
        log.append({"file": f, "pan": pan, "tilt": tilt, "size": [m.width, m.height],
                    "scan": {"angle_min": sc.angle_min, "angle_increment": sc.angle_increment,
                             "ranges": [float(r) if math.isfinite(r) else -1.0 for r in sc.ranges]}})
        print(f"저장 {f} (스캔 {len(sc.ranges)} 점)", flush=True)
    for _ in range(5):
        pub.publish(Float32MultiArray(data=[0.0, -20.0, 400.0, 10.0]))
        rclpy.spin_once(n, timeout_sec=0.05)
    json.dump({"poses": log, "date": time.strftime("%Y-%m-%d %H:%M"),
               "ground_py": f"TILT_OFFSET {ground.TILT_OFFSET}, LASER_X {ground.LASER_X}"},
              open(os.path.join(out, "poses.json"), "w"))
    n.destroy_node(); rclpy.shutdown()
    return out


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--redo":
        analyze(os.path.expanduser(sys.argv[2]))
    else:
        d = snap(sys.argv[1] if len(sys.argv) > 1 else time.strftime("%m%d_%H%M"))
        if d:
            analyze(d)
