#!/usr/bin/env python3
"""저장된 지도(logs/maps/*.pgm)로 scr_explore 의 경계·관찰점 확인. 비용지도는 Nav2 처럼 부풀려 흉내.
  python3 tests/frontier_map_sim.py logs/maps/room_20261108_2013 [robot_x robot_y]
"""
import math, os, re, sys, types
import numpy as np
sys.argv, ARGS = sys.argv[:1], sys.argv[1:]
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "explore_sim.py")).read().split("def make(")[0])
base = ARGS[0]; rx, ry = (float(ARGS[1]), float(ARGS[2])) if len(ARGS) >= 3 else (0.0, 0.0)
y = open(base + ".yaml").read()
res = float(re.search(r"resolution:\s*([\d.]+)", y).group(1))
org = [float(v) for v in re.search(r"origin:\s*\[([^\]]+)\]", y).group(1).split(",")[:2]]
raw = open(base + ".pgm", "rb").read(); tk = re.findall(rb"\S+", re.sub(rb"#[^\n]*", b"", raw[:200]))[:4]; w, h = int(tk[1]), int(tk[2])
img = np.frombuffer(raw[-w * h:], np.uint8).reshape(h, w)[::-1]
occ, free = img < 50, img > 250
slam = np.where(occ, 100, np.where(free, 0, -1)).astype(np.int16)
d = np.full((h, w), 9.0); cur = occ.copy(); d[cur] = 0
for i in range(1, int(0.25 / res) + 1):
    p = np.pad(cur, 1); cur = cur | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:]; d[cur & (d > 8)] = i * res
infl = np.where(d <= 0.12, 99, np.where(d <= 0.25, np.rint(98 * np.exp(-4 * (d - 0.12))), 0)).astype(np.int16)
cost = np.where(free | occ, np.maximum(0, infl), -1).astype(np.int16); cost[occ] = 100
info = types.SimpleNamespace(width=w, height=h, resolution=res, origin=types.SimpleNamespace(position=types.SimpleNamespace(x=org[0], y=org[1])))
X = E.Explorer("t"); X.grid = types.SimpleNamespace(info=info, data=slam.ravel().tolist())
X.cmap = types.SimpleNamespace(info=info, data=cost.ravel().tolist()); X.odom_hist.append((0, rx, ry, 0.0))
fr = X.find_frontiers()
print(f"로봇 ({rx}, {ry}) → 경계 {len(fr)}개 (30 cm 이상)")
for f in sorted(fr, key=lambda f: -f["size"]):
    v = f"관찰점 ({f['vx']:+.2f}, {f['vy']:+.2f}) 경계까지 {math.hypot(f['vx'] - f['x'], f['vy'] - f['y']):.2f} m" if f["vx"] is not None else "관찰점 없음"
    print(f"   경계 ({f['x']:+.2f}, {f['y']:+.2f}) {f['size']:.2f} m → {v}")
