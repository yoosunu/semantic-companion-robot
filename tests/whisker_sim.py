#!/usr/bin/env python3
"""Whisker 흉내 시험 (로봇 없이). 가짜 rclpy + 간단한 물리 + 벽 방 LiDAR + 실측 IMU 기록 재생.

  python3 tests/whisker_sim.py            장면 시험 (상태 흐름이 기대와 맞나)
  python3 tests/whisker_sim.py replay     logs/imu_lab/*.csv 를 Whisker 에 그대로 흘림 (어떤 감각이 뜨나)
  python3 tests/whisker_sim.py replay burst   … IMU 가 밀렸다 한꺼번에 처리되는 경우 (0.5 s 씩 몰아서)

물리: 명령 → 궤도별 접지력 → 실제 이동. 거리계 = 바퀴 (헛돌면 거짓말), 방향 = 자이로 (정직).
"""
import csv, glob, json, math, os, sys, types
import numpy as np
T = [1000.0]
import time as _time
_time.time = lambda: T[0]
def mod(n, **kw):
    m = types.ModuleType(n); m.__dict__.update(kw); sys.modules[n] = m; return m
class Node:
    def __init__(s, n): s.logs = []
    def declare_parameter(s, n, v): return types.SimpleNamespace(value=v)
    def create_subscription(s, *a, **k): pass
    def create_publisher(s, *a, **k): return types.SimpleNamespace(publish=lambda m: None)
    def create_timer(s, *a): pass
    def get_logger(s):
        L = s.logs; f = lambda x: L.append(x)
        return types.SimpleNamespace(info=f, warn=f, error=f)
    def get_clock(s): return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))
class Msg:
    def __init__(s, *a, **k): s.__dict__.update(k)
mod("rclpy"); mod("rclpy.node", Node=Node); mod("rclpy.qos", qos_profile_sensor_data=None)
for n, it in {"geometry_msgs.msg": ["PoseWithCovarianceStamped", "TwistStamped"], "nav_msgs.msg": ["Odometry"],
              "sensor_msgs.msg": ["Imu", "PointCloud2", "PointField", "LaserScan"], "std_msgs.msg": ["Empty", "Float32MultiArray", "String"]}.items():
    mod(n, **{i: Msg for i in it})
for n in ("geometry_msgs", "nav_msgs", "sensor_msgs", "std_msgs"): mod(n)
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))
import whisker
np.random.seed(1)
B, ROOM = 0.085, (-1.5, 2.5, -1.5, 1.5)
def stamp(t): return types.SimpleNamespace(sec=int(t), nanosec=int((t % 1) * 1e9))

class Sim:
    def __init__(s):
        s.W = W = whisker.Whisker(); W.laser_yaw = 0.0
        W.publish_cloud = lambda: None
        W.map_pose = lambda: ("map", W.odom[0], W.odom[1], W.odom[2]) if W.odom else None
        s.ev, s.states, s.trace = [], [], []
        W.record = lambda ev: (s.ev.append(dict(ev)), W.logs.append("SENSE " + json.dumps(ev, ensure_ascii=False)))
        s.wcmd, s.wt = (0.0, 0.0), -9
        def send(x, z): s.wcmd, s.wt = (x, z), T[0]
        W.send = send
        s.true = [0.0, 0.0, 0.0]; s.odo = [0.0, 0.0, 0.0]; s.n = 0
        s.run(40)
        s.ev.clear(); s.states.clear(); W.logs.clear()
    def scan(s):
        x, y, th = s.true
        a = np.linspace(-math.pi, math.pi, 450, endpoint=False)
        d = np.full(450, np.inf); ca, sa = np.cos(a + th), np.sin(a + th)
        x0, x1, y0, y1 = ROOM
        with np.errstate(divide="ignore", invalid="ignore"):
            for t in ((x1 - x) / ca, (x0 - x) / ca, (y1 - y) / sa, (y0 - y) / sa):
                d = np.where((t > 0) & (t < d), t, d)
        s.W.on_scan(types.SimpleNamespace(ranges=list(d + np.random.normal(0, 0.01, 450)), angle_min=-math.pi,
                                          angle_increment=2 * math.pi / 450, range_min=0.05))
    def step(s, nav=(0.0, 0.0), trac=(1.0, 1.0), ax=0.0, ay=0.0, gx=0.0, gy=0.0, block=False, push=0.0, gap=None, floor_z=0.0, off=False):
        dt = 0.05; T[0] += dt; s.n += 1
        mine = T[0] - s.wt < 0.3
        cx, cz = s.wcmd if mine else nav
        ncx, ncz = cx, cz                                   # Nav2(또는 Whisker) 명령 = /motor_out 에 실리는 값
        if floor_z and 0 < abs(cz) < floor_z: cz = math.copysign(floor_z, cz)   # 브리지 바닥선: 실제로는 더 세게 돌림
        vl, vr = cx - cz * B, cx + cz * B
        block = block or off                                # 모터 꺼짐 = 안 움직임
        tl, tr = (0.0, 0.0) if block else trac
        v, w = (tl * vl + tr * vr) / 2, (tr * vr - tl * vl) / (2 * B) + push
        x, y, th = s.true
        nx = x + v * math.cos(th) * dt
        if gap and (nx > gap[1] or nx < gap[0]):
            nx, v, block = min(max(nx, gap[0]), gap[1]), 0.0, True
        s.true = [nx, y + v * math.sin(th) * dt, th + w * dt]
        ov = 0.0 if block else (vl + vr) / 2
        s.odo = [s.odo[0] + ov * math.cos(s.true[2]) * dt, s.odo[1] + ov * math.sin(s.true[2]) * dt, s.true[2]]
        pwm = 0 if off or (abs(cx) < 1e-3 and abs(cz) < 1e-3) else 30
        dp = 0 if pwm == 0 else 60 * cz * B / max(abs(cx) + abs(cz) * B, 1e-3)   # 회전만큼 좌우 PWM 차이
        s.W.on_motor(types.SimpleNamespace(data=[pwm - dp / 2, pwm + dp / 2, ncx, ncz, 1.0 if mine else 0.0]))
        q = types.SimpleNamespace(x=0, y=0, z=math.sin(s.odo[2] / 2), w=math.cos(s.odo[2] / 2))
        s.W.on_odom(types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(T[0])), pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(position=types.SimpleNamespace(x=s.odo[0], y=s.odo[1]), orientation=q))))
        s.W.on_imu(types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(T[0])),
                   linear_acceleration=types.SimpleNamespace(x=ax * 9.81, y=ay * 9.81, z=9.81),
                   angular_velocity=types.SimpleNamespace(x=math.radians(gx), y=math.radians(gy), z=w)))
        if s.n % 2 == 0: s.scan()
        s.W.tick()
        if not s.states or s.states[-1] != s.W.state: s.states.append(s.W.state)
        s.trace.append((T[0], *s.true, cx, cz, s.W.state))
    def run(s, n, **kw):
        for _ in range(n): s.step(**kw)
    def flow(s): return " → ".join(s.states)
    def kinds(s): return [e["ev"] for e in s.ev if e["ev"] not in ("touch",)]

def check(title, s, want_flow=None, want_ev=None, no_ev=None):
    ok = True
    if want_flow and s.flow() != want_flow: ok = False
    if want_ev and not all(k in s.kinds() for k in want_ev): ok = False
    if no_ev and any(k in s.kinds() for k in no_ev): ok = False
    print(f"{'✓' if ok else '✗'} {title:46s} 흐름: {s.flow():40s} 감지: {','.join(s.kinds()) or '-'}")
    if not ok:
        for l in s.W.logs:
            if "WHISKER" in l or "SENSE" in l: print("      ", l[:140])
    return ok

def scenes():
    R = []
    s = Sim()
    for i in range(8): s.step(nav=(0.01 * i, 0), ax=0.17 if i in (3, 4) else 0, gy=6 if i == 4 else 0)
    s.run(60, nav=(0.08, 0)); s.run(20)
    R.append(check("1 출발 흔들림 + 주행 → 아무 일 없음", s, "drive", no_ev=["bump", "lift", "stuck", "pushed", "turn_blocked"]))
    s = Sim(); s.run(40, nav=(0.08, -0.23)); s.step(nav=(0.08, -0.23), ax=0.039, ay=-0.046)
    for i in range(4): s.step(nav=(0.08, -0.23), ax=0.02 * (-1) ** i, ay=0.03 * (-1) ** i)
    s.run(30, nav=(0.08, -0.23))
    R.append(check("2 곡선 고스트 → ghost 만", s, "drive", no_ev=["bump", "lift", "stuck", "pushed", "turn_blocked"]))
    s = Sim(); s.run(40, nav=(0.08, 0)); s.step(nav=(0.08, 0), ax=0.09, block=True)
    s.run(10, nav=(0.08, 0), block=True); s.run(120)
    R.append(check("3 상자 정면 → 멈춤 → 되감기 → 돌려줌", s, "drive → stop → back → drive", ["bump"]))
    s = Sim(); s.run(40, nav=(0.08, 0)); s.run(8, nav=(0.08, 0), gy=8.0); s.run(120)
    R.append(check("4 발 밟기 (들림) → 되감기", s, "drive → back → drive", ["lift"]))
    s = Sim(); s.run(40, nav=(0.08, 0))
    for i in range(4): s.step(nav=(0.08 - 0.02 * i, 0))
    s.step(ax=-0.08); s.step(ax=0.04); s.run(20)
    R.append(check("5 굴러가다 서기 → 아무 일 없음", s, "drive", no_ev=["bump", "lift", "stuck"]))
    s = Sim(); s.run(70, nav=(0.03, 0.15)); s.run(40, nav=(0.03, 0.15), trac=(0.05, 0.0)); s.run(200, trac=(1.0, 0.3))
    R.append(check("6 카펫 한쪽 헛돎 → 안 움직임 → 되감기", s, None, ["stuck"]))
    s = Sim(); s.run(70, nav=(0.05, 0), block=True); s.run(20)
    R.append(check("8 출발하자마자 막힘 → 빼지 않음", s, "drive", ["stuck"]))
    s = Sim()
    for i in range(80): s.step(nav=(0.0, 0.2 if (i // 4) % 2 else -0.2))
    R.append(check("9 제자리 좌우 떨림 → 아무 일 없음", s, "drive", no_ev=["stuck", "pushed", "turn_blocked"]))
    s = Sim(); s.run(120, nav=(0.03, 0))
    R.append(check("10 느린 톡톡 주행 (실제로 감) → 아무 일 없음", s, "drive", no_ev=["stuck"]))
    s = Sim(); s.run(30, nav=(0.05, 0)); s.run(10, nav=(0.05, 0), push=math.radians(90)); s.run(40, nav=(0.05, 0))
    R.append(check("13 주행 중 옆에서 밀기 → pushed, 1초 정지", s, "drive → pause → drive", ["pushed"], ["turn_blocked"]))
    s = Sim(); s.run(60, nav=(0.0, 0.25), trac=(0.5, 0.5))
    R.append(check("14 제자리 회전 절반만 돎 (미끄러짐) → 아무 일 없음", s, "drive", no_ev=["pushed", "turn_blocked"]))
    s = Sim(); s.run(40, nav=(0.05, 0)); x0 = s.true[0]; s.run(1400, nav=(0.05, 0), gap=(x0 - 0.03, x0 + 0.04))
    R.append(check("15 앞뒤 틈에 갇힘 → 구조 요청", s, None, ["stuck"]) and "help" in s.states)
    s = Sim(); s.run(40, nav=(0.05, 0.3)); s.run(40, nav=(0.05, 0.3), block=True); s.run(80)
    R.append(check("16 곡선 중 걸림 → turn_blocked → 되감기", s, None, ["turn_blocked"], ["pushed"]) and "back" in s.states)
    s = Sim(); s.run(30, nav=(0.0, 0.3), block=True); s.run(80)
    R.append(check("17 제자리 회전 막힘 → turn_blocked → 15° 되돌림, 흉터만 (장애물 아님)", s, None, ["turn_blocked"], ["pushed"]) and "back" in s.states
             and len(s.W.scars) >= 1 and not any(e.get("marked") for e in s.ev if e["ev"] == "turn_blocked"))
    s = Sim(); s.run(80, nav=(-0.015, 0.07), floor_z=0.33)
    R.append(check("18 Nav2 는 살짝(z 0.07), 브리지는 바닥선으로 세게 돎 → 아무 일 없음", s, "drive", no_ev=["pushed", "turn_blocked"]))
    s = Sim(); s.run(21, nav=(-0.05, 0.05), block=True); s.run(20, nav=(-0.05, 0.05), off=True); s.run(20)
    R.append(check("19 후진: 약한 힘 + 브리지 톡톡 꺼짐 0.5 s → 아무 일 없음 (11-08 3시 고스트)", s, "drive", no_ev=["stuck"]))
    print(f"\n{sum(R)}/{len(R)} 통과")

def replay(burst=False):
    tot = {}
    for f in sorted(glob.glob(os.path.join(REPO, "logs", "imu_lab", "*.csv"))):
        R = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(f))]
        W = whisker.Whisker(); W.react = False; W.publish_cloud = lambda: None; W.map_pose = lambda: ("map", 0, 0, 0)
        evs = []; W.record = lambda ev: evs.append(dict(ev, t=T[0]))
        x = yaw = 0.0; pt = None
        bz = sum(r["gz"] for r in R[:8]) / 8               # 브리지처럼: 방향 = 자이로 적분 (출발 전 치우침 뺌)
        for r in R:
            T[0] = int(r["t"] / 0.5) * 0.5 if burst else r["t"]
            dt = 0.05 if pt is None else r["t"] - pt; pt = r["t"]; x += r["wheel_v"] * dt
            yaw += math.radians(r["gz"] - bz) * dt
            on = abs(r["cmd_x"]) > 1e-3 or abs(r["cmd_z"]) > 1e-3
            W.on_motor(types.SimpleNamespace(data=[30 if on else 0, 30 if on else 0, r["cmd_x"], r["cmd_z"], 0]))
            W.on_odom(types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(r["t"])), pose=types.SimpleNamespace(
                pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=0.0),
                                          orientation=types.SimpleNamespace(x=0, y=0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))))
            W.on_imu(types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(r["t"])),
                     linear_acceleration=types.SimpleNamespace(x=r["ax"] * 9.81, y=r["ay"] * 9.81, z=r["az"] * 9.81),
                     angular_velocity=types.SimpleNamespace(x=math.radians(r["gx"]), y=math.radians(r["gy"]), z=math.radians(r["gz"]))))
        t0 = R[0]["t"]
        shown = [f'{e["ev"]}{"(" + e["was"] + ")" if "was" in e else ""}@{e["t"] - t0:.1f}' for e in evs if e["ev"] not in ("ghost",)]
        for e in evs: tot[e["ev"]] = tot.get(e["ev"], 0) + 1
        print(f"{os.path.basename(f)[:-4]:28s} {' '.join(shown) or '-'}")
    print("합계:", tot)

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "replay":
        replay(burst="burst" in sys.argv)
    else:
        scenes()
