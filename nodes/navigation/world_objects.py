"""world_objects — 네리가 '알고 있는' 객체들 (11-27, Jasper 와). ROS 없음 → tests/world_objects_sim.py.

  왜 (Jasper 11-27): 지금은 의자 모양을 그려 놓고 끝 — 움직이든 말든 모름 ("본 척").
    LiDAR + 비전으로 한 번 '의자' 라고 알았으면, LiDAR 가 매 스캔 그 지지점을 붙잡고 있어야 함.
    누가 옮기지 않는 한 그 자리에 '존재해야만' 하고, 움직이면 바로 앎. 가려진 사이에 옮긴 건 새 객체로 (근거 없는 재식별 안 함).
    모든 객체에 같은 알고리즘 (가구 추상화: 가운데 기둥 = 닻 1 개 / 다리형 = 닻 여러 개). 클래스마다 다른 건 설정값뿐.
  실측 (11-27 chair_push, 445 장 / 45 s, 로봇 정지):
    기둥 = 점 3~6 개, 지름 3~4 cm. 가만히 있어도 가끔 한 장씩 사라짐 (연속 최대 1 장) → 놓침은 hold_s 참음.
    천천히 밀기 1~3 cm/장, 빠르게 굴리기도 최대 5 cm/장 (10 Hz) → 문턱 gate 10 cm 로 처음부터 끝까지 같은 의자.
    사람이 앞에 서면 시선에 더 가까운 점 → '가려짐' (없음 아님).

  좌표: odom (연속). map 은 SLAM 보정으로 튐 (11-26 최대 25 cm > gate) → 이어가기·저장은 odom, 지도로 낼 때만 map.
  객체 = 이름 + 자세 (x, y, θ) + 닻 (LiDAR 로 '확인한' 지지점만 — 침대면 본 다리 2 개, 격자로 추론한 다리는 닻 아님) + 모양.
    닻·모양은 객체 기준 좌표 → 자세가 바뀌면 같이 움직이고 돎 (강체).
  매 스캔 (step):
    1. 스캔 점 (odom, 스캔 순서) → 덩어리 (gap 5 cm) → 작은 덩어리 (지름 ≤ small, 점 ≥ 2)
    2. 모든 객체의 닻을 예측 자리에 놓고, gate 안 가장 가까운 짝부터 (한 덩어리는 한 닻에만)
    3. 찾은 닻 2 개 이상 → 2D 정합 (평행이동 + 회전), 1 개 → 방향 유지·평행이동만. 맞춤 오차 > fit_err 면 가장 가까운 하나로만
    4. 못 찾은 닻: 시선에 닻보다 margin 넘게 가까운 점 → 가려짐 / 시선이 닻 너머로 감 → 안 보임 / 응답 없음 → 판단 안 함
    5. 객체: 하나라도 찾음 → 있음. 하나도 못 찾음 + 안 보임 닻 있음 → hold_s 뒤 없음 의심 (missing). 그 외 → 가려짐 (보류)
    6. 쉬는 자세에서 move_m 또는 move_deg 넘게 → moving. 그 뒤 settle_s 동안 한 자리 → moved (from → to, 거리, 회전)
  11-27 실물 (world_model 첫 주행): 천천히 민 0.34·0.27 m 는 따라감, 빠르게 굴리자 '움직이기 시작' 직후 놓침 (녹화 땐 5 cm/장이었는데 더 빨랐음)
    → 움직이는 중엔 최근 속도로 다음 자리를 예측하고 문턱을 gate_moving (25 cm) 로 (사람 추적과 같은 방식)
    → 맡기기는 key (floor_hazard 가구 하나) 로: 아는 key 면 새로 안 만듦 (놓친 뒤 옛 자리로 다시 맡겨 유령 #2 가 생겼음)
"""
import math

import numpy as np

PRESENT, MOVING, HIDDEN, MISSING = "present", "moving", "hidden", "missing"


def segments(pts, gap=0.05):
    """스캔 순서 점 N×2 → 덩어리 목록 (각각 점 배열)"""
    p = np.asarray(pts, float).reshape(-1, 2)
    if len(p) == 0:
        return []
    cut = np.where(np.hypot(*np.diff(p, axis=0).T) > gap)[0] + 1
    return np.split(p, cut)


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def fit(local, world):
    """객체 기준 점 → 관측 점 강체 (2D Kabsch). → (위치, θ, 평균 오차)"""
    lc, wc = local.mean(0), world.mean(0)
    h = (local - lc).T @ (world - wc)
    th = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    p = wc - rot(th) @ lc
    err = float(np.mean(np.hypot(*((local @ rot(th).T + p) - world).T)))
    return p, th, err


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Obj:
    def __init__(self, oid, name, anchors, shape, t):
        a = np.asarray(anchors, float).reshape(-1, 2)
        self.id, self.name = oid, name
        self.p, self.th = a.mean(0), 0.0                 # 자세 (odom): 닻 가운데, 처음 방향 0
        self.local = a - self.p                          # 닻 (객체 기준)
        self.shape = None if shape is None else np.asarray(shape, float).reshape(-1, 2) - self.p
        self.rest_p, self.rest_th = self.p.copy(), 0.0  # 마지막으로 멈춰 있던 자세
        self.state, self.seen_t, self.lost_t, self.clear_t = PRESENT, t, None, None
        self.still_from, self.still_p, self.still_th = None, self.p.copy(), 0.0
        self.history = []                                # [(t, from, to)]
        self.v = np.zeros(2)                             # 움직이는 중 속도 (m/s, odom) — 다음 자리 예측용
        self.last = {"found": len(a), "hidden": 0, "clear": 0}   # 이번 스캔 닻 상태 수 (로그용)

    def anchors(self):
        return self.local @ rot(self.th).T + self.p

    @property
    def anchor(self):
        """닻 가운데 (odom) — 닻 1 개면 그 닻"""
        return self.anchors().mean(0)

    def corners(self):
        return None if self.shape is None else self.shape @ rot(self.th).T + self.p


class WorldObjects:
    def __init__(self, gate=0.10, small=0.08, hold_s=1.0, move_m=0.10, move_deg=10.0, settle_s=1.0, margin=0.08,
                 ray_deg=2.0, fit_err=0.04, gate_moving=0.25):
        self.gate, self.small, self.hold_s, self.move_m, self.settle_s = gate, small, hold_s, move_m, settle_s
        self.gate_moving, self.keys = gate_moving, {}
        self.move_th, self.margin, self.ray, self.fit_err = math.radians(move_deg), margin, math.radians(ray_deg), fit_err
        self.objs, self.next_id = {}, 1

    def adopt(self, name, anchors, shape=None, t=0.0, key=None):
        """비전 + LiDAR 로 확인된 객체를 맡김. anchors = 확인된 지지점 (점 하나 또는 여러 개, odom).
           key (맡기는 쪽의 가구 하나) 를 이미 알면 그 객체 그대로 — 위치는 LiDAR 가 주인 (맡기는 쪽 자리는 옛 것일 수 있음).
           아니면 같은 이름 객체의 닻이 gate×2 안에 있으면 그것 (모양만 새로)"""
        a = np.asarray(anchors, float).reshape(-1, 2)
        if key is not None and self.keys.get(key) in self.objs:
            return self.objs[self.keys[key]], False
        for o in self.objs.values():
            d = np.hypot(*(o.anchors()[:, None, :] - a[None, :, :]).transpose(2, 0, 1))
            if o.name == name and float(d.min()) < self.gate * 2:
                if shape is not None:
                    o.shape = (np.asarray(shape, float).reshape(-1, 2) - o.p) @ rot(o.th)   # 객체 기준으로
                if key is not None:
                    self.keys[key] = o.id
                return o, False
        o = Obj(self.next_id, name, a, shape, t)
        self.objs[o.id] = o
        if key is not None:
            self.keys[key] = o.id
        self.next_id += 1
        return o, True

    def key_of(self, oid):
        return next((k for k, v in self.keys.items() if v == oid), None)

    def step(self, pts, sensor, t):
        """pts: 이번 스캔 점 (odom, 스캔 순서), sensor: 센서 위치 (odom). → 사건 [(종류, 객체, 정보)]"""
        pts = np.asarray(pts, float).reshape(-1, 2)
        cands = [c.mean(0) for c in segments(pts) if len(c) >= 2 and np.hypot(*(c.max(0) - c.min(0))) <= self.small]
        pred, gate = {}, {}
        for o in self.objs.values():                           # 움직이는 중이면 최근 속도로 예측 + 넓은 문턱
            moving = o.state == MOVING
            pred[o.id] = o.anchors() + (o.v * max(0.0, t - o.seen_t) if moving else 0.0)
            gate[o.id] = self.gate_moving if moving else self.gate
        pairs = sorted((float(np.hypot(*(m - pred[oid][i]))), oid, i, j)
                       for oid in pred for i in range(len(pred[oid])) for j, m in enumerate(cands)
                       if np.hypot(*(m - pred[oid][i])) <= gate[oid])
        used_a, used_c, match = set(), set(), {oid: {} for oid in pred}
        for _, oid, i, j in pairs:
            if (oid, i) in used_a or j in used_c:
                continue
            used_a.add((oid, i)), used_c.add(j)
            match[oid][i] = cands[j]
        rel = pts - np.asarray(sensor, float)
        ang, rng = np.arctan2(rel[:, 1], rel[:, 0]), np.hypot(rel[:, 0], rel[:, 1])
        events = []
        for o in self.objs.values():
            events += self.update(o, match[o.id], sensor, ang, rng, t)
        return events

    def look(self, q, sensor, ang, rng):
        """센서 → 점 q 시선: 'hidden' (앞에 뭐가 있음) / 'clear' (시선이 q 너머까지 = 거기 없음) / 'none' (응답 없음)"""
        rel = q - np.asarray(sensor, float)
        a0, r0 = math.atan2(rel[1], rel[0]), math.hypot(*rel)
        on = np.abs(wrap(ang - a0)) <= self.ray
        if np.any(rng[on] < r0 - self.margin):
            return "hidden"
        return "clear" if np.any(rng[on] > r0 + self.margin) else "none"

    def update(self, o, m, sensor, ang, rng, t):
        n = len(o.local)
        if m:
            idx = sorted(m)
            if len(idx) >= 2:
                p, th, err = fit(o.local[idx], np.array([m[i] for i in idx]))
                if err > self.fit_err:                         # 짝이 어긋남 → 가장 가까웠던 하나로만
                    i = min(idx, key=lambda k: np.hypot(*(m[k] - o.anchors()[k])))
                    p, th = m[i] - rot(o.th) @ o.local[i], o.th
            else:
                i = idx[0]
                p, th = m[i] - rot(o.th) @ o.local[i], o.th
            dt = t - o.seen_t
            if dt > 1e-3:                                      # 속도 (부드럽게) — 다음 스캔 예측용
                o.v = 0.5 * o.v + 0.5 * (np.asarray(p, float) - o.p) / dt
            o.p, o.th = np.asarray(p, float), float(th)
            pred = o.anchors()
            looks = [self.look(pred[k], sensor, ang, rng) for k in range(n) if k not in m]
            o.last = {"found": len(m), "hidden": looks.count("hidden"), "clear": looks.count("clear")}
            return self.seen(o, t)
        looks = [self.look(q, sensor, ang, rng) for q in o.anchors()]
        o.last = {"found": 0, "hidden": looks.count("hidden"), "clear": looks.count("clear")}
        o.lost_t = o.lost_t or t
        if "clear" in looks:                                   # 보여야 할 닻이 하나도 없고, 그 자리를 시선이 지나감
            o.clear_t = o.clear_t or t
            if o.state != MISSING and t - o.clear_t >= self.hold_s:
                o.state = MISSING
                return [("missing", o, {"since": round(t - o.clear_t, 2)})]
            return []
        o.clear_t = None
        if "hidden" in looks and o.state not in (HIDDEN, MISSING):
            o.state = HIDDEN
            return [("hidden", o, {})]
        return []

    def seen(self, o, t):
        ev = []
        if o.state in (HIDDEN, MISSING):
            ev.append(("found", o, {"after": round(t - (o.lost_t or t), 2)}))
            o.state = PRESENT
        o.seen_t, o.lost_t, o.clear_t = t, None, None
        far = float(np.hypot(*(o.p - o.rest_p)))
        turn = abs(wrap(o.th - o.rest_th))
        if o.state != MOVING and (far > self.move_m or turn > self.move_th):
            o.state, o.still_from, o.still_p, o.still_th = MOVING, t, o.p.copy(), o.th
            ev.append(("moving", o, {"from": o.rest_p.round(3).tolist()}))
        return ev + self.settle(o, t)

    def settle(self, o, t):
        """움직이던 객체가 settle_s 동안 한 자세 (move_m/2, move_deg/2 안) → moved 사건, 쉬는 자세 갱신"""
        if o.state != MOVING:
            return []
        if np.hypot(*(o.p - o.still_p)) > self.move_m / 2 or abs(wrap(o.th - o.still_th)) > self.move_th / 2:
            o.still_from, o.still_p, o.still_th = t, o.p.copy(), o.th
            return []
        if t - o.still_from < self.settle_s:
            return []
        frm = o.rest_p.copy()
        turn = math.degrees(wrap(o.th - o.rest_th))
        o.history.append((t, frm.round(3).tolist(), o.p.round(3).tolist(), round(turn, 1)))
        o.rest_p, o.rest_th, o.state, o.v = o.p.copy(), o.th, PRESENT, np.zeros(2)
        return [("moved", o, {"from": frm.round(3).tolist(), "to": o.p.round(3).tolist(),
                              "dist": round(float(np.hypot(*(o.p - frm))), 3), "turn": round(turn, 1)})]

    def drop(self, oid):
        self.objs.pop(oid, None)
        for k in [k for k, v in self.keys.items() if v == oid]:
            del self.keys[k]
