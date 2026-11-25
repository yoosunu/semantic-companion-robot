"""furniture — 밑이 뜬 가구 (침대·식탁) 영역 추론 (11-25 v2, Jasper 와 정함). ROS 없음 → 시뮬로 시험.

  왜: 침대 프레임이 LiDAR (17.5 cm) 보다 높아 다리만 보임 → 지도에선 침대 밑이 빈 공간 → 팬틸트 머리가 부딪힘.
  11-24 bed_snap 실측 (bed1): YOLO26-seg 마스크가 다리 끝을 바닥까지 잡음 → 마스크 아래점 ↔ LiDAR 다리 1~7 cm.
    떠 있는 프레임 아랫선은 2~19 m 로 멀리 찍힘 → 영역에 안 씀.

  철학 (Jasper 11-25): 한 면만 보면 넉넉하게 가정 (밑으로 못 들어가게), 다른 면을 보면 줄여서 확정. 탐색하다 보면 두 면은 봄.
    증거 (다리·마스크 끝) 가 있을 때만 줄임, "안 보인다" 로는 안 줄임. ToF 는 나중에 마지막 보완.
    가구 크기를 외워 두지 않음 (우리 침대 실측은 시험 정답으로만).

  규칙
    1. 다리 확정: 확신한 가구 마스크 아래점 (바닥에 내린 점) 에서 match (10 cm) 안의 LiDAR 작은 덩어리 = 그 가구 다리.
       odom 에 쌓음 (merge 8 cm 안이면 같은 다리).
    2. 앞줄: 처음 다리 2 개를 같이 본 장면에서 로봇에 가까운 두 다리 → 축 u (긴 변), 앞줄 선 = 확정. 깊이 방향 = 로봇 반대쪽.
       다리마다 (s = 축 따라, d = 앞줄에서 깊이).
    3. 양 끝 (s): 앞에서 본 장면의 마스크 방향 끝을 앞줄 선에 댐 → 비전 끝.
       - 비전 끝이 끝 다리에서 overhang (간격의 30 %) 안 → 끝 = 둘 중 바깥 + pad
         (프레임은 모서리 다리보다 튀어나옴 — 11-25 bed1 실물 오른쪽 11 cm. 10 cm 고정이면 오버행을 가림으로 봄 → 1 m 더 막음)
       - 더 나가 있음 = 가려진 다리 (11-24: 서랍장) → 다리는 같은 간격 (격자) 가정 → max(비전 끝, 끝 다리 + 간격 + 다리 두께) + pad
         (다리 두께 = LiDAR 가 잰 덩어리 지름 중 최대. 숨은 다리 자리는 중심이라 두께만큼 더 — 11-25 sim: pad 만으론 1.5 cm 여유)
       - 그쪽 비전 없음 → 끝 다리 + 간격 + 두께 + pad (넉넉하게)
       - 카메라가 그쪽 끝 다리보다 바깥 = 옆면이 보임 (11-25 bed2) → 그쪽 마스크 끝은 옆면 먼 모서리 → 앞면 끝 = 다리 + pad,
         그 마스크 끝은 옆면 선에 대서 깊이 증거로 (비스듬히 한 장 = 앞면 + 옆면). 전엔 앞줄 선에 대서 "31 cm 더 감 → 가려진 다리"
         → 양쪽 늘어 상한에서 반씩 잘림 → 반대쪽 숨은 다리가 빠짐 (위험 쪽 오류)
       - 전체가 max_len (2.2 m, 큰 가구 기준) 넘으면 가려진 쪽부터 자름
    4. 깊이 (d):
       - 확정: 옆에서 본 장면 (카메라가 앞줄 선 뒤쪽 = 옆, 또는 비스듬히 끝 바깥) 의 마스크 방향 끝을 옆면 선에 댐 + 깊은 줄 다리
         → 가장 깊은 것 + pad. 옆면 마스크 깊이가 row_gap (50 cm) 보다 작으면 버림 (앞 모서리를 옆면 끝으로 착각 → 깊이 9 cm 사고 방지)
       - 벽이 있으면 깊이는 벽을 넘지 않음 (확정값이 더 멀면 벽에서 자름)
       - 가정 (확정 없을 때): 앞줄 여러 점에서 깊이 방향으로 LiDAR 벽에 닿을 때까지 (이 가구 다리 점은 뺌) — 가장 먼 것,
         (보인 샘플 중 최대) 벽 없으면 max_len. 벽에 붙은 가구면 벽까지라 손해 없음 (Jasper). 벽과 떨어져 있으면 옆에서 볼 때 줄어듦.
    5. → 사각형 4 점 (odom) + 근거 (why). 깊이는 "가정"/"확정" 표시.
    교차 각도 (11-25): 비전 방향은 믿고, 거리는 광선이 선을 min_cross (30°) 넘게 가로지를 때만 믿음.
       선 위치 오차 e 는 만나는 점을 e / sin(각도) 만큼 밀어냄 (GPS DOP·스테레오 시차와 같은 이야기).
       11-25 bed2: 옆면 먼 모서리 방향 −8° = 정답 −7.5° (방향 맞음) 인데 옆면 선과 8° 로 스쳐 깊이 1.87 (실제 1.085).
       → 스치는 앞면 끝은 "비전 없음 (스침)" 으로 (넉넉하게), 스치는 옆면 깊이는 버림 (벽까지 가정).
    pad 는 근거에 따라 (11-25, Jasper: 튀어나온 만큼 통로가 좁아질 걱정):
       LiDAR 로 잰 선 (앞줄, 확정된 깊은 다리) = 0 (다리 중심 + 두께 반) / 비전으로 잰 끝 (마스크) = pad / 추론 (숨은 다리) = pad + 두께.
       벽까지 가정은 벽 자체가 장애물이라 pad 없음. 부풀리기 (whisker_inflation 12 cm) 는 Nav2 가 따로 함.
"""
import math

import numpy as np

FURNITURE = ("bed", "dining table", "couch", "chair")   # 밑이 뜬 가구 (실물 시험: 침대, 의자)
ROUND = ("chair",)       # 11-25 회전의자: 기둥 하나 + 별 모양 다리·바퀴 (LiDAR 아래) → 원


def leg_clusters(pts, gap=0.05, max_d=0.08, clear=0.15):
    """스캔 순서 그대로의 점 N×2 → 다리 후보 [(cx, cy, 지름)]: gap 으로 끊은 덩어리 중 지름 ≤ max_d, 다른 점이 clear 안에 없음
       (각도로 다시 정렬하지 않음: 라이다가 로봇 중심보다 7.8 cm 앞이라 다리 가장자리와 뒷벽 점이 섞임, 11-24)"""
    p = np.asarray(pts, float).reshape(-1, 2)
    if len(p) == 0:
        return []
    cut = np.where(np.hypot(*np.diff(p, axis=0).T) > gap)[0] + 1
    out = []
    for c in np.split(p, cut):
        d = float(np.hypot(*(c.max(0) - c.min(0))))
        if d > max_d or len(c) < 2:
            continue
        cen = c.mean(0)
        dist = np.hypot(*(p - cen).T)
        others = dist[dist > d / 2 + 0.01]
        if len(others) and others.min() < clear:
            continue
        out.append((float(cen[0]), float(cen[1]), d))
    return out


def ray_hit(origin, direction, pts, width=0.03, skip=(), skip_r=0.08, t_min=0.05):
    """origin 에서 direction (단위 벡터) 광선에 width 안으로 붙은 점 중 가장 가까운 거리 (없으면 inf). skip 점 skip_r 안의 점은 뺌"""
    p = np.asarray(pts, float).reshape(-1, 2)
    for q in skip:
        p = p[np.hypot(*(p - q).T) > skip_r]
    if len(p) == 0:
        return math.inf
    p = p - origin
    t = p @ direction
    side = np.abs(p @ np.array([-direction[1], direction[0]]))
    ok = (t > t_min) & (side < width)
    return float(t[ok].min()) if ok.any() else math.inf


def ray_to_line(cam, ang, p0, n):
    """cam 에서 ang 방향 광선이 점 p0, 법선 n 인 직선과 만나는 점 (뒤쪽·평행이면 None)"""
    d = np.array([math.cos(ang), math.sin(ang)])
    den = d @ n
    if abs(den) < 1e-6:
        return None
    t = ((p0 - cam) @ n) / den
    return cam + t * d if t > 0 else None


class Furniture:
    def __init__(self, name="bed", match=0.10, merge=0.15, row_gap=0.5, spacing=1.0, max_len=2.2, pad=0.05, overhang=0.3,
                 min_cross=30.0, min_spacing=0.30):
        # merge 0.08 → 0.15 (11-25 20:00 실기: 초기화로 90° 씩 돌며 같은 다리를 여러 각도에서 → 각진 다리 다른 면·SLAM 어긋남으로
        #   10 cm 떨어진 다리 둘로 → 간격 0.10 으로 계산됨). min_spacing: 이보다 가까운 쌍은 간격 계산에서 뺌
        self.name, self.match, self.merge, self.row_gap, self.overhang = name, match, merge, row_gap, overhang
        self.spacing, self.max_len, self.pad, self.min_cross, self.min_spacing = spacing, max_len, pad, min_cross, min_spacing
        self.legs = []          # [[x, y, 본 횟수, 지름]] odom
        self.views = []         # [(cam_xy, 왼쪽 각, 오른쪽 각 (rad, odom), 그 순간 스캔 odom N×2)]
        self.frame = None       # (a, u, n): 앞줄 한 점, 축, 깊이 방향 (로봇 반대쪽)

    def add_view(self, cam_xy, bearing, mask_floor, scan, robot_xy):
        """한 장: cam_xy 렌즈 위치, bearing (왼쪽 끝, 오른쪽 끝) rad (odom, 왼쪽 +; 마스크가 화면 끝에 잘린 쪽은 None —
           잘린 끝은 가구 끝이 아니라 화면 끝, 11-25 bed2 에서 그걸 끝으로 믿어 숨은 다리가 빠짐), mask_floor 마스크 아래점 바닥 (odom, nan 은 뺌),
           scan 그 순간 스캔 점 (odom, 스캔 순서), robot_xy 로봇 위치. → 이번에 다리로 확정한 것 [(x, y)]"""
        mf = np.asarray(mask_floor, float).reshape(-1, 2)
        mf = mf[np.isfinite(mf).all(1)]
        found = []
        for cx, cy, dia in leg_clusters(scan):
            if len(mf) and np.hypot(*(mf - [cx, cy]).T).min() <= self.match:
                found.append((cx, cy))
                for L in self.legs:
                    if math.hypot(L[0] - cx, L[1] - cy) < self.merge:
                        L[0] += (cx - L[0]) / (L[2] + 1); L[1] += (cy - L[1]) / (L[2] + 1); L[2] += 1; L[3] = max(L[3], dia)
                        break
                else:
                    self.legs.append([cx, cy, 1, dia])
        self.views.append((np.asarray(cam_xy, float), bearing[0], bearing[1], np.asarray(scan, float).reshape(-1, 2)))
        if self.frame is None and len(found) >= 2:                    # 앞줄 = 처음 두 다리를 같이 본 장면의 가까운 두 다리
            r = np.asarray(robot_xy, float)
            F = np.array(found)
            o = np.argsort(np.hypot(*(F - r).T))
            a, b = F[o[0]], F[o[1]]
            u = (b - a) / max(np.linalg.norm(b - a), 1e-6)
            n = np.array([-u[1], u[0]])
            if (r - a) @ n > 0:
                n = -n
            self.frame = (a, u, n)
        return found

    def shape(self):
        """→ dict (corners 4×2 odom, 근거) 또는 None (앞줄 못 정함)"""
        if self.frame is None:
            return None
        a, u, n = self.frame
        L = np.array([l[:2] for l in self.legs])
        s, d = (L - a) @ u, (L - a) @ n
        front = np.abs(d) < self.row_gap / 2
        if front.sum() >= 2:                                         # 앞줄 다리로 선 다시 맞춤 (깊이 평균 0 으로)
            a = a + n * float(np.mean(d[front]))
            d = (L - a) @ n
        sf = np.sort(s[front])
        gaps = np.diff(sf)
        gaps = gaps[gaps >= self.min_spacing]                           # 30 cm 안 쌍 = 같은 다리를 두 번 본 것
        spacing = float(np.median(gaps)) if len(gaps) else self.spacing
        leg_w = max(l[3] for l in self.legs)                         # 다리 두께 (LiDAR 덩어리 지름 최대)
        s_lo, s_hi = float(min(s)), float(max(s))                    # 옆에서 본 깊은 다리도 포함
        # 3. 양 끝: 앞에서 본 장면 (카메라 깊이 < 0, 앞줄 범위 근처) 의 방향 끝 → 앞줄 선
        vis = {-1: [], +1: []}
        side_views, side_rays, side_seen = [], [], set()
        grazed = {-1: 0, +1: 0, "side": 0}
        cross = lambda ang, normal: math.degrees(math.asin(min(1.0, abs(np.array([math.cos(ang), math.sin(ang)]) @ normal))))
        for cam, al, ar, scan in self.views:
            sc, dc = float((cam - a) @ u), float((cam - a) @ n)
            if dc < 0:                                               # 앞줄 선 앞에 있으면 앞면을 봄 (11-25 bed2: 비스듬히 끝 바깥에서 봐도 앞면.
                #   전엔 카메라가 다리 범위 ±0.3 m 밖이면 옆으로 쳐서 비전 끝을 다 버림 → 양쪽 "비전 없음" → 1 m 씩 늘어 상한)
                for ang in (al, ar):
                    if ang is None:
                        continue
                    q = ray_to_line(cam, ang, a, n)
                    if q is None:
                        continue
                    sv = float((q - a) @ u)
                    sign = +1 if sv > (sf[0] + sf[-1]) / 2 else -1
                    if (sc - (s_hi if sign > 0 else s_lo)) * sign > 0:   # 카메라가 그쪽 끝 다리보다 바깥 → 옆면 보임
                        side_seen.add(sign)
                        side_rays.append((cam, ang, sign))
                    elif cross(ang, n) < self.min_cross:                # 앞줄 선을 스침 → 거리 못 믿음
                        grazed[sign] += 1
                    else:
                        vis[sign].append(sv)
            else:
                side_views.append((cam, al, ar, sc))
        ends, why = {}, {}
        for sign, s_leg in ((-1, s_lo), (+1, s_hi)):
            k = "lo" if sign < 0 else "hi"
            if sign in side_seen and not vis[sign]:
                ends[k], why[k] = s_leg + sign * (leg_w / 2 + self.pad), "옆면 보임 → 끝 = 다리 + pad"
                continue
            if not vis[sign]:
                tag = f"비전 없음 (스침 {grazed[sign]})" if grazed[sign] else "비전 없음"
                ends[k], why[k] = s_leg + sign * (spacing + leg_w + self.pad), tag + " → 다리 + 간격 + 두께 + pad"
                continue
            sv = max(v * sign for v in vis[sign]) * sign
            beyond = (sv - s_leg) * sign
            if beyond <= max(self.match, self.overhang * spacing):
                ends[k], why[k] = max(sv * sign, s_leg * sign) * sign + sign * self.pad, f"마스크 끝 = 다리 ({beyond * 100:+.0f} cm)"
            else:
                ends[k] = max(sv * sign, (s_leg + sign * (spacing + leg_w)) * sign) * sign + sign * self.pad
                why[k] = f"마스크가 다리보다 {beyond * 100:.0f} cm 더 감 → 가려진 다리 (간격 {spacing:.2f})"
        lo, hi = ends["lo"], ends["hi"]
        if hi - lo > self.max_len:                                   # 가려진 쪽부터 자름
            over = hi - lo - self.max_len
            occ = {k: ("가려진" in why[k] or "비전 없음" in why[k]) for k in ("lo", "hi")}
            if occ["hi"] and not occ["lo"]:
                hi -= over
            elif occ["lo"] and not occ["hi"]:
                lo += over
            else:
                lo += over / 2; hi -= over / 2
            why["lo"] += " (상한)"; why["hi"] += " (상한)"
        # 4. 깊이: 확정 (옆에서 본 마스크 끝 + 깊은 다리) 아니면 가정 (벽까지)
        deep = d[d >= self.row_gap / 2]
        conf = [float(v) for v in deep]
        for cam, ang, sign in side_rays:                             # 비스듬히 본 옆면 먼 모서리
            if cross(ang, u) < self.min_cross:                       # 옆면 선 (방향 n, 법선 u) 을 스침 → 버림
                grazed["side"] += 1
                continue
            q = ray_to_line(cam, ang, a + (s_hi if sign > 0 else s_lo) * u, u)   # 옆면 선 = 끝 다리 자리 (pad 밖 선에 대면 비스듬한 광선이 일찍 만나 깊이가 짧아짐)
            if q is not None:
                dv = float((q - a) @ n)
                if self.row_gap <= dv <= self.max_len:
                    conf.append(dv)
        for cam, al, ar, sc in side_views:
            edge = a + (hi if sc > (lo + hi) / 2 else lo) * u            # 카메라 쪽 옆면 선 (u 가 법선)
            for ang in (al, ar):
                if ang is None:
                    continue
                if cross(ang, u) < self.min_cross:
                    grazed["side"] += 1
                    continue
                q = ray_to_line(cam, ang, edge, u)
                if q is not None:
                    dv = float((q - a) @ n)
                    if self.row_gap <= dv <= self.max_len:
                        conf.append(dv)
        allscan = np.vstack([v[3] for v in self.views])
        hits = [ray_hit(a + sv * u, n, allscan, skip=L, t_min=self.row_gap / 2) for sv in np.linspace(lo + self.pad, hi - self.pad, 9)]
        seen = [h for h in hits if h < self.max_len]
        wall = max(seen) if seen else self.max_len                  # 벽이 한 군데라도 보이면 그 벽까지 (가려져 안 보인 샘플 하나로 상한까지 가지 않게, sim 10)
        if conf:
            cand = [float(v) + leg_w / 2 for v in deep] + [v + self.pad for v in conf[len(deep):]]   # 다리 = LiDAR (pad 0), 옆면 마스크 = 비전 (pad)
            depth, why_d = max(cand), f"확정 (옆면 마스크·깊은 다리 {len(deep)} 개)"
            if wall < self.max_len and depth > wall:
                depth, why_d = wall, why_d + f" → 벽에서 자름 ({max(cand):.2f} > 벽 {wall:.2f})"
        else:
            depth = wall
            why_d = "가정 (벽까지)" if depth < self.max_len else "가정 (벽 없음 → 상한)"
            if grazed["side"]:
                why_d += f" (옆면 스침 {grazed['side']} 버림)"
        base = a - n * (leg_w / 2)                                   # 앞줄 = LiDAR 다리 선 → pad 없이 다리 두께 반만
        corners = np.array([base + lo * u, base + hi * u, a + hi * u + n * depth, a + lo * u + n * depth])
        return {"corners": corners, "axis_u": u, "origin": a, "normal": n, "lo": lo, "hi": hi, "depth": depth,
                "spacing": spacing, "confirmed": bool(conf), "why": {"lo": why["lo"], "hi": why["hi"], "depth": why_d}}


def fill(poly, step=0.03):
    """볼록 다각형 안을 step 격자로 채운 점 + 꼭짓점 (costmap 용, hazards.cloud_points 와 같은 방식)"""
    poly = np.asarray(poly, float)
    (x0, y0), (x1, y1) = poly.min(0), poly.max(0)
    xs, ys = np.meshgrid(np.arange(x0, x1 + step, step), np.arange(y0, y1 + step, step))
    g = np.column_stack([xs.ravel(), ys.ravel()])
    sides = [(poly[(i + 1) % len(poly)] - poly[i])[0] * (g - poly[i])[:, 1] - (poly[(i + 1) % len(poly)] - poly[i])[1] * (g - poly[i])[:, 0]
             for i in range(len(poly))]
    S = np.array(sides)
    ok = (S >= -1e-9).all(0) | (S <= 1e-9).all(0)
    return np.vstack([g[ok], poly])


class FurnitureMap:
    """여러 가구 (11-25, floor_hazard 가 씀). 같은 이름 + 새 발자국이 기존 다리 assoc (0.5 m) 안이거나 기존 사각형 안 → 같은 가구, 아니면 새로.
       좌표는 부르는 쪽이 하나로 (floor_hazard = map).
       이중 문턱 (11-25 Jasper: 각도에 따라 YOLO 점수가 떨어짐 — bed2 pan +40 0.37, bed1 잘린 조각 0.23):
         새 가구는 on (0.5) 이상, 이미 있는 가구 자리면 keep_on (0.3) 이상 → 받음 (추적의 시작 엄격·유지 너그러움).
         낮춰도 다리는 LiDAR 덩어리가 마스크 발자국 10 cm 안일 때만 → 엉뚱한 걸 받아도 다리가 안 맞으면 영향 없음."""

    def __init__(self, on=0.5, keep_on=0.3, assoc=0.5, chair_kw=None, **kw):
        self.on, self.keep_on, self.assoc, self.kw, self.chair_kw = on, keep_on, assoc, kw, chair_kw or {}
        self.items = []

    def observe(self, name, conf, cam, bearing, foot, scan, robot):
        """→ (Furniture 또는 None, 이번에 확정한 다리, "new"/"same..."/"skip: 이유")"""
        if name not in FURNITURE:
            return None, [], "skip: 밑이 뜬 가구 아님"
        if conf < self.keep_on:
            return None, [], f"skip: 점수 {conf:.2f} < {self.keep_on}"
        foot = np.asarray(foot, float).reshape(-1, 2)
        foot = foot[np.isfinite(foot).all(1)]
        if not len(foot):
            return None, [], "skip: 발자국 없음"
        F, how = None, "new"
        if name in ROUND:                                               # 옮긴 의자: 보이는 범위인데 기둥이 없으면 지움
            for it in [i for i in self.items if i.name == name and i.gone(scan, cam, bearing)]:
                self.items.remove(it)
        for it in self.items:
            if it.name != name:
                continue
            near = any(np.hypot(*(foot - l[:2]).T).min() < self.assoc for l in it.legs)
            r = it.shape()
            inside = r is not None and any(_inside(r["corners"], q) for q in foot)
            if near or inside:
                F, how = it, "same"
                break
        if F is None and conf < self.on:
            return None, [], f"skip: 점수 {conf:.2f} < {self.on} (새 가구) — 기존 가구 자리면 {self.keep_on} 부터 받음"
        if F is not None and conf < self.on:
            how = f"same (낮은 점수 {conf:.2f}, 기존 가구 자리라 받음)"
        if F is None:
            F = Chair(name=name, **self.chair_kw) if name in ROUND else Furniture(name=name, **self.kw)
            self.items.append(F)
        found = F.add_view(cam, bearing, foot, scan, robot)
        if name in ROUND and how == "new" and not F.legs and F.shape() is None:   # 11-26: 기둥도 모양도 없는 새 의자는 두지 않음
            self.items.remove(F)                                        #   (안 그러면 기둥 못 찾은 의자가 볼 때마다 새로 쌓임 — 연관할 근거가 없음)
        return F, found, how

    def shapes(self):
        return [(it, it.shape()) for it in self.items]

    def cloud_points(self, step=0.03):
        pts = [fill(r["corners"], step) for _, r in self.shapes() if r is not None]
        return np.vstack(pts) if pts else np.zeros((0, 2))

    def clear(self):
        n = len(self.items)
        self.items = []
        return n


def convex(pts):
    """볼록 껍질 (반시계, monotone chain)"""
    p = sorted(set(map(tuple, np.round(np.asarray(pts, float).reshape(-1, 2), 4))))
    if len(p) < 3:
        return np.array(p, float)
    def half(seq):
        h = []
        for q in seq:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (q[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0]) <= 0:
                h.pop()
            h.append(q)
        return h
    lo, up = half(p), half(p[::-1])
    return np.array(lo[:-1] + up[:-1], float)


def in_wedge(cam, bearing, pts):
    """cam 에서 본 방향이 bearing (왼쪽 끝, 오른쪽 끝) 사이인 점 (bool). 한쪽이 None 이면 그쪽은 열어 둠"""
    p = np.asarray(pts, float).reshape(-1, 2)
    a = np.arctan2(p[:, 1] - cam[1], p[:, 0] - cam[0])
    L, R = bearing
    if L is None and R is None:
        return np.ones(len(p), bool)
    mid = (L + R) / 2 if L is not None and R is not None and abs(L - R) < math.pi else (L if R is None else R)
    d = np.arctan2(np.sin(a - mid), np.cos(a - mid))
    ok = np.ones(len(p), bool)
    if L is not None:
        ok &= d <= math.atan2(math.sin(L - mid), math.cos(L - mid)) + 1e-9
    if R is not None:
        ok &= d >= math.atan2(math.sin(R - mid), math.cos(R - mid)) - 1e-9
    return ok


def stale(old, new, tol=0.10):
    """11-26 옛 모양의 꼭짓점 중 새 모양 바깥으로 tol 넘게 떨어진 게 있나 (= costmap 에 옛 칸이 남음 → 지워야 함).
       costmap 가구 층은 clearing: false 라 새로 칠하는 건 되지만 줄어들거나 옮겨진 자리는 안 지워짐.
       new 가 None (가구가 사라짐) 이면 True. 침대처럼 1~5 cm 흔들리는 건 tol 안."""
    if old is None:
        return False
    if new is None:
        return True
    new = np.asarray(new, float)
    n = len(new)
    for p in np.asarray(old, float):
        if _inside(new, p):
            continue
        d = min(_seg_dist(p, new[i], new[(i + 1) % n]) for i in range(n))
        if d > tol:
            return True
    return False


def _seg_dist(p, a, b):
    ab = b - a
    t = 0.0 if not ab.any() else float(np.clip((p - a) @ ab / (ab @ ab), 0, 1))
    return float(np.hypot(*(p - (a + t * ab))))


def _inside(poly, p):
    n = len(poly)
    s = [(poly[(i + 1) % n] - poly[i])[0] * (p - poly[i])[1] - (poly[(i + 1) % n] - poly[i])[1] * (p - poly[i])[0] for i in range(n)]
    return all(v >= -1e-9 for v in s) or all(v <= 1e-9 for v in s)


class Chair:
    """의자. 1차 = 기둥 (아래), 2차 = 접점 + 끝 방향 (11-25 밤, 아래 add_view 의 contact 설명).
    1차: 회전의자 (11-25, Jasper: 별 모양 다리는 LiDAR 아래라 위스커가 요리조리 피하다 포기 → 원기둥처럼 그려 비켜 가면 그만).
       chair1 실측 (bed_snap): LiDAR 가 가스 기둥을 작은 덩어리 (1~3 cm) 로 5 장 모두 같은 자리, 마스크 아래점에서 8~13 cm.
       마스크 아래점 = 바퀴 (바닥, 정확) + 좌석 옆·등받이 아래 (떠 있어 멀리 찍힘 0.45~0.56) → 기둥까지 거리의 장면별 중간값 →
       그 중간값들의 중간값 = 0.34 (줄자 별 다리 지름 68 cm → 0.34). 한 장만 보면 0.22 까지 작게 나옴 → 최소 min_r.
       좌석 (~45 cm) 은 머리 (~30 cm) 보다 높음 → 막는 건 별 다리·바퀴만.
       중심 = 기둥 평균, 반지름 = clamp(장면 중간값들의 중간값, min_r, max_r) + pad. 원을 sides 각형으로."""

    def __init__(self, name="chair", match=0.15, near=0.6, min_r=0.28, max_r=0.45, pad=0.05, sides=16, merge=0.10, long=0.30, use1=True, use2=False):
        self.name, self.match, self.near, self.min_r, self.max_r, self.pad, self.sides, self.merge = name, match, near, min_r, max_r, pad, sides, merge
        self.long, self.use1, self.use2 = long, use1, use2   # 11-26 use1=False: 1차 끄고 2차만 (실물 비교 시험용)
        # 11-26 2차는 기본 끔 (Jasper 와 결정): 실물 2차만 주행 (run_1126b) 에서 중심이 장면마다 10~25 cm 흔들리고
        #   모양이 비뚤어진 사다리꼴 (폭 = 좌석·팔걸이 끝 방향). 의자는 '덤' 으로 몇 장 스쳐 보는 게 전부라
        #   짧게·적은 장면으로 정확해야 함 → 한 장으로 중심이 정확한 1차 (기둥) 만. 놓친 의자는 위스커 → 나중에 ToF.
        #   1차 실패 이유 (miss) 는 계속 로그로 남김.
        self.cols = []           # [[x, y, 본 횟수]] 기둥
        self.radii = []          # 장면별 중간값
        self.legs = []           # FurnitureMap 의 연관 판단용 (= 기둥)
        self.views = []
        self.c2 = []             # 2차: 장면마다 (접점 N×2, 끝 점 M×2, 카메라→물체 단위 방향, 벽까지 거리)
        self.miss = ""           # 1차 실패 이유 (마지막 장면) — 11-25 run_1125d 의자 6 번 보고 "다리 0 개" 원인 찾기

    def add_view(self, cam, bearing, mask_floor, scan, robot):
        mf = np.asarray(mask_floor, float).reshape(-1, 2)
        mf = mf[np.isfinite(mf).all(1)]
        if not len(mf):
            return []
        if self.use2 or not self.use1:
            self.contact(cam, bearing, mf, scan)
        cand = [(cx, cy) for cx, cy, _ in leg_clusters(scan) if np.hypot(*(mf - [cx, cy]).T).min() <= self.match]
        if len(cand) >= 2:                                              # 11-25: 다리 여럿 = 4 다리 의자 → 기둥 하나 (회전의자) 규칙이 아님
            self.miss = f"작은 덩어리 {len(cand)} 개 (기둥 하나가 아님 → 2차)"
            cand = []
        elif not cand:
            self.miss = self.why_miss(mf, scan)
        if not cand:
            if self.c2:                                                 # 2차만으로도 연관 판단 (FurnitureMap) 되게 접점 대표점
                allc = np.vstack([v["c"] for v in self.c2])
                self.legs = [[float(allc[:, 0].mean()), float(allc[:, 1].mean()), len(self.c2), 0.03]] if not self.cols else self.legs
            return []
        self.miss = ""
        mid = np.median(mf, axis=0)                                     # 발자국 가운데에 가장 가까운 덩어리 = 기둥
        cx, cy = min(cand, key=lambda c: math.hypot(c[0] - mid[0], c[1] - mid[1]))
        for c in self.cols:
            if math.hypot(c[0] - cx, c[1] - cy) < self.merge:
                c[0] += (cx - c[0]) / (c[2] + 1); c[1] += (cy - c[1]) / (c[2] + 1); c[2] += 1
                break
        else:
            self.cols.append([cx, cy, 1])
        r = np.hypot(*(mf - [cx, cy]).T)
        r = r[r < self.near]
        if len(r):
            self.radii.append(float(np.median(r)))
        self.legs = [[c[0], c[1], c[2], 0.03] for c in self.cols]
        if not self.use1 and self.c2:                                   # 1차 끔: 연관 판단도 2차 접점 대표점으로
            allc = np.vstack([v["c"] for v in self.c2])
            self.legs = [[float(allc[:, 0].mean()), float(allc[:, 1].mean()), len(self.c2), 0.03]]
        self.views.append((np.asarray(cam, float), bearing[0], bearing[1], np.asarray(scan, float).reshape(-1, 2)))
        return [(cx, cy)]

    def why_miss(self, mf, scan):
        """1차 (기둥) 실패 이유: 떨어짐 조건 없이 다시 찾아 보고, 그래도 없으면 가장 가까운 작은 덩어리까지 거리"""
        loose = leg_clusters(scan, clear=0.0)
        if not loose:
            return "작은 덩어리 없음"
        dist = [(float(np.hypot(*(mf - [cx, cy]).T).min()), cx, cy) for cx, cy, _ in loose]
        d, cx, cy = min(dist)
        if d <= self.match:
            return f"후보 ({cx:+.2f},{cy:+.2f}) 가 있지만 15 cm 안에 다른 점 (붙어 있음)"
        return f"가장 가까운 작은 덩어리 ({cx:+.2f},{cy:+.2f}) 가 마스크에서 {d * 100:.0f} cm (> {self.match * 100:.0f})"

    def contact(self, cam, bearing, mf, scan):
        """2차 (11-25 Jasper: 기둥을 LiDAR 로 봐야만 그리면, 가려지거나 4 다리 의자면 영영 못 그림. 떠 있는 가구는 다리가 있고,
           LiDAR 가 보는 건 크기와 상관없이 '접점' → 가구 디자인에 상관없는 규칙 하나):
           카메라는 방향만 정확, 거리 기준점은 LiDAR 가 그 가구에 닿은 점 (접점).
           접점 = 카메라가 본 좌우 끝 방향 사이 + 마스크 아래점 match 안의 LiDAR 점, 단 long (30 cm) 넘게 이어진 선 (벽·책상 옆판) 은 뺌
             (11-26 run_1126: 여러 장면에 벽·책상 점이 섞여 2차가 세로 1.3 m 로 커짐)
           거리 = 접점 중 가까운 쪽 (10 % 분위), 폭 = 끝 방향을 그 거리에 놓은 끝 점 (방향은 믿음)
           장면마다 모양 하나 (합치지 않음) → shape2 가 가운데 넓이의 장면을 고름"""
        cam = np.asarray(cam, float)
        p = np.asarray(scan, float).reshape(-1, 2)
        if not len(p):
            return
        cut = np.where(np.hypot(*np.diff(p, axis=0).T) > 0.05)[0] + 1   # 스캔 순서로 덩어리 → 긴 선은 접점 후보에서 뺌
        short = np.zeros(len(p), bool)
        for idx in np.split(np.arange(len(p)), cut):
            short[idx] = float(np.hypot(*(p[idx].max(0) - p[idx].min(0)))) <= self.long
        ok = short & in_wedge(cam, bearing, p) & (np.hypot(*(p[:, None, :] - mf[None, :, :]).transpose(2, 0, 1)).min(1) <= self.match)
        c = p[ok]
        if not len(c):
            return
        rng = np.hypot(*(c - cam).T)
        r0 = float(np.quantile(rng, 0.1))
        ends = [cam + r0 * np.array([math.cos(b), math.sin(b)]) for b in bearing if b is not None]
        near = mf[np.hypot(*(mf - cam).T) < r0]                         # 바닥에 내린 마스크 점은 실제보다 멀리 찍힐 수는 있어도
        own = np.vstack([c, near, np.array(ends).reshape(-1, 2)])       #   가깝게 찍히진 않음 → 접점보다 가까운 마스크 점 = 가구가 거기까지는 옴
        u = c.mean(0) - cam                                             #   (회전의자: 기둥은 가운데, 바퀴는 34 cm 앞)
        u = u / max(np.linalg.norm(u), 1e-6)
        n = np.array([-u[1], u[0]])
        w, e = float(np.ptp(own @ n)), float(np.ptp(own @ u))
        ext = max(0.0, w - e)                                           # 안 보인 뒤쪽 = 보인 폭만큼 (정사각형 가정)
        legs = [(x, y) for x, y, _ in leg_clusters(p, clear=0.0)]       # 벽 찾기에선 작은 덩어리 (다리·기둥) 만 뺌
        wall = ray_hit(c.mean(0), u, p, width=0.05, skip=legs, skip_r=0.05)
        if math.isfinite(wall):                                         # 벽에서 멈춤 (접점 가운데에서 잰 거리 − 이미 뒤로 나간 만큼)
            ext = max(0.0, min(ext, wall - float(np.max((own - c.mean(0)) @ u))))
        a = np.linspace(0, 2 * math.pi, 8, endpoint=False)
        ring = np.column_stack([np.cos(a), np.sin(a)]) * self.pad
        allp = np.vstack([own, own + ext * u])
        poly = convex((allp[:, None, :] + ring[None, :, :]).reshape(-1, 2))
        x, y = poly.T
        area = 0.5 * abs(float(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))))
        self.c2.append({"poly": poly, "area": area, "w": w, "e": e, "ext": ext, "n": len(c), "c": c})

    def shape2(self):
        """2차 모양: 장면별 모양 중 넓이가 가운데인 것 (한 장면의 엉뚱한 점이 전체를 키우지 않게). 여유 pad 포함."""
        if not self.c2:
            return None
        order = sorted(self.c2, key=lambda v: v["area"])
        v = order[len(order) // 2]
        P = v["poly"]
        lo, hi = order[0]["area"], order[-1]["area"]
        return {"corners": P, "center": P.mean(0), "radius": float(max(v["w"], v["e"] + v["ext"]) / 2 + self.pad), "round": True,
                "confirmed": False, "lo": 0.0, "hi": v["w"], "depth": v["e"] + v["ext"], "spacing": 0.0, "method": 2,
                "why": {"lo": f"2차 장면 {len(self.c2)} 개 중 가운데 (넓이 {v['area']:.2f}, 범위 {lo:.2f}~{hi:.2f} m²), "
                              f"접점 {v['n']} 점, 폭 {v['w']:.2f} 보인 깊이 {v['e']:.2f} + 가정 {v['ext']:.2f}", "hi": "", "depth": ""}}

    def shape(self):
        """1차 (기둥) 가 있으면 1차, 없으면 2차. 둘 다 계산해서 'alt' 에 다른 쪽을 실음 (로그로 비교, 줄자와 맞춰 보고 합치는 규칙은 그다음)"""
        r1 = self.shape1() if self.use1 else None
        r2 = self.shape2() if (self.use2 or not self.use1) else None
        r = r1 or r2
        if r is not None:
            r["alt"] = r2 if r is r1 else None
            r["miss"] = self.miss
        return r

    def shape1(self):
        if not self.cols or not self.radii:
            return None
        c = max(self.cols, key=lambda k: k[2])                         # 가장 많이 본 기둥
        r0 = float(np.median(self.radii))
        r = min(self.max_r, max(self.min_r, r0)) + self.pad
        a = np.linspace(0, 2 * math.pi, self.sides, endpoint=False)
        poly = np.column_stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)])
        why = f"기둥 x{c[2]}, 장면 {len(self.radii)} 개 중간값 {r0:.2f}" + (" (최소로 올림)" if r0 < self.min_r else " (최대로 내림)" if r0 > self.max_r else "")
        return {"corners": poly, "center": np.array(c[:2]), "radius": r, "round": True, "confirmed": False, "method": 1,
                "lo": 0.0, "hi": 2 * r, "depth": 2 * r, "spacing": 0.0, "why": {"lo": "1차 " + why, "hi": "", "depth": ""}}

    def gone(self, scan, cam, bearing):
        """카메라 방향 범위 안 (2 m 안) 인데 기둥 자리에 LiDAR 덩어리가 없음 = 의자를 옮김"""
        if not self.cols or bearing[0] is None or bearing[1] is None:
            return False
        c = max(self.cols, key=lambda k: k[2])
        ang = math.atan2(c[1] - cam[1], c[0] - cam[0])
        lo, hi = min(bearing), max(bearing)
        if not (lo <= ang <= hi) or math.hypot(c[0] - cam[0], c[1] - cam[1]) > 2.0:
            return False
        return not any(math.hypot(x - c[0], y - c[1]) < self.merge for x, y, _ in leg_clusters(scan))
