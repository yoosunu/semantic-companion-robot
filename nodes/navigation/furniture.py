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
       - 그쪽 비전 없음 → 끝 다리 + 간격 + pad (넉넉하게)
       - 전체가 max_len (2.2 m, 큰 가구 기준) 넘으면 가려진 쪽부터 자름
    4. 깊이 (d):
       - 확정: 옆에서 본 장면 (카메라가 양 끝 바깥) 의 마스크 방향 끝을 옆면 선에 댐 + 깊은 줄 다리 → 가장 깊은 것 + pad
       - 가정 (확정 없을 때): 앞줄 여러 점에서 깊이 방향으로 LiDAR 벽에 닿을 때까지 (이 가구 다리 점은 뺌) — 가장 먼 것,
         벽 없으면 max_len. 벽에 붙은 가구면 벽까지라 손해 없음 (Jasper). 벽과 떨어져 있으면 옆에서 볼 때 줄어듦.
    5. → 사각형 4 점 (odom) + 근거 (why). 깊이는 "가정"/"확정" 표시.
    pad 는 근거에 따라 (11-25, Jasper: 튀어나온 만큼 통로가 좁아질 걱정):
       LiDAR 로 잰 선 (앞줄, 확정된 깊은 다리) = 0 (다리 중심 + 두께 반) / 비전으로 잰 끝 (마스크) = pad / 추론 (숨은 다리) = pad + 두께.
       벽까지 가정은 벽 자체가 장애물이라 pad 없음. 부풀리기 (whisker_inflation 12 cm) 는 Nav2 가 따로 함.
"""
import math

import numpy as np

FURNITURE = ("bed", "dining table", "couch")   # 밑이 뜬 가구 (지금 실물 시험은 침대만)


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
    def __init__(self, name="bed", match=0.10, merge=0.08, row_gap=0.5, spacing=1.0, max_len=2.2, pad=0.05, overhang=0.3):
        self.name, self.match, self.merge, self.row_gap, self.overhang = name, match, merge, row_gap, overhang
        self.spacing, self.max_len, self.pad = spacing, max_len, pad
        self.legs = []          # [[x, y, 본 횟수, 지름]] odom
        self.views = []         # [(cam_xy, 왼쪽 각, 오른쪽 각 (rad, odom), 그 순간 스캔 odom N×2)]
        self.frame = None       # (a, u, n): 앞줄 한 점, 축, 깊이 방향 (로봇 반대쪽)

    def add_view(self, cam_xy, bearing, mask_floor, scan, robot_xy):
        """한 장: cam_xy 렌즈 위치, bearing (왼쪽 끝, 오른쪽 끝) rad (odom, 왼쪽 +), mask_floor 마스크 아래점 바닥 (odom, nan 은 뺌),
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
        spacing = float(np.median(gaps)) if len(gaps) else self.spacing
        leg_w = max(l[3] for l in self.legs)                         # 다리 두께 (LiDAR 덩어리 지름 최대)
        s_lo, s_hi = float(min(s)), float(max(s))                    # 옆에서 본 깊은 다리도 포함
        # 3. 양 끝: 앞에서 본 장면 (카메라 깊이 < 0, 앞줄 범위 근처) 의 방향 끝 → 앞줄 선
        vis = {-1: [], +1: []}
        side_views = []
        for cam, al, ar, scan in self.views:
            sc, dc = float((cam - a) @ u), float((cam - a) @ n)
            if dc < 0 and sf[0] - 0.3 <= sc <= sf[-1] + 0.3:
                for ang in (al, ar):
                    q = ray_to_line(cam, ang, a, n)
                    if q is not None:
                        sv = float((q - a) @ u)
                        vis[+1 if sv > (sf[0] + sf[-1]) / 2 else -1].append(sv)
            else:
                side_views.append((cam, al, ar, sc))
        ends, why = {}, {}
        for sign, s_leg in ((-1, s_lo), (+1, s_hi)):
            k = "lo" if sign < 0 else "hi"
            if not vis[sign]:
                ends[k], why[k] = s_leg + sign * (spacing + leg_w + self.pad), "비전 없음 → 다리 + 간격 + 두께 + pad"
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
        for cam, al, ar, sc in side_views:
            edge = a + (hi if sc > (lo + hi) / 2 else lo) * u            # 카메라 쪽 옆면 선 (u 가 법선)
            for ang in (al, ar):
                q = ray_to_line(cam, ang, edge, u)
                if q is not None:
                    dv = float((q - a) @ n)
                    if 0 < dv <= self.max_len:
                        conf.append(dv)
        if conf:
            cand = [float(v) + leg_w / 2 for v in deep] + [v + self.pad for v in conf[len(deep):]]   # 다리 = LiDAR (pad 0), 옆면 마스크 = 비전 (pad)
            depth, why_d = max(cand), f"확정 (옆면 마스크·깊은 다리 {len(deep)} 개)"
        else:
            allscan = np.vstack([v[3] for v in self.views])
            hits = [min(ray_hit(a + sv * u, n, allscan, skip=L, t_min=self.row_gap / 2), self.max_len)
                    for sv in np.linspace(lo + self.pad, hi - self.pad, 9)]
            depth = max(hits)
            why_d = "가정 (벽까지)" if depth < self.max_len else "가정 (벽 없음 → 상한)"
        base = a - n * (leg_w / 2)                                   # 앞줄 = LiDAR 다리 선 → pad 없이 다리 두께 반만
        corners = np.array([base + lo * u, base + hi * u, a + hi * u + n * depth, a + lo * u + n * depth])
        return {"corners": corners, "axis_u": u, "origin": a, "normal": n, "lo": lo, "hi": hi, "depth": depth,
                "spacing": spacing, "confirmed": bool(conf), "why": {"lo": why["lo"], "hi": why["hi"], "depth": why_d}}
