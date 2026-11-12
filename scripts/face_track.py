"""face_track — 얼굴 추적기 (ROS 없음, 순수 계산). face_perception 노드가 씀, tests/face_track_sim.py 로 시험.

  하는 일: 영상마다 나온 얼굴 상자들 → "아까 그 사람" 끼리 이어서 트랙 번호를 붙임.

  핵심 아이디어: 화면 좌표가 아니라 **방향(도)** 으로 잇는다.
    방향 = 영상 찍힐 때 고개 각도 + 화면 안 각도.  고개가 확 돌면 화면 속 얼굴은 튀지만 방향은 그대로 → 트랙 안 끊김.

  잇는 규칙 (영상 한 장마다)
    1. 모든 (트랙, 얼굴) 짝의 방향 거리를 잼. 문턱 = max(8°, 얼굴 크기 각도 × 2) 안이고, 크기 비율 0.5~2 배인 것만 후보
    2. 가까운 짝부터 하나씩 확정 (욕심쟁이 매칭: 한 트랙엔 얼굴 하나)
    3. 짝 없는 얼굴 → 새 트랙 (아직 "확인 전"). 두 번 보이면 확인됨 → 그때부터 밖으로 알림 (한 장짜리 헛것 거르기)
    4. 1.5 s 못 본 트랙 → 지움. 다음에 다시 보이면 새 번호
"""
import math

HFOV, VFOV = 120.0, 90.0    # 카메라 가로·세로 화각 (도). 어안이라 대략값 (look_at_person 과 같음)
GATE_MIN = 8.0              # 잇기 문턱 최소 (도)
LOST_S = 1.5                # 이만큼 못 보면 트랙 지움
CONFIRM_HITS = 2            # 이만큼 보여야 확인됨


def face_angles(x, y, img_w, img_h):
    """화면 픽셀 → 화면 가운데 기준 각도 (도). + = 오른쪽 / 위. 등거리 어안 근사: 픽셀이 각도에 비례"""
    ex = (x - img_w / 2) / (img_w / 2) * (HFOV / 2)
    ey = (img_h / 2 - y) / (img_h / 2) * (VFOV / 2)
    return ex, ey


def yaw_pitch(lm):
    """YuNet 점 5개 [(x,y) 오른눈, 왼눈, 코, 오른입, 왼입] (사람 기준 좌우, 화면에선 오른눈이 왼쪽) → 얼굴 방향 (도, 대략)
       yaw  + = 사람이 자기 왼쪽으로 고개 돌림 (화면에선 코가 오른쪽으로)
       pitch + = 위를 봄 (코가 눈 쪽으로 올라감)"""
    (rex, rey), (lex, ley), (nx, ny), (rmx, rmy), (lmx, lmy) = lm
    ex, ey = (rex + lex) / 2, (rey + ley) / 2
    d = math.hypot(lex - rex, ley - rey) or 1.0
    yaw = max(-90.0, min(90.0, (nx - ex) / d * 100.0))
    my = (rmy + lmy) / 2
    span = (my - ey) or 1.0
    pitch = max(-90.0, min(90.0, (0.55 - (ny - ey) / span) * 150.0))
    return yaw, pitch


class Track:
    def __init__(self, tid, d, t):
        self.id, self.born, self.hits = tid, t, 0
        self.update(d, t)

    def update(self, d, t):
        self.pan, self.tilt, self.px = d["pan"], d["tilt"], d["px"]
        self.det, self.yaw, self.pitch = d["det"], d["yaw"], d["pitch"]
        self.box, self.lm = d.get("box"), d.get("lm")      # 이번 영상 속 위치 (인식할 때 얼굴 자르기용)
        self.seen, self.hits = t, self.hits + 1


class Tracker:
    def __init__(self):
        self.tracks, self.next_id = {}, 1

    def step(self, dets, t):
        """dets: [{"pan","tilt","px","det","yaw","pitch",...}] 이번 영상의 얼굴들 (방향은 이미 고개 보정됨)
           t: 영상 찍힌 시각.  → 이번에 갱신·생성된 트랙 id 목록"""
        pairs = []
        for tr in self.tracks.values():
            for j, d in enumerate(dets):
                ratio = d["px"] / max(1e-3, tr.px)
                if not 0.5 <= ratio <= 2.0:
                    continue                                # 크기가 너무 다르면 다른 사람 (가까운 사람 vs 먼 사람)
                dist = math.hypot(d["pan"] - tr.pan, d["tilt"] - tr.tilt)
                gate = max(GATE_MIN, 2.0 * d["px"] * HFOV / 320.0)   # 얼굴 폭 각도 × 2
                if dist <= gate:
                    pairs.append((dist, tr.id, j))
        pairs.sort()
        used_t, used_d, touched = set(), set(), []
        for _, tid, j in pairs:
            if tid in used_t or j in used_d:
                continue
            self.tracks[tid].update(dets[j], t)
            used_t.add(tid)
            used_d.add(j)
            touched.append(tid)
        for j, d in enumerate(dets):
            if j not in used_d:
                self.tracks[self.next_id] = Track(self.next_id, d, t)
                touched.append(self.next_id)
                self.next_id += 1
        for tid in [k for k, tr in self.tracks.items() if t - tr.seen > LOST_S]:
            del self.tracks[tid]
        return touched

    def confirmed(self):
        """밖으로 알릴 트랙들 (두 번 이상 보인 것), 얼굴 큰 순"""
        return sorted((tr for tr in self.tracks.values() if tr.hits >= CONFIRM_HITS), key=lambda tr: -tr.px)
