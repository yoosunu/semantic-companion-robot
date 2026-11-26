"""person_bind — 얼굴과 몸은 하나 (11-26 Jasper). ROS 없음 → tests/person_bind_sim.py.

  왜: 얼굴 추적만으로는 뒤돌면 1.5 s 뒤 끊기고, 다시 보면 새 추적이라 이름을 처음부터 다시 모음 (11-26 실물 #26 → #29).
      따라오려면 대부분 뒷모습을 봄 → 이름의 주인은 몸 (YOLO person 상자) 이어야 함.
  가정 (Jasper): 얼굴은 사람 상자 안에 있다. 뒤돌면 '얼굴 안 보임' 일 뿐 같은 사람. 한 순간 얼굴을 보여 주면 이름을 붙이고 몸으로 계속 추적.

  몸 추적: 상자 가운데 방향 (고개 + 화면 각도, 도) 으로 이음 — 문턱 max(gate_min, 상자 폭 각도 × 0.5), 폭 비 0.5~2.
           lost_s (3 s) 못 보면 지움 (YOLO 2 Hz 라 띄엄띄엄 → 얼굴 1.5 s 보다 길게).
  묶기: 얼굴 방향이 몸 상자 안 + 몸 가운데보다 위 (얼굴은 윗부분) → 그 몸. 여러 개면 가장 작은 상자 (앞사람 뒤사람 겹칠 때 가까운 쪽이 보통 큼 → 그래도 작은 쪽이 더 꼭 맞음).
  이름: 묶인 얼굴의 인식 결과는 몸의 투표 (OwnerVote) 로. 묶이기 전 얼굴이 모은 투표는 몸이 투표가 없으면 넘겨받음.
        새 얼굴 추적이 이름 있는 몸에 묶이면 바로 그 이름 (다시 모으지 않음). 몸 없는 얼굴은 얼굴 추적이 투표를 가짐.
"""
import math


class Body:
    def __init__(self, bid, d, t):
        self.id, self.born, self.hits, self.vote, self.face = bid, t, 0, None, None
        self.update(d, t)

    def update(self, d, t):
        self.pan, self.tilt, self.w, self.h = d["pan"], d["tilt"], d["w"], d["h"]
        self.seen, self.hits = t, self.hits + 1

    def contains_face(self, pan, tilt, margin=2.0):
        return (abs(pan - self.pan) <= self.w / 2 + margin and self.tilt - margin <= tilt <= self.tilt + self.h / 2 + margin)

    @property
    def name(self):
        return self.vote.state if self.vote else ""


class PeopleBinder:
    def __init__(self, vote_factory, gate_min=10.0, lost_s=3.0):
        self.new_vote, self.gate_min, self.lost_s = vote_factory, gate_min, lost_s
        self.bodies, self.next_id = {}, 1
        self.face_vote, self.face_body = {}, {}         # 얼굴 추적 id → 투표 (몸 없을 때) / 묶인 몸 id

    # ---------- 몸 ----------
    def step_bodies(self, dets, t):
        """dets: [{"pan","tilt","w","h"}] 이번 YOLO 장의 person 상자 (각도, 도). → 이번에 갱신·생성된 몸 id"""
        pairs = []
        for b in self.bodies.values():
            for j, d in enumerate(dets):
                if not 0.5 <= d["w"] / max(1e-3, b.w) <= 2.0:
                    continue
                dist = math.hypot(d["pan"] - b.pan, d["tilt"] - b.tilt)
                if dist <= max(self.gate_min, 0.5 * d["w"]):
                    pairs.append((dist, b.id, j))
        pairs.sort()
        used_b, used_d, touched = set(), set(), []
        for _, bid, j in pairs:
            if bid in used_b or j in used_d:
                continue
            self.bodies[bid].update(dets[j], t)
            used_b.add(bid), used_d.add(j), touched.append(bid)
        for j, d in enumerate(dets):
            if j not in used_d:
                self.bodies[self.next_id] = Body(self.next_id, d, t)
                touched.append(self.next_id)
                self.next_id += 1
        for bid in [k for k, b in self.bodies.items() if t - b.seen > self.lost_s]:
            del self.bodies[bid]
            for f in [f for f, b in self.face_body.items() if b == bid]:
                del self.face_body[f]
        return touched

    # ---------- 묶기 ----------
    def bind(self, faces, t):
        """faces: [(얼굴 추적 id, pan, tilt)] — YOLO 를 돌린 장에서 이번에 보인 얼굴. 몸 상자 윗부분 안이면 묶음 → {얼굴 id: 몸 id}"""
        out = {}
        for fid, pan, tilt in faces:
            cand = [b for b in self.bodies.values() if b.seen == t and b.contains_face(pan, tilt)]
            if not cand:
                continue
            b = min(cand, key=lambda b: b.w * b.h)
            out[fid] = b.id
            self.face_body[fid] = b.id
            b.face = fid
            fv = self.face_vote.pop(fid, None)
            if b.vote is None:
                b.vote = fv or self.new_vote()          # 묶이기 전 얼굴이 모은 투표를 넘겨받음
        return out

    def drop_faces(self, live):
        """끝난 얼굴 추적 정리 (몸은 남음 — 뒤돌아도 이름 유지)"""
        for f in [f for f in self.face_vote if f not in live]:
            del self.face_vote[f]
        for f in [f for f in self.face_body if f not in live]:
            b = self.bodies.get(self.face_body.pop(f))
            if b and b.face == f:
                b.face = None

    # ---------- 이름 ----------
    def vote_of(self, fid):
        """이 얼굴의 인식 결과를 넣을 투표: 묶인 몸의 것, 아니면 얼굴 자기 것"""
        b = self.bodies.get(self.face_body.get(fid))
        if b is not None:
            if b.vote is None:
                b.vote = self.new_vote()
            return b.vote
        return self.face_vote.setdefault(fid, self.new_vote())

    def name_of(self, fid):
        b = self.bodies.get(self.face_body.get(fid))
        if b is not None:
            return b.name
        v = self.face_vote.get(fid)
        return v.state if v else ""

    def body_list(self, t):
        return [{"id": b.id, "name": b.name, "pan": round(float(b.pan), 1), "tilt": round(float(b.tilt), 1), "w": round(float(b.w), 1),
                 "h": round(float(b.h), 1), "face": None if b.face is None else int(b.face), "unseen": round(float(t - b.seen), 2)}
                for b in self.bodies.values()]   # 11-26 실물: YOLO 좌표가 numpy float32 → json 이 못 써서 노드가 죽음 → float 로
