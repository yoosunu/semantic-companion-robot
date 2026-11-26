#!/usr/bin/env python3
"""face_eval — 얼굴 인식 정확도 재기 (11-26). Pi 에서 돌림 (특징 벡터·사진이 Pi 밖으로 안 나감).

  왜: 투표 규칙 (몇 장 중 몇 장이면 이름) 을 감으로 정하지 않고 데이터로. 남을 나로 착각하는 게 제일 위험
      (엉뚱한 사람을 따라감) → 착각률을 먼저 재고, 그 안에서 나를 빨리 알아보는 규칙을 고름.

  재료
    ① 나 (양성): face_lab.py record <이름> 으로 찍은 ~/face_lab/rec/*.npz (얼굴마다 크기 px320, 검출 점수, 유사도 cos)
    ② 남 (음성): 공개 얼굴 사진 폴더 (LFW: 사람 이름 폴더 / 사진들) → 사진마다 등록된 나와의 유사도
       ~/face_lab/rec/neg_<이름>.npz (cos, 사람 번호) — 이름별로 여러 장이라 "모르는 사람 한 명을 여러 장 본" 투표도 흉내 냄

  쓰기 (Pi)
    ~/yolo_venv/bin/python face_eval.py neg ~/lfw [--max 4000]    음성 만들기 (사진 4000 장 ≈ 몇 분)
    ~/yolo_venv/bin/python face_eval.py report                     표: 상황별 정답률 / 크기별 / 문턱별 착각률 / 투표 규칙
"""
import glob
import json
import os
import sys

import numpy as np

HOME = os.path.expanduser("~")
REC = os.path.join(HOME, "face_lab", "rec")
COS_OK = 0.363
BINS = [(0, 10), (10, 15), (15, 20), (20, 30), (30, 999)]


def neg(folder, max_n=4000):
    """공개 사진 폴더 → 등록된 사람들과의 최고 유사도. 폴더 구조: <사람>/<사진>.jpg (LFW)"""
    import cv2
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from face_lab import Lab
    lab = Lab(det_w=320, min_px=0)
    if not lab.gallery:
        print("등록된 사람 없음 → face_lab.py enroll 먼저")
        return
    files = sorted(glob.glob(os.path.join(folder, "*", "*.jpg")))
    people = sorted({os.path.basename(os.path.dirname(f)) for f in files})
    pid = {p: i for i, p in enumerate(people)}
    rng = np.random.default_rng(0)                       # 이름별로 여러 장 있는 사람을 많이 넣기 (투표 흉내용)
    count = {}
    for f in files:
        count[os.path.dirname(f)] = count.get(os.path.dirname(f), 0) + 1
    multi = [f for f in files if count[os.path.dirname(f)] >= 4]
    mset = set(multi)
    single = [f for f in files if f not in mset]
    pick = multi[:max_n] if len(multi) >= max_n else multi + list(rng.choice(single, min(len(single), max_n - len(multi)), replace=False))
    cos, who, px, feats = [], [], [], []
    for i, f in enumerate(pick):
        bgr = cv2.imread(f)
        fs = lab.faces(bgr) if bgr is not None else []
        if not fs:
            continue
        face, p = max(fs, key=lambda x: x[0][2])         # 가장 큰 얼굴 (LFW 는 가운데 사람)
        _, feat = lab.embed(bgr, face)
        _, c = lab.who(feat)
        cos.append(c), who.append(pid[os.path.basename(os.path.dirname(f))]), px.append(p), feats.append(feat[0])
        if i % 500 == 0:
            print(f"  {i}/{len(pick)} … 지금까지 최고 {max(cos):.3f}")
    name = os.path.basename(os.path.normpath(folder))
    os.makedirs(REC, mode=0o700, exist_ok=True)
    np.savez(os.path.join(REC, f"neg_{name}.npz"), cos=np.array(cos), who=np.array(who), px=np.array(px),
             feat=np.array(feats, np.float32))                  # 11-26 벡터도 → 등록을 바꿔도 다시 검출 안 하고 점수만 다시
    c = np.array(cos)
    print(f"음성 {len(c)} 장 ({len(set(who))} 명): 유사도 중간값 {np.median(c):.3f}, 99% {np.quantile(c, 0.99):.3f}, "
          f"최고 {c.max():.3f}, ≥{COS_OK} {np.sum(c >= COS_OK)} 장")


def vote_ok(seq, thr, k, n):
    """한 사람 추적의 유사도 순서 seq → 처음으로 '최근 n 장 중 k 장 ≥ thr' 이 되는 위치 (없으면 None)"""
    hit = (np.asarray(seq) >= thr).astype(int)
    for i in range(len(hit)):
        if hit[max(0, i - n + 1):i + 1].sum() >= k:
            return i
    return None


def gallery():
    """등록 (~/.scr_faces/*.npy) 을 단위 벡터로. SFace 코사인 = 단위 벡터 내적 (cv2 없이 다시 계산)"""
    g = [np.load(p) for p in sorted(glob.glob(os.path.join(HOME, ".scr_faces", "*.npy")))]
    if not g:
        return None
    g = np.vstack(g).astype(np.float32)
    return g / np.linalg.norm(g, axis=1, keepdims=True)


def rescore(d, G):
    """저장된 벡터로 지금 등록 기준 유사도 (벡터 없으면 기록 때 값 그대로)"""
    if G is None or "feat" not in d or not len(d["feat"]):
        return d["cos"], False
    f = d["feat"] / np.linalg.norm(d["feat"], axis=1, keepdims=True)
    return (f @ G.T).max(1), True


def report():
    G = gallery()
    print(f"지금 등록: {0 if G is None else len(G)} 장 (11-26: 점수는 저장된 벡터로 지금 등록 기준 다시 계산, '기록때' = 찍을 때 등록 기준)")
    pos = {os.path.basename(p)[:-4]: dict(np.load(p)) for p in sorted(glob.glob(os.path.join(REC, "*.npz")))
           if not os.path.basename(p).startswith("neg_")}
    negs = {os.path.basename(p)[4:-4]: dict(np.load(p)) for p in sorted(glob.glob(os.path.join(REC, "neg_*.npz")))}
    for d in pos.values():
        d["cos_rec"] = d["cos"]
        d["cos"], _ = rescore(d, G)
    for k, d in negs.items():
        d["cos"], ok = rescore(d, G)
        if not ok:
            print(f"  ! 음성 {k}: 벡터 없음 → 찍을 때 등록 기준 점수 (등록을 바꿨으면 neg 다시 돌릴 것)")
    print("=== ① 나: 상황별 (한 장 기준, 문턱 0.363)")
    print(f"{'상황':16s} {'영상':>5s} {'얼굴':>5s} {'잡힘%':>6s} {'크기px':>7s} {'cos 중간':>8s} {'≥문턱%':>7s} {'기록때%':>7s} {'2명이상%':>8s}")
    allc, allpx = [], []
    for k, d in pos.items():
        one = d["nface"] == 1
        c, px = d["cos"], d["px"]
        n_img = int(d["n_img"])
        seen = len(set(d["img"].astype(int)))
        print(f"{k:16s} {n_img:5d} {len(c):5d} {seen / max(n_img, 1) * 100:6.0f} {np.median(px):7.0f} {np.median(c):8.2f} "
              f"{(c >= COS_OK).mean() * 100:7.0f} {(d['cos_rec'] >= COS_OK).mean() * 100:7.0f} {(~one).mean() * 100:8.0f}")
        allc.append(c), allpx.append(px)
    if allc:
        c, px = np.concatenate(allc), np.concatenate(allpx)
        print("\n=== ② 나: 얼굴 크기별 (320 기준 px)")
        for lo, hi in BINS:
            m = (px >= lo) & (px < hi)
            if m.any():
                print(f"  {lo:3d}~{hi if hi < 999 else '':>3}px: {m.sum():5d} 장, cos 중간 {np.median(c[m]):.2f}, ≥문턱 {(c[m] >= COS_OK).mean() * 100:.0f}%")
    if negs:
        nc = np.concatenate([d["cos"] for d in negs.values()])
        print(f"\n=== ③ 남 (공개 사진 {len(nc)} 장): 한 장 기준 착각 (남을 나로)")
        for thr in (0.30, 0.33, 0.363, 0.40, 0.45, 0.50):
            mine = f"{(c >= thr).mean() * 100:5.1f}%" if allc else "  -"
            print(f"  문턱 {thr:.3f}: 남 착각 {np.sum(nc >= thr):4d} 장 ({(nc >= thr).mean() * 100:.2f}%) | 나 맞힘 {mine}")
        print(f"  남 유사도: 중간 {np.median(nc):.3f}, 99.9% {np.quantile(nc, 0.999):.3f}, 최고 {nc.max():.3f}")
    print("\n=== ④ 투표 규칙: 최근 n 장 중 k 장 ≥ 문턱 이면 이름")
    print("  나 = 상황별 기록을 한 추적으로 보고 이름 붙기까지 장 수 (영상 기준, 못 붙으면 x)")
    print("  남 = 같은 사람 사진 n 장씩 묶어 (모르는 사람 한 명을 여러 장 본 셈) 이름이 잘못 붙은 묶음 비율")
    rules = [(0.363, 1, 1), (0.363, 2, 3), (0.363, 3, 5), (0.40, 2, 3), (0.40, 3, 5), (0.45, 2, 3), (0.45, 3, 5)]
    for thr, k, n in rules:
        line = f"  문턱 {thr:.3f} {k}/{n}:"
        for name, d in pos.items():
            one = d["nface"] == 1
            i = vote_ok(d["cos"][one], thr, k, n)
            line += f" {name} {('x' if i is None else i + 1)}"
        if negs:
            bad = tot = 0
            for d in negs.values():
                for w in np.unique(d["who"]):
                    s = d["cos"][d["who"] == w]
                    for j in range(0, len(s) - n + 1, n):
                        tot += 1
                        bad += vote_ok(s[j:j + n], thr, k, n) is not None
            line += f" | 남 착각 {bad}/{tot} 묶음"
        print(line)


def main():
    a = sys.argv[1:]
    if a[:1] == ["neg"] and len(a) > 1:
        neg(os.path.expanduser(a[1]), int(a[a.index("--max") + 1]) if "--max" in a else 4000)
    elif a[:1] == ["report"]:
        report()
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
