#!/usr/bin/env python3
"""[Mac] YOLOE 단어 정해서 ONNX 로 내보내기 (11-18). YOLOE 는 단어를 내보낼 때 정함 → 단어를 바꾸면 다시 실행.

  원칙 (11-18 Jasper): COCO 80 종류에 있는 건 YOLO26 이 더 잘 봄 → 여기는 **COCO 에 없는 집 물건만**.
    (핸드폰 = cell phone, 강아지 인형 = teddy bear, 의자·침대·소파·책·컵·병·가방·리모컨·노트북은 COCO)
  필요: pip install ultralytics  (처음 실행 때 yoloe-26s-seg.pt 와 글자 모델을 받음)
  쓰기: python3 tools/mac/export_yoloe.py                → models/yoloe-26s-seg-home.onnx (320)
        python3 tools/mac/export_yoloe.py "trash can" sock …   단어 직접
  Pi 로: scp models/yoloe-26s-seg-home.onnx jasper@<Pi>:~/models/
        → glance_node 에서 쓰기: INSPECT_ARGS="-p yoloe_model:=~/models/yoloe-26s-seg-home.onnx"
"""
import os
import shutil
import sys

WORDS = [
    "trash can", "doormat", "rug", "cable", "power strip", "charger",
    "sock", "shoe", "slipper", "clothes", "towel", "pillow", "blanket",
    "cardboard box", "plastic bag", "laundry basket", "storage box",
    "glasses", "earphones", "wallet", "keys", "tissue box", "toy", "stuffed animal",
    "drawer", "shelf", "door", "curtain",
]


def main():
    from ultralytics import YOLOE
    words = sys.argv[1:] or WORDS
    model = YOLOE("yoloe-26s-seg.pt")
    try:
        model.set_classes(words, model.get_text_pe(words))
    except (AttributeError, TypeError):
        model.set_classes(words)
    out = model.export(format="onnx", imgsz=320)
    here = os.path.dirname(os.path.abspath(__file__))
    dst = os.path.join(here, "..", "..", "models", "yoloe-26s-seg-home.onnx")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.move(out, dst)
    print(f"단어 {len(words)}개 → {os.path.realpath(dst)}")


if __name__ == "__main__":
    main()
