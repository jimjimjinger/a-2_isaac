""".txt YOLO 라벨 → 원본 PNG 에 박스 그려 preview 저장.

usage:
  python3 scripts/render_labels.py <image_path> [<image_path> ...]

각 이미지의 라벨은 같은 폴더의 `<stem>.txt` 에서 자동으로 읽음.
preview 는 dataset/manual/_preview/<color>/<image_name>.png 로 저장.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2

PKG = Path(__file__).resolve().parents[1]
PREVIEW_ROOT = PKG / "dataset" / "manual" / "_preview"
NAMES = {0: "blue_mineral", 1: "yellow_mineral", 2: "green_gas"}
BGR = {0: (255, 200, 0), 1: (0, 255, 255), 2: (0, 255, 0)}


def render(img_path: Path) -> None:
    label_path = img_path.with_suffix(".txt")
    if not label_path.exists():
        print(f"  [skip] no label: {label_path}")
        return
    img = cv2.imread(str(img_path))
    if img is None:
        print(f"  [skip] cannot read: {img_path}")
        return
    H, W = img.shape[:2]
    viz = img.copy()

    txt = label_path.read_text()
    boxes = 0
    for line in txt.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        cid = int(parts[0])
        cx, cy, w, h = (float(p) for p in parts[1:5])
        x = int((cx - w / 2) * W)
        y = int((cy - h / 2) * H)
        ww = int(w * W)
        hh = int(h * H)
        color = BGR.get(cid, (255, 255, 255))
        cv2.rectangle(viz, (x, y), (x + ww, y + hh), color, 2)
        cv2.putText(viz, NAMES.get(cid, str(cid)), (x, max(0, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        boxes += 1

    folder = img_path.parent.name
    dst = PREVIEW_ROOT / folder / img_path.name
    dst.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(dst), viz)
    print(f"  [{img_path.name}] {boxes} bbox → {dst}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        render(Path(p))
