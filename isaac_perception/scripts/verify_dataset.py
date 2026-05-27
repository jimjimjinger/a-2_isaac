"""Phase 2 — 수집된 YOLO 데이터셋의 bbox annotation 시각 검증.

샘플 이미지에 bbox 그려서 사람이 눈으로 확인.

사용:
  python3 verify_dataset.py                       # 기본: 무작위 16장 격자 표시
  python3 verify_dataset.py --n 25 --split val
  python3 verify_dataset.py --output /tmp/verify  # 격자 이미지 저장
"""
import argparse
import random
import os
from pathlib import Path

_A2_ROOT = Path(os.environ.get("A2_ISAAC_ROOT") or Path(__file__).resolve().parents[2])

import cv2
import numpy as np


CLASS_COLORS = {
    0: (255, 100, 100),    # blue_mineral — 파란계열
    1: (100, 255, 100),    # green_gas    — 초록계열
    2: (50, 220, 255),     # yellow_mineral — 노랑계열
}
CLASS_NAMES = ["blue", "green_gas", "yellow"]
DEFAULT_DATASET = _A2_ROOT / "isaac_perception/dataset"


def yolo_to_xyxy(cx, cy, w, h, img_w, img_h):
    x1 = (cx - w / 2) * img_w
    y1 = (cy - h / 2) * img_h
    x2 = (cx + w / 2) * img_w
    y2 = (cy + h / 2) * img_h
    return int(x1), int(y1), int(x2), int(y2)


def draw_labels(img, label_path):
    if not label_path.exists():
        return img
    H, W = img.shape[:2]
    with open(label_path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            cid = int(parts[0])
            cx, cy, w, h = map(float, parts[1:])
            x1, y1, x2, y2 = yolo_to_xyxy(cx, cy, w, h, W, H)
            color = CLASS_COLORS.get(cid, (200, 200, 200))
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            label = CLASS_NAMES[cid] if cid < len(CLASS_NAMES) else f"cls{cid}"
            cv2.putText(img, label, (x1, max(15, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", type=str, default=str(DEFAULT_DATASET))
    ap.add_argument("--split", choices=["train", "val"], default="train")
    ap.add_argument("--n", type=int, default=16, help="격자에 표시할 이미지 수")
    ap.add_argument("--output", type=str, default=None,
                    help="설정 시 격자 PNG 저장 (cv2 imshow 없을 때 유용)")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    ds = Path(args.dataset)
    img_dir = ds / "images" / args.split
    lbl_dir = ds / "labels" / args.split

    imgs = sorted(img_dir.glob("*.png"))
    if not imgs:
        print(f"[ERROR] no images in {img_dir}")
        return 1
    print(f"[verify] {len(imgs)} images in {img_dir}")
    sample = rng.sample(imgs, min(args.n, len(imgs)))

    # 격자 크기 계산
    cols = int(np.ceil(np.sqrt(len(sample))))
    rows = int(np.ceil(len(sample) / cols))
    cell_h, cell_w = 240, 320

    grid = np.full((rows * cell_h, cols * cell_w, 3), 80, dtype=np.uint8)
    for i, img_path in enumerate(sample):
        lbl_path = lbl_dir / (img_path.stem + ".txt")
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        img = draw_labels(img, lbl_path)
        img = cv2.resize(img, (cell_w, cell_h))
        # 파일명 표기
        cv2.putText(img, img_path.stem, (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        # 라벨 수
        n_lbl = 0
        if lbl_path.exists():
            n_lbl = sum(1 for _ in open(lbl_path))
        cv2.putText(img, f"n={n_lbl}", (cell_w - 50, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 255), 1)
        r, c = divmod(i, cols)
        grid[r * cell_h:(r + 1) * cell_h, c * cell_w:(c + 1) * cell_w] = img

    if args.output:
        out_path = Path(args.output).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out_path), grid)
        print(f"[save] {out_path}")
    else:
        try:
            cv2.imshow("verify (q to quit)", grid)
            while True:
                key = cv2.waitKey(0) & 0xFF
                if key in (ord('q'), 27):
                    break
            cv2.destroyAllWindows()
        except cv2.error as e:
            print(f"[ERROR] cv2.imshow 실패 ({e}). --output 으로 PNG 저장 사용")
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
