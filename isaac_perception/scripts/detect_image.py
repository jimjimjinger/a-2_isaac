"""학습된 best.pt 로 이미지 한 장 또는 폴더 추론 + 시각화.

사용:
  python3 detect_image.py path/to/image.png
  python3 detect_image.py path/to/folder/ --output /tmp/yolo_out/
  python3 detect_image.py img.png --model models/mineral_yolo_best.pt --conf 0.3
"""
import argparse
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "isaac_perception"))
from yolo_mineral_detector import YoloMineralDetector


DEFAULT_MODEL = Path(
    "/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_perception/models/mineral_yolo_best.pt"
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="이미지 파일 또는 폴더")
    ap.add_argument("--model", default=str(DEFAULT_MODEL))
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--output", default=None,
                    help="결과 저장 폴더. 미지정 시 cv2.imshow")
    args = ap.parse_args()

    det = YoloMineralDetector(args.model, conf=args.conf)
    inp = Path(args.input).expanduser().resolve()
    out_dir = Path(args.output).expanduser().resolve() if args.output else None
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)

    if inp.is_file():
        imgs = [inp]
    else:
        imgs = sorted(list(inp.glob("*.png")) + list(inp.glob("*.jpg")))
    if not imgs:
        print(f"[ERROR] no images at {inp}")
        return 1

    for img_path in imgs:
        bgr = cv2.imread(str(img_path))
        if bgr is None:
            continue
        dets = det.detect(bgr)
        print(f"[{img_path.name}] {len(dets)} detections")
        for d in dets:
            print(f"  {d.cls_name:14s} conf={d.conf:.3f}  bbox={tuple(int(v) for v in d.bbox)}")
        vis = YoloMineralDetector.draw_overlay(bgr, dets)
        if out_dir:
            cv2.imwrite(str(out_dir / img_path.name), vis)
        else:
            cv2.imshow(img_path.name, vis)
            cv2.waitKey(0)
    if not out_dir:
        cv2.destroyAllWindows()
    if out_dir:
        print(f"\n[done] saved {len(imgs)} images → {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
