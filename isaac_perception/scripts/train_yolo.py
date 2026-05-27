"""Phase 3 — Ultralytics YOLOv8 학습.

사용:
  pip install ultralytics
  python3 train_yolo.py                                    # default: yolov8n, 100 epochs
  python3 train_yolo.py --model yolov8s.pt --epochs 200    # 더 큰 모델
  python3 train_yolo.py --resume runs/detect/mineral_v1    # 재개

학습 종료 후:
  best.pt 가 runs/detect/mineral_vN/weights/ 에 저장됨.
  자동으로 models/mineral_yolo_best.pt 로 복사 (--copy 가 기본 켜짐).
"""
import argparse
import shutil
import os
from pathlib import Path


_A2_ROOT = Path(os.environ.get("A2_ISAAC_ROOT") or Path(__file__).resolve().parents[2])
DEFAULT_DATASET_YAML = _A2_ROOT / "isaac_perception/dataset/dataset.yaml"
DEFAULT_MODELS_DIR = _A2_ROOT / "isaac_perception/models"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=str, default=str(DEFAULT_DATASET_YAML))
    ap.add_argument("--model", type=str, default="yolov8n.pt",
                    help="base model: yolov8n.pt / yolov8s.pt / yolov8m.pt")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--name", type=str, default="mineral_v1")
    ap.add_argument("--patience", type=int, default=20,
                    help="early stopping patience")
    ap.add_argument("--resume", type=str, default=None,
                    help="resume from runs/detect/<name>")
    ap.add_argument("--device", type=str, default="0",
                    help="cuda device (e.g. '0', '0,1', 'cpu')")
    ap.add_argument("--no-copy", action="store_true",
                    help="best.pt 를 models/ 로 복사 안 함")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("[ERROR] ultralytics 미설치. 'pip install ultralytics' 실행")
        return 1

    if args.resume:
        model = YOLO(str(Path(args.resume) / "weights" / "last.pt"))
        kwargs = {"resume": True}
    else:
        model = YOLO(args.model)
        kwargs = {
            "data": args.data,
            "epochs": args.epochs,
            "imgsz": args.imgsz,
            "batch": args.batch,
            "name": args.name,
            "patience": args.patience,
            "device": args.device,
        }
    print(f"[train] start  model={args.model}  kwargs={kwargs}")
    results = model.train(**kwargs)

    # best 자동 복사
    if not args.no_copy:
        # ultralytics 의 결과 위치: runs/detect/<name>/weights/best.pt
        best = Path("runs/detect") / args.name / "weights" / "best.pt"
        if best.exists():
            DEFAULT_MODELS_DIR.mkdir(parents=True, exist_ok=True)
            dst = DEFAULT_MODELS_DIR / "mineral_yolo_best.pt"
            shutil.copy2(best, dst)
            print(f"[copy] {best} → {dst}")
        else:
            print(f"[WARN] best.pt not found at {best}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
