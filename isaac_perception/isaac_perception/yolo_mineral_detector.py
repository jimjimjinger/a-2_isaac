"""YOLO Mineral Detector — best.pt 로 RGB 이미지에서 mineral detect.

사용 예:
    from yolo_mineral_detector import YoloMineralDetector
    det = YoloMineralDetector("models/mineral_yolo_best.pt", conf=0.5)
    dets = det.detect(bgr_image)
    for d in dets:
        print(d.cls_name, d.conf, d.bbox)
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np


CLASS_NAMES = ["blue_mineral", "green_gas", "yellow_mineral"]
CLASS_COLORS = {
    0: (255, 100, 100),     # blue
    1: (100, 255, 100),     # green
    2: (50, 220, 255),      # yellow
}


@dataclass
class Detection:
    cls_id: int
    cls_name: str
    conf: float
    bbox: tuple        # (x1, y1, x2, y2) in pixels
    cx: float
    cy: float


class YoloMineralDetector:
    def __init__(self, model_path: str, conf: float = 0.5,
                 iou: float = 0.45, device: Optional[str] = None):
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise ImportError(
                "ultralytics 미설치. 'pip install ultralytics'\n"
                f"원본 에러: {e}"
            )
        mp = Path(model_path).expanduser().resolve()
        if not mp.exists():
            raise FileNotFoundError(f"YOLO model not found: {mp}")
        self.model = YOLO(str(mp))
        self.conf = conf
        self.iou = iou
        self.device = device
        self.names = CLASS_NAMES

    def detect(self, bgr: np.ndarray) -> List[Detection]:
        """BGR 이미지 → Detection 리스트."""
        kwargs = {"conf": self.conf, "iou": self.iou, "verbose": False}
        if self.device is not None:
            kwargs["device"] = self.device
        results = self.model(bgr, **kwargs)
        out: List[Detection] = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                xyxy = box.xyxy[0].tolist()
                x1, y1, x2, y2 = xyxy
                out.append(Detection(
                    cls_id=cls_id,
                    cls_name=self.names[cls_id] if cls_id < len(self.names) else f"cls{cls_id}",
                    conf=conf,
                    bbox=(x1, y1, x2, y2),
                    cx=(x1 + x2) / 2,
                    cy=(y1 + y2) / 2,
                ))
        return out

    @staticmethod
    def draw_overlay(bgr: np.ndarray, dets: List[Detection]) -> np.ndarray:
        """Detection 들을 이미지에 그려서 반환 (copy)."""
        import cv2
        out = bgr.copy()
        for d in dets:
            x1, y1, x2, y2 = [int(v) for v in d.bbox]
            color = CLASS_COLORS.get(d.cls_id, (200, 200, 200))
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            label = f"{d.cls_name} {d.conf:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(out, (x1, max(0, y1 - th - 6)),
                          (x1 + tw + 4, y1), color, -1)
            cv2.putText(out, label, (x1 + 2, max(12, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        return out


__all__ = ["YoloMineralDetector", "Detection", "CLASS_NAMES", "CLASS_COLORS"]
