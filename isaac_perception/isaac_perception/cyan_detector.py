"""Cyan HSV centroid detector.

OpenCV HSV mask + connected components 로 cyan blob 중심점/면적/bbox 만 반환.
3D pose 는 추정 안 함 (호출자가 servo / GT 로 처리).

dual_cam_pick_place/cyan_tracker.py 패턴을 a2_isaac 워크스페이스용으로 포팅
(외부 CFG 의존성 제거, 인자에 기본값 명시).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


# ── 기본 HSV 범위 (cyan) ─────────────────────────────────────────────
# OpenCV HSV: H 0~179, S/V 0~255. Cyan 은 H≈85~95 부근.
DEFAULT_HSV_LOWER = (80, 100, 80)
DEFAULT_HSV_UPPER = (100, 255, 255)
DEFAULT_MIN_AREA = 200      # px²
DEFAULT_MAX_AREA = 200_000  # px²
DEFAULT_MORPH_KERNEL = 5    # MORPH_OPEN/CLOSE 커널 크기 (홀수 권장)


@dataclass
class Detection:
    found: bool = False
    cx: float = 0.0
    cy: float = 0.0
    area: float = 0.0
    bbox: tuple = (0, 0, 0, 0)
    mask: Optional[np.ndarray] = None


class CyanDetector:
    """OpenCV HSV → cyan blob 픽셀 중심.

    Parameters
    ----------
    hsv_lower / hsv_upper : (H, S, V) bound (OpenCV 범위)
    min_area / max_area   : 픽셀² 단위 면적 필터
    morph_kernel          : MORPH_OPEN+CLOSE 정사각형 커널 변. 0 이면 morph skip
    """

    def __init__(self,
                 hsv_lower: Tuple[int, int, int] = DEFAULT_HSV_LOWER,
                 hsv_upper: Tuple[int, int, int] = DEFAULT_HSV_UPPER,
                 min_area: int = DEFAULT_MIN_AREA,
                 max_area: int = DEFAULT_MAX_AREA,
                 morph_kernel: int = DEFAULT_MORPH_KERNEL):
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.min_area = int(min_area)
        self.max_area = int(max_area)
        self._kernel = (
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))
            if morph_kernel > 0 else None
        )

    def detect(self, bgr: np.ndarray) -> Detection:
        det = Detection()
        if bgr is None or bgr.size == 0:
            return det
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        if self._kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        det.mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return det

        biggest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(biggest))
        if area < self.min_area or area > self.max_area:
            return det
        M = cv2.moments(biggest)
        if M["m00"] <= 0:
            return det
        x, y, w, h = cv2.boundingRect(biggest)

        det.found = True
        det.cx = float(M["m10"] / M["m00"])
        det.cy = float(M["m01"] / M["m00"])
        det.area = area
        det.bbox = (x, y, w, h)
        return det


__all__ = ["CyanDetector", "Detection",
           "DEFAULT_HSV_LOWER", "DEFAULT_HSV_UPPER",
           "DEFAULT_MIN_AREA", "DEFAULT_MAX_AREA", "DEFAULT_MORPH_KERNEL"]
