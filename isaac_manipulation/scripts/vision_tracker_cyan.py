"""Cyan cube HSV tracker (4일차 BlueBlockTracker 의 cyan 버전).

OpenCV HSV: H 0~179, S/V 0~255. Cyan ≈ H 85~95. Mars 조명에서 desaturate
될 수 있어 S/V 임계를 약간 낮춤 (S≥40, V≥40).
"""
import cv2
import numpy as np
from dataclasses import dataclass


@dataclass
class Detection:
    found: bool
    cx: float = 0.0
    cy: float = 0.0
    area: float = 0.0
    bbox: tuple = (0, 0, 0, 0)
    mask: np.ndarray = None


class CyanCubeTracker:
    """BGR 이미지에서 시안 큐브의 픽셀 중심을 추출."""

    def __init__(self,
                 lower_hsv=(70, 20, 15),
                 upper_hsv=(110, 255, 255),
                 min_area=30,
                 morph_kernel=3):
        self.lower = np.array(lower_hsv, dtype=np.uint8)
        self.upper = np.array(upper_hsv, dtype=np.uint8)
        self.min_area = min_area
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))

    def detect(self, bgr: np.ndarray) -> Detection:
        if bgr is None:
            return Detection(found=False)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return Detection(found=False, mask=mask)

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.min_area:
            return Detection(found=False, area=area, mask=mask)

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return Detection(found=False, mask=mask)

        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        x, y, w, h = cv2.boundingRect(largest)
        return Detection(found=True, cx=cx, cy=cy, area=area,
                         bbox=(x, y, w, h), mask=mask)
