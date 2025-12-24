from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from .types import HSVRange, PixelPoint


def find_marker_center_pixel(
    frame_bgr: np.ndarray,
    hsv_range: HSVRange,
    margin_of_error_area: float,
) -> Optional[PixelPoint]:
    # convert picture BGR → HSV
    hsv_image = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_image, hsv_range.lower, hsv_range.upper)
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,  # RETR_EXTERNAL for external contours
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None

    marker = max(contours, key=cv2.contourArea)
    marker_area = cv2.contourArea(marker)
    if marker_area < margin_of_error_area:
        return None

    M = cv2.moments(marker)
    if M["m00"] == 0:
        return None

    center_x = float(M["m10"] / M["m00"])
    center_y = float(M["m01"] / M["m00"])
    return PixelPoint(u=center_x, v=center_y)
