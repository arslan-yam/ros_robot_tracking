from __future__ import annotations

import cv2
import numpy as np
from .types import WorldPoint


class AffinePixelToWorld:
    def __init__(self, pix_pts: np.ndarray, world_pts: np.ndarray):
        # build affine transform matrix once
        self._A = cv2.getAffineTransform(pix_pts, world_pts)

    def pixel_to_world(self, u: float, v: float) -> WorldPoint:
        A = self._A
        X = float(A[0, 0] * u + A[0, 1] * v + A[0, 2])
        Y = float(A[1, 0] * u + A[1, 1] * v + A[1, 2])
        return WorldPoint(x=X, y=Y)
