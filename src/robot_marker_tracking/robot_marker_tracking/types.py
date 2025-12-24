from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class HSVRange:
    lower: np.ndarray
    upper: np.ndarray


@dataclass(frozen=True)
class PixelPoint:
    u: float
    v: float


@dataclass(frozen=True)
class WorldPoint:
    x: float
    y: float
