"""Solution: fit a plane through three 3D points."""
from __future__ import annotations

import numpy as np


def plane_from_points(
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Return unit normal ``n`` and offset ``d`` with ``n · x + d = 0``."""
    v1 = np.asarray(p2, dtype=np.float64) - np.asarray(p1, dtype=np.float64)
    v2 = np.asarray(p3, dtype=np.float64) - np.asarray(p1, dtype=np.float64)
    normal = np.cross(v1, v2)
    norm = float(np.linalg.norm(normal))
    if norm < 1e-12:
        raise ValueError("points are colinear; cannot define a plane")
    n = normal / norm
    if n[2] < 0.0:
        n = -n
    d = float(-n @ np.asarray(p1, dtype=np.float64))
    return n, d
