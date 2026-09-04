"""Solution: minimal RANSAC plane fit."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_PLANE_DIR = Path(__file__).resolve().parents[1] / "04_plane_from_3_points"
if str(_PLANE_DIR) not in sys.path:
    sys.path.insert(0, str(_PLANE_DIR))

from plane import plane_from_points  # noqa: E402


def ransac_plane(
    points: np.ndarray,
    threshold: float = 0.05,
    max_iter: int = 100,
    rng: np.random.Generator | None = None,
) -> tuple[np.ndarray, float, np.ndarray]:
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be (N, 3)")
    if points.shape[0] < 3:
        raise ValueError("need at least 3 points")

    rng = rng or np.random.default_rng()
    best_n: np.ndarray | None = None
    best_d: float | None = None
    best_count = -1

    n_pts = points.shape[0]
    for _ in range(max_iter):
        idx = rng.choice(n_pts, size=3, replace=False)
        try:
            n, d = plane_from_points(points[idx[0]], points[idx[1]], points[idx[2]])
        except ValueError:
            continue
        residuals = np.abs(points @ n + d)
        count = int(np.count_nonzero(residuals < threshold))
        if count > best_count:
            best_count = count
            best_n, best_d = n, d

    if best_n is None or best_d is None:
        raise RuntimeError("RANSAC found no valid plane hypothesis")

    mask = np.abs(points @ best_n + best_d) < threshold
    return best_n, best_d, mask
