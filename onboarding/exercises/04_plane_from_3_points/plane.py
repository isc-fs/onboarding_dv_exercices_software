"""Fit a plane through three 3D points.

Phase A warm-up for the RANSAC ground-plane step in
``pipeline/cone_detection/cone_detection/ransac.py``.
"""
from __future__ import annotations

import numpy as np


def plane_from_points(
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Return unit normal ``n`` and offset ``d`` with ``n · x + d = 0``.

    Args:
        p1, p2, p3: shape (3,) Cartesian points.

    Returns:
        ``n`` — unit normal, shape (3,), with ``n[2] >= 0`` (prefer +z).
        ``d`` — scalar offset so that ``n · x + d = 0`` for every point
        on the plane.

    Raises:
        ValueError: if the three points are (nearly) colinear.
    """
    # === STUDENT TODO ===
    # 1. Form two edge vectors from p1.
    # 2. normal = cross product; reject if ||normal|| is tiny.
    # 3. Unitize; flip so n_z >= 0.
    # 4. d = -n · p1.
    raise NotImplementedError("STUDENT TODO: plane_from_points")
    # === END TODO ===
