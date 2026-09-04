"""Toy centerline from ordered left/right cone pairs.

Phase A warm-up for path planning frame math / local-window intuition
(``pipeline/path_planning/...``). Production uses FaSTTUBe — you only
build the geometric midpoint idea here.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def world_to_body(pts_xy: np.ndarray, pose: Pose2D) -> np.ndarray:
    """Project (N, 2) world-frame points into body frame (x forward, y left)."""
    # === STUDENT TODO ===
    # Translate by -pose.xy, then rotate by -yaw:
    #   body = R(-yaw) @ (world - pose)
    # Empty input should return a copy of the empty array.
    raise NotImplementedError("STUDENT TODO: world_to_body")
    # === END TODO ===


def midpoint_path(
    left_xy: np.ndarray,
    right_xy: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Build a centerline from paired left/right cones.

    Args:
        left_xy, right_xy: (N, 2) already ordered along the track and
            paired (index i on left matches index i on right).

    Returns:
        ``xy`` — (N, 2) midpoints.
        ``yaw`` — (N,) tangent headings in radians (last point copies
        the previous segment heading; single-point path → yaw 0).
    """
    # === STUDENT TODO ===
    # Midpoint = 0.5 * (left + right).
    # Yaw[i] = atan2 of (xy[i+1] - xy[i]); yaw[-1] = yaw[-2] if N > 1.
    raise NotImplementedError("STUDENT TODO: midpoint_path")
    # === END TODO ===
