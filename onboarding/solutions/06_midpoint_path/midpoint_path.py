"""Solution: midpoint path + world_to_body."""
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
    if pts_xy.shape[0] == 0:
        return pts_xy.copy()
    cos_y, sin_y = float(np.cos(pose.yaw)), float(np.sin(pose.yaw))
    R = np.array([[cos_y, sin_y], [-sin_y, cos_y]], dtype=np.float64)
    return (pts_xy - np.array([pose.x, pose.y], dtype=np.float64)) @ R.T


def midpoint_path(
    left_xy: np.ndarray,
    right_xy: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if left_xy.shape != right_xy.shape:
        raise ValueError("left and right must have the same shape")
    xy = 0.5 * (left_xy + right_xy)
    n = xy.shape[0]
    yaw = np.zeros(n, dtype=np.float64)
    if n == 0:
        return xy, yaw
    if n == 1:
        return xy, yaw
    for i in range(n - 1):
        dx = xy[i + 1, 0] - xy[i, 0]
        dy = xy[i + 1, 1] - xy[i, 1]
        yaw[i] = math.atan2(dy, dx)
    yaw[-1] = yaw[-2]
    return xy, yaw
