"""Tests for midpoint path + world_to_body."""
from __future__ import annotations

import math

import numpy as np
import pytest

from midpoint_path import Pose2D, midpoint_path, world_to_body


def test_world_to_body_identity() -> None:
    pts = np.array([[1.0, 0.0], [0.0, 1.0]])
    out = world_to_body(pts, Pose2D(0.0, 0.0, 0.0))
    assert out == pytest.approx(pts)


def test_world_to_body_translate_and_yaw() -> None:
    # Point 1 m ahead of a car at (2, 3) facing +x → body (1, 0).
    pts = np.array([[3.0, 3.0]])
    out = world_to_body(pts, Pose2D(2.0, 3.0, 0.0))
    assert out == pytest.approx(np.array([[1.0, 0.0]]))

    # Same world point, car facing +y (yaw = π/2): ahead is world +y,
    # so body_x = 0, body_y = -1 relative? Wait: car at (2,3), point at
    # (3,3) is to the right when facing +y → body (0, -1).
    out_yaw = world_to_body(pts, Pose2D(2.0, 3.0, math.pi / 2))
    assert out_yaw == pytest.approx(np.array([[0.0, -1.0]]), abs=1e-9)


def test_midpoint_straight() -> None:
    left = np.array([[0.0, 1.0], [2.0, 1.0], [4.0, 1.0]])
    right = np.array([[0.0, -1.0], [2.0, -1.0], [4.0, -1.0]])
    xy, yaw = midpoint_path(left, right)
    assert xy == pytest.approx(np.array([[0.0, 0.0], [2.0, 0.0], [4.0, 0.0]]))
    assert yaw[0] == pytest.approx(0.0, abs=1e-9)
    assert yaw[1] == pytest.approx(0.0, abs=1e-9)
    assert yaw[2] == pytest.approx(yaw[1])


def test_empty_world_to_body() -> None:
    out = world_to_body(np.zeros((0, 2)), Pose2D(0.0, 0.0, 0.0))
    assert out.shape == (0, 2)
