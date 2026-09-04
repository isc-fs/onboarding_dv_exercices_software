"""Tests for plane_from_points."""
from __future__ import annotations

import numpy as np
import pytest

from plane import plane_from_points


def test_xy_plane() -> None:
    n, d = plane_from_points(
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
    )
    assert n == pytest.approx(np.array([0.0, 0.0, 1.0]), abs=1e-9)
    assert d == pytest.approx(0.0, abs=1e-9)


def test_elevated_xy_plane() -> None:
    n, d = plane_from_points(
        np.array([0.0, 0.0, 2.0]),
        np.array([1.0, 0.0, 2.0]),
        np.array([0.0, 1.0, 2.0]),
    )
    assert n == pytest.approx(np.array([0.0, 0.0, 1.0]), abs=1e-9)
    # n·x + d = 0 → z + d = 0 → d = -2
    assert d == pytest.approx(-2.0, abs=1e-9)


def test_points_lie_on_plane() -> None:
    p1 = np.array([1.0, 0.0, 1.0])
    p2 = np.array([0.0, 1.0, 1.0])
    p3 = np.array([-1.0, 0.0, 1.0])
    n, d = plane_from_points(p1, p2, p3)
    for p in (p1, p2, p3):
        assert float(n @ p) + d == pytest.approx(0.0, abs=1e-9)
    assert float(np.linalg.norm(n)) == pytest.approx(1.0, abs=1e-9)
    assert n[2] >= 0.0


def test_colinear_raises() -> None:
    with pytest.raises(ValueError):
        plane_from_points(
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([2.0, 0.0, 0.0]),
        )
