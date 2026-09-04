"""Tests for mini RANSAC."""
from __future__ import annotations

import numpy as np
import pytest

from ransac_simple import ransac_plane


def _ground_with_outliers(n_in: int = 200, n_out: int = 40, seed: int = 0):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-5, 5, size=(n_in, 2))
    z = np.full((n_in, 1), 0.0) + rng.normal(0, 0.01, size=(n_in, 1))
    ground = np.hstack([xy, z])
    outliers = rng.uniform(-5, 5, size=(n_out, 3))
    outliers[:, 2] = rng.uniform(0.5, 3.0, size=n_out)
    pts = np.vstack([ground, outliers])
    return pts, n_in


def test_recovers_ground_plane() -> None:
    pts, n_in = _ground_with_outliers()
    n, d, mask = ransac_plane(pts, threshold=0.05, max_iter=200,
                              rng=np.random.default_rng(1))
    assert n[2] == pytest.approx(1.0, abs=0.05)
    assert abs(d) < 0.05
    # Most ground points should be inliers; most outliers should not.
    assert mask[:n_in].mean() > 0.9
    assert mask[n_in:].mean() < 0.2


def test_too_few_points_raises() -> None:
    with pytest.raises(ValueError):
        ransac_plane(np.zeros((2, 3)))
