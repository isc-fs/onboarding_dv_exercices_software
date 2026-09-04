"""Tests for toy Pure Pursuit."""
from __future__ import annotations

import math

import pytest

from pure_pursuit_simple import compute_steer


def test_straight_ahead_zero() -> None:
    assert compute_steer(0.0, 3.0, 1.627, math.radians(28.0)) == pytest.approx(0.0)


def test_left_target_positive_steer() -> None:
    # α > 0 → left of heading → positive κ → positive steer (our convention).
    s = compute_steer(math.radians(20.0), 3.0, 1.627, math.radians(28.0))
    assert s > 0.0


def test_right_target_negative_steer() -> None:
    s = compute_steer(-math.radians(20.0), 3.0, 1.627, math.radians(28.0))
    assert s < 0.0


def test_clamped_to_one() -> None:
    # Large α and short Ld → large κ → saturates.
    s = compute_steer(math.radians(80.0), 0.5, 1.627, math.radians(28.0))
    assert s == pytest.approx(1.0)


def test_known_formula() -> None:
    alpha = math.radians(30.0)
    ld = 4.0
    L = 1.5
    max_steer = math.radians(30.0)
    kappa = 2.0 * math.sin(alpha) / ld
    delta = math.atan(L * kappa)
    expected = max(-1.0, min(1.0, delta / max_steer))
    assert compute_steer(alpha, ld, L, max_steer) == pytest.approx(expected)
