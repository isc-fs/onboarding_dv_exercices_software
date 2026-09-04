"""Solution: Pure Pursuit steering formula."""
from __future__ import annotations

import math


def compute_steer(
    alpha: float,
    ld: float,
    wheelbase: float,
    max_steer: float,
) -> float:
    if ld <= 0.0:
        raise ValueError("ld must be > 0")
    if max_steer <= 0.0:
        raise ValueError("max_steer must be > 0")
    kappa = 2.0 * math.sin(alpha) / ld
    delta = math.atan(wheelbase * kappa)
    return max(-1.0, min(1.0, delta / max_steer))
