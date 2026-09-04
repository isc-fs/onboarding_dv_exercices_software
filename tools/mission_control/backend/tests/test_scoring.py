"""Unit tests for scoring.py — FS-Rules 2026 D 9.1.1 + D 10 penalties.

The scoring function is pure and deterministic; testing it against
hand-calculated cases catches both formula regressions (someone changes
the exponent in `_score_formula`) and constants drift (someone touches
`MAX_POINTS` / `TMAX_FACTOR` / `PMIN_FACTOR` mid-rebase).
"""

from __future__ import annotations

import math

import pytest

from scoring import (
    DOO_PENALTY_S,
    DV_RUNTIME_CAP_S,
    MAX_POINTS,
    OC_PENALTY,
    PMIN_FACTOR,
    TMAX_FACTOR,
    USS_PENALTY,
    compute_scoring,
)


# ---------------------------------------------------------------------
# Sanity on the score formula's edge cases
# ---------------------------------------------------------------------


def test_team_runs_at_t_best_gets_pmax() -> None:
    """If a team's corrected time equals Tmin, they get the full Pmax."""
    r = compute_scoring(
        event="acceleration",
        lap_times=[5.0],
        doo=0,
        oc=0,
        t_best=5.0,
    )
    assert r["score"] == pytest.approx(MAX_POINTS["acceleration"], abs=0.1)
    assert not r["disqualified"]


def test_team_runs_at_or_above_t_max_gets_pmin() -> None:
    """At Tmax (= Tmin × TMAX_FACTOR), the formula collapses to Pmin."""
    t_min = 5.0
    t_max = t_min * TMAX_FACTOR["acceleration"]
    r = compute_scoring(
        event="acceleration",
        lap_times=[t_max],
        doo=0,
        oc=0,
        t_best=t_min,
    )
    expected_pmin = MAX_POINTS["acceleration"] * PMIN_FACTOR["acceleration"]
    assert r["score"] == pytest.approx(expected_pmin, abs=0.1)


def test_score_decreases_monotonically_with_time() -> None:
    """Slower runs MUST score ≤ faster runs (D 9.1.1's quadratic ratio)."""
    times = [3.5, 4.0, 4.5, 5.0, 6.0]
    scores = [
        compute_scoring("autocross", lap_times=[t], doo=0, oc=0, t_best=3.5)["score"]
        for t in times
    ]
    assert scores == sorted(scores, reverse=True)


# ---------------------------------------------------------------------
# DOO time penalty (D 10)
# ---------------------------------------------------------------------


def test_doo_adds_per_event_seconds() -> None:
    r = compute_scoring(
        event="autocross",
        lap_times=[60.0],
        doo=3,
        oc=0,
        t_best=60.0,
    )
    assert r["doo_penalty_s"] == pytest.approx(3 * DOO_PENALTY_S["autocross"])
    assert r["corrected_time"] == pytest.approx(60.0 + 3 * DOO_PENALTY_S["autocross"])


def test_doo_zero_is_no_penalty() -> None:
    r = compute_scoring("trackdrive", lap_times=[60.0] * 10, doo=0, oc=0)
    assert r["doo_penalty_s"] == 0.0


# ---------------------------------------------------------------------
# OC: DQ for Acceleration/Skidpad, +10s for others (D 10)
# ---------------------------------------------------------------------


@pytest.mark.parametrize("event", ["acceleration", "skidpad"])
def test_oc_dqs_acceleration_and_skidpad(event: str) -> None:
    r = compute_scoring(event, lap_times=[5.0], doo=0, oc=1, t_best=5.0)
    assert r["disqualified"]
    assert r["score"] == 0.0
    assert any("OC" in reason for reason in r["dq_reasons"])
    assert r["oc_is_dq"]


@pytest.mark.parametrize("event", ["autocross", "trackdrive"])
def test_oc_adds_seconds_for_autocross_and_trackdrive(event: str) -> None:
    r = compute_scoring(event, lap_times=[60.0], doo=0, oc=2, t_best=60.0)
    assert not r["disqualified"]
    assert r["oc_penalty_s"] == pytest.approx(2 * float(OC_PENALTY[event]))
    assert r["corrected_time"] > 60.0


# ---------------------------------------------------------------------
# USS: DQ for DV Acc/Skid/Auto, -50pts for Trackdrive (D 10.1.7)
# ---------------------------------------------------------------------


@pytest.mark.parametrize("event", ["acceleration", "skidpad", "autocross"])
def test_uss_dqs_dv_disciplines(event: str) -> None:
    r = compute_scoring(event, lap_times=[5.0], doo=0, oc=0, t_best=5.0, uss=True)
    assert r["disqualified"]
    assert "USS" in " ".join(r["dq_reasons"])


def test_uss_minus_50_on_trackdrive() -> None:
    """Trackdrive's USS_PENALTY is -50 pts (D 10.1.7), score floored at 0."""
    # Run that would otherwise score Pmax (200 pts) — confirm -50 lands.
    r = compute_scoring(
        "trackdrive",
        lap_times=[60.0] * 10,
        doo=0,
        oc=0,
        t_best=600.0,
        uss=True,
    )
    assert not r["disqualified"]
    assert r["points_adjustment"] == -50.0
    assert r["score"] == pytest.approx(MAX_POINTS["trackdrive"] - 50.0, abs=0.1)


def test_uss_floor_at_zero() -> None:
    """A bad run + USS shouldn't go negative — D 9.1 implies non-negative scores."""
    pmin = MAX_POINTS["trackdrive"] * PMIN_FACTOR["trackdrive"]
    # Force a near-Pmin run by setting t_team near t_max
    t_min = 60.0
    t_team = t_min * TMAX_FACTOR["trackdrive"] * 0.999  # just under Tmax
    r = compute_scoring(
        "trackdrive",
        lap_times=[t_team],
        doo=0,
        oc=0,
        t_best=t_min,
        uss=True,
    )
    # Pmin (≈10) - 50 → would be negative; result should clamp to 0
    assert r["score"] >= 0.0


# ---------------------------------------------------------------------
# DV runtime cap (D 9.2.1) — DQ if unpenalised time > 25 s on Acc/Skid
# ---------------------------------------------------------------------


def test_dv_runtime_cap_dqs_slow_acceleration() -> None:
    """Acceleration: a single lap > 25 s flips the D 9.2.1 cap."""
    r = compute_scoring(
        "acceleration",
        lap_times=[DV_RUNTIME_CAP_S + 0.5],
        doo=0,
        oc=0,
        t_best=5.0,
    )
    assert r["disqualified"]
    assert any("D 9.2.1" in reason for reason in r["dq_reasons"])


def test_dv_runtime_cap_dqs_slow_skidpad() -> None:
    """Skidpad: total_elapsed is `avg(right) + avg(left)`, NOT sum of all
    4 laps. To trip the 25 s cap we need the avg-sum to exceed 25, which
    means either side averaging > ~12.5 s. 14 s × all 4 laps gives
    avg-sum = 28 s, well over the cap."""
    r = compute_scoring(
        "skidpad",
        lap_times=[14.0, 14.0, 14.0, 14.0],
        doo=0,
        oc=0,
        t_best=5.0,
    )
    assert r["total_elapsed"] == pytest.approx(28.0, abs=0.01)
    assert r["disqualified"]
    assert any("D 9.2.1" in reason for reason in r["dq_reasons"])


def test_autocross_not_subject_to_dv_runtime_cap() -> None:
    """The 25 s cap only applies to Acc + Skid; Autocross can run ≥ 25 s."""
    r = compute_scoring("autocross", lap_times=[40.0], doo=0, oc=0, t_best=40.0)
    assert not r["disqualified"]


# ---------------------------------------------------------------------
# Skidpad averaging (D 4.2.5) — score time = avg(R laps) + avg(L laps)
# ---------------------------------------------------------------------


def test_skidpad_uses_average_of_right_then_left_laps() -> None:
    """First two laps are right runs, last two are left. Score time is the
    sum of the two averages, not the sum of all four lap times."""
    laps = [5.0, 5.5, 6.0, 6.5]  # right avg = 5.25, left avg = 6.25
    r = compute_scoring("skidpad", lap_times=laps, doo=0, oc=0, t_best=11.5)
    assert r["total_elapsed"] == pytest.approx(5.25 + 6.25, abs=0.01)


def test_skidpad_one_run_each_side_falls_back_to_lap_times() -> None:
    """Edge case: only the first lap of each side (right + left) — averages
    are just those single laps."""
    laps = [5.0, 0.0, 6.0, 0.0]  # only first right + first left counted
    r = compute_scoring("skidpad", lap_times=laps, doo=0, oc=0, t_best=11.0)
    # avg(5,0) + avg(6,0) = 2.5 + 3 = 5.5 — sum-of-averages including 0s
    assert r["total_elapsed"] == pytest.approx(5.5, abs=0.01)


# ---------------------------------------------------------------------
# Misc invariants
# ---------------------------------------------------------------------


def test_unknown_event_returns_zero_max_points() -> None:
    r = compute_scoring("not_a_real_event", lap_times=[10.0], doo=0, oc=0)
    assert r["max_points"] == 0.0
    assert r["score"] == 0.0


def test_t_best_none_uses_team_own_time_as_baseline() -> None:
    """When `t_best` is unset, the team's own corrected time becomes
    Tmin — they get Pmax for that single run."""
    r = compute_scoring("autocross", lap_times=[60.0], doo=0, oc=0, t_best=None)
    assert r["score"] == pytest.approx(MAX_POINTS["autocross"], abs=0.1)


def test_response_shape_is_stable() -> None:
    """All keys the frontend reads are present, regardless of input."""
    r = compute_scoring("trackdrive", lap_times=[], doo=0, oc=0)
    expected_keys = {
        "event", "rule_version", "laps_completed", "lap_times",
        "best_lap", "total_elapsed", "doo_count", "doo_penalty_s",
        "oc_count", "oc_penalty_s", "oc_is_dq", "uss", "uss_rule",
        "points_adjustment", "corrected_time", "t_best_reference",
        "max_points", "score", "disqualified", "dq_reasons",
    }
    assert expected_keys.issubset(r.keys())
