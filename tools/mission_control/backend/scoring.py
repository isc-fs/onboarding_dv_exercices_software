"""
FS Rules 2026 — Dynamic Disciplines Scoring (D 9) and Penalties (D 10).

Single-run scoring only. Multi-run aggregation (best-of-2 for DV Acceleration /
Skidpad, min/avg for DC Autocross) is the caller's responsibility — the
referee only tracks one run at a time.

USS (Unsafe Stop, D 10.1.7) is accepted as an input flag; actual detection
(did the car stop in the Stop Area? orientation? finish-state within 30 s?)
belongs in the referee and is not implemented yet.
"""

from typing import Optional, List, Union

# Pmax per discipline (Table 3 — DV column, DC-only where specified)
MAX_POINTS = {
    "acceleration": 75.0,   # DV
    "skidpad": 75.0,        # DV
    "autocross": 100.0,     # DV; DC-Autocross is 250 pts and uses a different formula
    "trackdrive": 200.0,    # DC only
}

# Table 11: Tmax is a multiple of Tmin; Pmin is a fraction of Pmax
TMAX_FACTOR = {
    "acceleration": 1.4,
    "skidpad": 1.7,
    "autocross": 1.5,
    "trackdrive": 1.35,     # D 9.1.1 + default table row
}

PMIN_FACTOR = {
    "acceleration": 0.1,
    "skidpad": 0.05,
    "autocross": 0.1,
    "trackdrive": 0.05,
}

# D 10.1: time penalties in seconds
DOO_PENALTY_S = {
    "acceleration": 2.0,
    "skidpad": 0.2,
    "autocross": 2.0,
    "trackdrive": 2.0,
}

# OC action per discipline: float = seconds penalty; "dq" = disqualification
OC_PENALTY: dict = {
    "acceleration": "dq",
    "skidpad": "dq",
    "autocross": 10.0,
    "trackdrive": 10.0,
}

# USS action: "dq" | "-50pts" | "n/a"
USS_PENALTY = {
    "acceleration": "dq",
    "skidpad": "dq",
    "autocross": "dq",
    "trackdrive": "-50pts",
}

# D 9.2.1: DV Skidpad/Acceleration runs with unpenalised time > 25 s are DQ'd
DV_RUNTIME_CAP_S = 25.0


def _score_formula(t_team: float, t_min: float, t_max: float,
                   p_max: float, p_min: float) -> float:
    """D 9.1.1: SCORE = (Pmax - Pmin) * ((Tmax - Tteam)/(Tmax - Tmin))^2 + Pmin."""
    if t_max <= t_min or t_team >= t_max:
        return p_min
    ratio = (t_max - t_team) / (t_max - t_min)
    return (p_max - p_min) * ratio * ratio + p_min


def _skidpad_elapsed(lap_times: List[float]) -> float:
    """Skidpad (D 4.2.5): 4 laps, first 2 right, last 2 left. Score time is
    the sum of the right and left averages."""
    right = lap_times[:2]
    left = lap_times[2:4]
    avg_right = sum(right) / len(right) if right else 0.0
    avg_left = sum(left) / len(left) if left else 0.0
    return avg_right + avg_left


def compute_scoring(
    event: str,
    lap_times: List[float],
    doo: int,
    oc: int,
    t_best: Optional[float] = None,
    uss: bool = False,
) -> dict:
    """Compute the score and DQ state for a single run.

    Args:
        event: one of "acceleration", "skidpad", "autocross", "trackdrive"
        lap_times: lap times in seconds (may be empty if the run didn't complete)
        doo: number of Down-or-Out cones on this run
        oc: number of Off-Course events on this run
        t_best: the fastest team's corrected time (Tmin). If None, use this
                run's own corrected time — the result becomes a baseline score.
        uss: True if the run ended with an Unsafe Stop (see USS_PENALTY).
    """
    event = event.lower()
    max_points = MAX_POINTS.get(event, 0.0)

    # Aggregate elapsed time according to event format
    if event == "skidpad":
        total_elapsed = _skidpad_elapsed(lap_times)
    else:
        total_elapsed = sum(lap_times) if lap_times else 0.0

    # DOO is always a time penalty
    doo_s = doo * DOO_PENALTY_S.get(event, 0.0)
    t_corrected = total_elapsed + doo_s

    dq_reasons: List[str] = []
    points_adjustment = 0.0

    # OC: DQ for Acc/Skid, +10 s elsewhere
    oc_rule = OC_PENALTY.get(event)
    oc_s = 0.0
    if oc > 0:
        if oc_rule == "dq":
            dq_reasons.append(f"OC × {oc} → DQ ({event})")
        elif isinstance(oc_rule, (int, float)):
            oc_s = oc * float(oc_rule)
            t_corrected += oc_s

    # USS: DQ or -50 pts; n/a is absent here (no Endurance) — defensive default is n/a
    uss_rule = USS_PENALTY.get(event, "n/a")
    if uss:
        if uss_rule == "dq":
            dq_reasons.append("USS → DQ")
        elif uss_rule == "-50pts":
            points_adjustment -= 50.0

    # D 9.2.1: DV Acceleration/Skidpad — unpenalised time > 25 s is a DQ
    if event in ("acceleration", "skidpad") and total_elapsed > DV_RUNTIME_CAP_S:
        dq_reasons.append(f"Unpenalised time {total_elapsed:.2f} s > 25 s (D 9.2.1) → DQ")

    disqualified = bool(dq_reasons)

    # D 9.1.1 scoring
    if disqualified:
        score = 0.0
    else:
        t_min = t_best if (t_best and t_best > 0) else t_corrected
        t_max = t_min * TMAX_FACTOR.get(event, 1.5)
        p_min = max_points * PMIN_FACTOR.get(event, 0.1)
        score = _score_formula(t_corrected, t_min, t_max, max_points, p_min)
        score = max(0.0, score + points_adjustment)

    return {
        "event": event,
        "rule_version": "FS-Rules_2026_v1.1",
        "laps_completed": len(lap_times),
        "lap_times": lap_times,
        "best_lap": round(min(lap_times), 3) if lap_times else 0.0,
        "total_elapsed": round(total_elapsed, 3),
        "doo_count": doo,
        "doo_penalty_s": round(doo_s, 2),
        "oc_count": oc,
        "oc_penalty_s": round(oc_s, 2),
        "oc_is_dq": (oc > 0 and oc_rule == "dq"),
        "uss": uss,
        "uss_rule": uss_rule,
        "points_adjustment": points_adjustment,
        "corrected_time": round(t_corrected, 3),
        "t_best_reference": round(t_best, 3) if t_best else None,
        "max_points": max_points,
        "score": round(score, 1),
        "disqualified": disqualified,
        "dq_reasons": dq_reasons,
    }
