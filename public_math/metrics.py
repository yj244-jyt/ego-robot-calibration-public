"""Public metric and confidence helpers for report-level evaluation."""

from __future__ import annotations

import numpy as np


def summary_stats(values) -> dict[str, float]:
    values = np.asarray(values, dtype=float)
    if values.size == 0 or not np.isfinite(values).all():
        raise ValueError("empty_or_nonfinite_metrics")
    return {
        "mean": float(values.mean()),
        "mae": float(np.abs(values).mean()),
        "rmse": float(np.sqrt(np.mean(values**2))),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "maximum": float(values.max()),
    }


def pose_error(robot_position_m, observer_position_m) -> float:
    """Euclidean translation error in millimetres."""
    delta = np.asarray(observer_position_m, dtype=float) - np.asarray(robot_position_m, dtype=float)
    return float(np.linalg.norm(delta) * 1000.0)


def coverage_ratio(valid, candidate) -> float:
    """Effective coverage C = N_valid / N_candidate."""
    if candidate < 0 or valid < 0 or valid > candidate:
        raise ValueError("invalid_coverage_counts")
    return float(valid / candidate) if candidate else 0.0


def bootstrap_metric_ci(values, groups=None, samples: int = 1000, seed: int = 51) -> dict:
    """Group bootstrap for RMSE and P95; a frame is not an independent trial."""
    values = np.asarray(values, dtype=float)
    if values.ndim != 1 or not len(values) or not np.isfinite(values).all():
        raise ValueError("invalid_bootstrap_values")
    groups = np.arange(len(values)) if groups is None else np.asarray(groups)
    if len(groups) != len(values):
        raise ValueError("bootstrap_group_length_mismatch")
    units = np.unique(groups)
    if len(units) < 2:
        raise ValueError("bootstrap_requires_two_independent_groups")
    rng = np.random.default_rng(seed)
    estimates = []
    for _ in range(samples):
        picked = rng.choice(units, size=len(units), replace=True)
        indices = np.concatenate([np.flatnonzero(groups == group) for group in picked])
        sample = values[indices]
        estimates.append([np.sqrt(np.mean(sample**2)), np.percentile(sample, 95)])
    return {
        "unit": "independent_group",
        "groups": int(len(units)),
        "seed": int(seed),
        "samples": int(samples),
        "columns": ["rmse", "p95"],
        "ci95": np.percentile(estimates, [2.5, 97.5], axis=0).tolist(),
    }


def wilson_interval(successes: int, trials: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson interval for a coverage/pass fraction, returned as proportions."""
    if trials <= 0 or successes < 0 or successes > trials:
        raise ValueError("invalid_binomial_counts")
    p = successes / trials
    denominator = 1 + z**2 / trials
    center = (p + z**2 / (2 * trials)) / denominator
    radius = z * np.sqrt(p * (1 - p) / trials + z**2 / (4 * trials**2)) / denominator
    return float(max(0.0, center - radius)), float(min(1.0, center + radius))


def first_order_position_budget(ego_sigma_m, robot_sigma_m, speed_mps, time_sigma_s,
                                lever_m, angle_sigma_rad, fixture_sigma_m, calibration_sigma_m) -> dict:
    """Independent first-order terms; values are sigma-like, not P95 errors."""
    values = np.asarray([
        ego_sigma_m, robot_sigma_m, speed_mps, time_sigma_s,
        lever_m, angle_sigma_rad, fixture_sigma_m, calibration_sigma_m,
    ], dtype=float)
    if not np.isfinite(values).all() or np.any(values < 0):
        raise ValueError("invalid_uncertainty_budget")
    terms = {
        "ego": float(ego_sigma_m),
        "robot": float(robot_sigma_m),
        "timing": float(speed_mps * time_sigma_s),
        "lever_angle": float(lever_m * angle_sigma_rad),
        "fixture": float(fixture_sigma_m),
        "calibration": float(calibration_sigma_m),
    }
    return {"terms_m": terms, "combined_sigma_m": float(np.linalg.norm(list(terms.values())))}
