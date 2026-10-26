"""Optional SE(3) joint refinement used after a closed-form initializer."""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares

from .se3 import exp, inverse, log


def residuals(robot_poses, observer_poses, robot_to_tool, world_to_observer):
    """Return one six-dimensional Lie-algebra residual per paired pose."""
    return np.array([
        log(inverse(robot @ robot_to_tool) @ (world_to_observer @ observer))
        for robot, observer in zip(robot_poses, observer_poses)
    ])


def refine(robot_poses, observer_poses, robot_to_tool, world_to_observer,
           translation_sigma_m: float = 0.002, rotation_sigma_deg: float = 0.2):
    """Refine two transforms with one soft-L1 loss per six-dimensional pose."""
    if translation_sigma_m <= 0 or rotation_sigma_deg <= 0:
        raise ValueError("noise_scales_must_be_positive")
    scale = np.array([translation_sigma_m] * 3 + [np.radians(rotation_sigma_deg)] * 3)

    def objective(delta):
        normalized = residuals(
            robot_poses,
            observer_poses,
            robot_to_tool @ exp(delta[:6]),
            world_to_observer @ exp(delta[6:]),
        ) / scale
        squared_norm = np.sum(normalized * normalized, axis=1)
        # sqrt(rho'(s)) for rho(s) = 2 * (sqrt(1 + s) - 1).
        weight = np.sqrt(2.0 / (np.sqrt(1.0 + squared_norm) + 1.0))
        return (normalized * weight[:, None]).ravel()

    fit = least_squares(objective, np.zeros(12), loss="linear", x_scale="jac", max_nfev=400)
    if not fit.success or not np.isfinite(fit.x).all():
        raise ValueError("optimizer_failure")
    refined_tool = robot_to_tool @ exp(fit.x[:6])
    refined_world = world_to_observer @ exp(fit.x[6:])
    return refined_tool, refined_world, {
        "cost": float(fit.cost),
        "function_evaluations": int(fit.nfev),
        "noise_scales": scale.tolist(),
        "residual_units": "metres, radians",
    }
