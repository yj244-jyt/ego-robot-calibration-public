"""Device-independent Park-Martin hand-eye initialization.

Given A_ij X = X B_ij, this module solves a rotation first and a translation
second. It contains no device names, adapters, or data-loading assumptions.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .se3 import inverse, validate


def select_motion_pairs(robot_poses, observer_poses, min_rotation_deg: float = 3.0):
    if len(robot_poses) != len(observer_poses):
        raise ValueError("pose_count_mismatch")
    count = len(robot_poses)
    offsets = (1, max(2, count // 3), max(3, count // 2))
    pairs = []
    for offset in offsets:
        for i in range(max(0, count - offset)):
            j = i + offset
            a = inverse(robot_poses[i]) @ robot_poses[j]
            b = inverse(observer_poses[i]) @ observer_poses[j]
            alpha = Rotation.from_matrix(a[:3, :3]).as_rotvec()
            beta = Rotation.from_matrix(b[:3, :3]).as_rotvec()
            if min(np.linalg.norm(alpha), np.linalg.norm(beta)) >= np.radians(min_rotation_deg):
                pairs.append((a, b, alpha, beta))
    return pairs


def initialize(robot_poses, observer_poses, min_poses: int = 6, max_condition: float = 1e6):
    robot_poses = np.asarray(robot_poses, dtype=float)
    observer_poses = np.asarray(observer_poses, dtype=float)
    if len(robot_poses) != len(observer_poses) or len(robot_poses) < min_poses:
        raise ValueError("insufficient_pose_count")
    for transform in [*robot_poses, *observer_poses]:
        validate(transform)
    pairs = select_motion_pairs(robot_poses, observer_poses)
    if len(pairs) < 3:
        raise ValueError("insufficient_rotational_excitation")

    alpha = np.array([pair[2] for pair in pairs])
    beta = np.array([pair[3] for pair in pairs])
    singular = np.linalg.svd(alpha, compute_uv=False)
    if singular[1] < max(1e-8, singular[0] / max_condition):
        raise ValueError("rank_deficient_rotation_excitation")

    u, _, vt = np.linalg.svd(alpha.T @ beta)
    rotation = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    left = np.vstack([a[:3, :3] - np.eye(3) for a, _, _, _ in pairs])
    right = np.concatenate([rotation @ b[:3, 3] - a[:3, 3] for a, b, _, _ in pairs])
    translation, _, rank, translation_singular = np.linalg.lstsq(left, right, rcond=None)
    condition = float(translation_singular[0] / translation_singular[-1]) if translation_singular[-1] > 0 else float("inf")
    if rank < 3 or condition > max_condition:
        raise ValueError("rank_or_condition_failure")

    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform, {
        "method": "park_martin",
        "motion_pairs": len(pairs),
        "rotation_singular_values": singular.tolist(),
        "translation_singular_values": translation_singular.tolist(),
        "translation_condition": condition,
    }
