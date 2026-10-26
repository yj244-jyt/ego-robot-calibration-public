"""Public SE(3) reference utilities used by the calibration reports.

Device adapters, acquisition code, and private run data are intentionally omitted.
Transforms use column vectors, metres, right-handed frames, and xyzw quaternions.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def validate(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("invalid_transform")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError("invalid_homogeneous_row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7):
        raise ValueError("non_orthonormal_rotation")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-7):
        raise ValueError("reflection_or_non_rigid_rotation")
    return transform


def from_pose(position, quaternion_xyzw) -> np.ndarray:
    position = np.asarray(position, dtype=float)
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if position.shape != (3,) or quaternion.shape != (4,):
        raise ValueError("invalid_pose_shape")
    if not np.isfinite(np.r_[position, quaternion]).all():
        raise ValueError("non_finite_pose")
    if not np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-3):
        raise ValueError("non_unit_quaternion")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    transform[:3, 3] = position
    return transform


def inverse(transform: np.ndarray) -> np.ndarray:
    transform = validate(transform)
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def left_jacobian(phi: np.ndarray) -> np.ndarray:
    phi = np.asarray(phi, dtype=float)
    theta = np.linalg.norm(phi)
    cross = skew(phi)
    if theta < 1e-6:
        return np.eye(3) + (0.5 - theta**2 / 24) * cross + (1 / 6 - theta**2 / 120) * cross @ cross
    return np.eye(3) + (1 - np.cos(theta)) / theta**2 * cross + (theta - np.sin(theta)) / theta**3 * cross @ cross


def exp(twist: np.ndarray) -> np.ndarray:
    twist = np.asarray(twist, dtype=float)
    if twist.shape != (6,) or not np.isfinite(twist).all():
        raise ValueError("invalid_twist")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(twist[3:]).as_matrix()
    transform[:3, 3] = left_jacobian(twist[3:]) @ twist[:3]
    return transform


def log(transform: np.ndarray) -> np.ndarray:
    transform = validate(transform)
    phi = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    return np.r_[np.linalg.solve(left_jacobian(phi), transform[:3, 3]), phi]


def angle_deg(rotation: np.ndarray) -> float:
    return float(np.degrees(Rotation.from_matrix(rotation).magnitude()))
