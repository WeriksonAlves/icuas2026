# src/planes/geometry.py
"""Geometry utilities for plane processing.

This module centralizes small, reusable geometric helpers used across
plane segmentation, tracking and analysis.

All functions are stateless and side-effect free.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def safe_acos(x: float) -> float:
    """Clamp x to [-1, 1] before calling :func:`math.acos`.

    This avoids NaNs caused by small numerical drift when the argument
    is slightly outside the valid range.
    """
    return math.acos(max(min(x, 1.0), -1.0))


def stabilize_normal(normal: np.ndarray) -> np.ndarray:
    """Enforce a simple normal orientation convention.

    Currently we force the z component to be non-negative. This makes
    plane normals more stable over time, which is important for
    temporal consistency and matching.
    """
    n = np.asarray(normal, dtype=np.float64)
    if n[2] < 0.0:
        n = -n
    return n


def refit_plane_svd(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """Refit a plane to points using SVD on the covariance matrix.

    The returned normal has unit norm and the plane is given in
    Hessian form ``n.T @ x + d = 0``. A deterministic orientation
    convention is enforced via :func:`stabilize_normal`.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")

    centroid = pts.mean(axis=0)
    centered = pts - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    norm = float(np.linalg.norm(normal))
    if norm == 0.0:
        raise ValueError("Cannot fit plane with zero normal")

    normal = stabilize_normal(normal / norm)
    d = -float(normal.dot(centroid))
    return normal, d


def point_plane_distances(
    points: np.ndarray,
    normal: np.ndarray,
    d: float,
) -> np.ndarray:
    """Signed distances from points to a plane in Hessian form.

    Parameters
    ----------
    points:
        Array of shape (N, 3) with 3D points.
    normal:
        Plane unit normal (3,).
    d:
        Plane offset such that ``normal.T @ x + d = 0``.

    Returns
    -------
    np.ndarray
        Array of shape (N,) with signed distances.
    """
    n = np.asarray(normal, dtype=np.float64)
    pts = np.asarray(points, dtype=np.float64)
    return pts @ n + float(d)


def plane_extent_pca_2d(
    points: np.ndarray,
    normal: np.ndarray,
) -> Tuple[float, float]:
    """Estimate planar extent using PCA in the local plane coordinates.

    1. Subtract the centroid.
    2. Project points onto the plane by removing the normal component.
    3. Run PCA (SVD) on the projected coordinates and take the two
       largest principal directions.
    4. Return the span (max - min) along each in-plane axis.
    """
    pts = np.asarray(points, dtype=np.float64)
    n = np.asarray(normal, dtype=np.float64)

    centroid = pts.mean(axis=0)
    centered = pts - centroid

    proj = centered - (centered @ n)[:, None] * n[None, :]
    _, _, vh = np.linalg.svd(proj, full_matrices=False)
    u = vh[0]
    v = vh[1]

    pu = proj @ u
    pv = proj @ v
    dx = float(pu.max() - pu.min())
    dy = float(pv.max() - pv.min())
    return dx, dy


def plane_extent_on_basis(
    points: np.ndarray,
    normal: np.ndarray,
) -> Tuple[float, float]:
    """Estimate planar extent on an orthonormal basis aligned with the plane.

    This is an alternative to :func:`plane_extent_pca_2d` that builds
    an orthonormal basis by choosing an arbitrary reference axis and
    orthogonalizing it against the normal.
    """
    pts = np.asarray(points, dtype=np.float64)
    n = np.asarray(normal, dtype=np.float64)

    # Choose a reference axis that is not parallel to the normal
    ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(float(n.dot(ref))) > 0.9:
        ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    u = np.cross(n, ref)
    u = u / (np.linalg.norm(u) + 1e-12)
    v = np.cross(n, u)
    v = v / (np.linalg.norm(v) + 1e-12)

    proj_u = pts @ u
    proj_v = pts @ v
    dx = float(proj_u.max() - proj_u.min())
    dy = float(proj_v.max() - proj_v.min())
    return dx, dy
