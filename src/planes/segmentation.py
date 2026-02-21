# src/planes/segmentation.py
"""Plane segmentation and coarse semantic classification utilities.

This module provides a reusable API for multi-plane segmentation using
Open3D RANSAC, along with simple heuristics to classify floor-like,
table-like and wall-like planes.

It is designed to be imported by CLI scripts and notebooks, keeping
all heavy logic and data structures in one place.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import open3d as o3d

from planes.geometry import (
    safe_acos,
    point_plane_distances,
    refit_plane_svd,
    plane_extent_on_basis,
)


@dataclass(frozen=True)
class PlaneModel:
    """Plane in Hessian form ``n.T @ x + d = 0`` with ``||n|| = 1``."""

    n: List[float]  # [nx, ny, nz]
    d: float


@dataclass(frozen=True)
class PlaneMetrics:
    """Per-plane statistics computed after RANSAC and refitting."""

    plane_id: int
    num_inliers: int
    inlier_ratio: float           # ratio w.r.t. remaining points
    inlier_ratio_total: float     # ratio w.r.t. all points in the frame
    rmse: float
    mean_abs_dist: float
    std_abs_dist: float
    extent_xy: List[float]        # [dx, dy] on plane basis
    centroid: List[float]         # [x, y, z]
    model: PlaneModel


@dataclass(frozen=True)
class SegConfig:
    """Configuration parameters for plane segmentation."""

    input_path: Path
    out_dir: Path
    preset: str
    max_planes: int
    min_inliers: int
    distance_threshold: float
    ransac_n: int
    num_iterations: int
    sor_nb_neighbors: int
    sor_std_ratio: float
    seed: int

    @classmethod
    def from_cli_args(cls, args) -> "SegConfig":
        """Build a configuration from an argparse namespace.

        This keeps argparse-related concerns outside of this module
        while still offering a convenient factory for CLI front-ends.
        """
        defaults = _preset_defaults(args.preset)

        dist_th = (
            float(args.distance_threshold)
            if args.distance_threshold > 0
            else float(defaults["distance_threshold"])
        )
        min_inl = (
            int(args.min_inliers)
            if args.min_inliers > 0
            else int(defaults["min_inliers"])
        )
        sor_std = (
            float(args.sor_std_ratio)
            if args.sor_std_ratio > 0
            else float(defaults["sor_std_ratio"])
        )

        return cls(
            input_path=Path(os.path.expanduser(args.input)).resolve(),
            out_dir=Path(os.path.expanduser(args.out_dir)).resolve(),
            preset=args.preset,
            max_planes=max(1, int(args.max_planes)),
            min_inliers=max(50, min_inl),
            distance_threshold=max(1e-6, dist_th),
            ransac_n=max(3, int(args.ransac_n)),
            num_iterations=max(100, int(args.num_iterations)),
            sor_nb_neighbors=max(5, int(args.sor_nb_neighbors)),
            sor_std_ratio=max(0.1, sor_std),
            seed=int(args.seed),
        )


def _preset_defaults(preset: str) -> Dict[str, float]:
    """Return default parameters for a given preset name."""
    preset = preset.lower().strip()
    if preset == "indoor":
        return {
            "distance_threshold": 0.03,
            "min_inliers": 1500,
            "sor_std_ratio": 2.0,
        }
    if preset == "outdoor":
        return {
            "distance_threshold": 0.06,
            "min_inliers": 1200,
            "sor_std_ratio": 2.5,
        }
    raise ValueError(f"Unknown preset: {preset}")


# ---------------------------------------------------------------------------
# High-level semantic classification: floor / table / main wall
# ---------------------------------------------------------------------------


def _estimate_vertical_axis(planes: List[PlaneMetrics]) -> np.ndarray:
    """Estimate a dominant vertical axis from plane normals.

    We assume that either floor/ceiling or walls are large and their
    normals will cluster around one of the canonical axes (x, y, z).
    """
    if not planes:
        raise ValueError("Cannot estimate vertical axis with no planes.")

    normals = np.array([p.model.n for p in planes], dtype=np.float64)
    weights = np.array(
        [p.inlier_ratio_total for p in planes], dtype=np.float64
    )

    axes = np.eye(3, dtype=np.float64)  # ex, ey, ez
    # scores[i, k] = |n_i . e_k| * weight_i
    scores = np.abs(normals @ axes) * weights[:, None]
    total_scores = scores.sum(axis=0)

    best_axis_idx = int(np.argmax(total_scores))
    return axes[best_axis_idx]


def classify_floor_and_wall(
    planes: List[PlaneMetrics],
    ground_angle_deg: float = 25.0,
    wall_angle_deg: float = 65.0,
    min_table_area_ratio: float = 0.05,
    max_table_area_ratio: float = 0.8,
    min_height_gap: float = 0.10,
) -> Dict[str, object]:
    """Classify floor-like, table-like and wall-like planes.

    The classification is based on:
    - angle to the estimated vertical axis;
    - plane area and height (for distinguishing floor vs. table);
    - inlier ratios as a proxy for support.

    Returns
    -------
    dict
        Dictionary with keys:

        - ``vertical_axis``: [vx, vy, vz] or None
        - ``floor_plane_id``: int or None
        - ``table_plane_id``: int or None
        - ``main_wall_plane_id``: int or None
    """
    if not planes:
        return {
            "vertical_axis": None,
            "floor_plane_id": None,
            "table_plane_id": None,
            "main_wall_plane_id": None,
        }

    v = _estimate_vertical_axis(planes)
    v_norm = np.linalg.norm(v)
    if v_norm < 1e-9:
        return {
            "vertical_axis": None,
            "floor_plane_id": None,
            "table_plane_id": None,
            "main_wall_plane_id": None,
        }

    horizontal_planes = []  # (plane, angle_deg, height, area, weight)
    vertical_planes = []    # (plane, angle_deg, weight)

    for p in planes:
        n = np.array(p.model.n, dtype=np.float64)
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-9:
            continue

        cos_theta = abs(float(n.dot(v)) / (n_norm * v_norm))
        theta_deg = math.degrees(safe_acos(cos_theta))

        c = np.array(p.centroid, dtype=np.float64)
        height = float(v.dot(c))
        dx, dy = p.extent_xy
        area = float(dx * dy)
        weight = p.inlier_ratio_total

        if theta_deg <= ground_angle_deg:
            horizontal_planes.append((p, theta_deg, height, area, weight))
        if theta_deg >= wall_angle_deg:
            vertical_planes.append((p, theta_deg, weight))

    # Floor: largest horizontal plane by area * weight
    floor_plane_id: int | None = None
    floor_area = 0.0
    floor_height = 0.0

    if horizontal_planes:
        best_score = -1.0
        for p, _, h, a, w in horizontal_planes:
            score = a * w
            if score > best_score:
                best_score = score
                floor_plane_id = p.plane_id
                floor_area = a
                floor_height = h

    # Table: horizontal plane above floor with intermediate area
    table_plane_id: int | None = None
    if floor_plane_id is not None and floor_area > 0.0:
        for p, _, h, a, w in horizontal_planes:
            if p.plane_id == floor_plane_id:
                continue
            area_ratio = a / floor_area
            height_gap = h - floor_height
            if (
                min_table_area_ratio <= area_ratio <= max_table_area_ratio
                and height_gap >= min_height_gap
                and w > 0.1
            ):
                table_plane_id = p.plane_id
                break

    # Main wall: vertical plane with largest inlier_ratio_total
    main_wall_plane_id: int | None = None
    if vertical_planes:
        best_score = -1.0
        for p, _, w in vertical_planes:
            if w > best_score:
                best_score = w
                main_wall_plane_id = p.plane_id

    return {
        "vertical_axis": v.tolist(),
        "floor_plane_id": floor_plane_id,
        "table_plane_id": table_plane_id,
        "main_wall_plane_id": main_wall_plane_id,
    }


# ---------------------------------------------------------------------------
# Core segmentation logic
# ---------------------------------------------------------------------------


def _apply_sor(
    pcd: o3d.geometry.PointCloud,
    nb_neighbors: int,
    std_ratio: float,
) -> o3d.geometry.PointCloud:
    """Apply statistical outlier removal for a lightweight cleanup."""
    if len(pcd.points) < nb_neighbors:
        return pcd
    filtered, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )
    return filtered


def _color_for_plane_id(
    plane_id: int,
    rng: np.random.Generator,
) -> np.ndarray:
    """Return an RGB color for a given plane id.

    The first few ids get fixed, semantically meaningful colors to keep
    visualizations consistent across runs. Remaining ids are assigned
    bright random colors.
    """
    if plane_id == 0:
        # Floor-like
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)  # red
    if plane_id == 1:
        # Table-like
        return np.array([0.0, 1.0, 0.0], dtype=np.float64)  # green
    if plane_id == 2:
        # Main wall
        return np.array([0.0, 0.0, 1.0], dtype=np.float64)  # blue

    # Random but reasonably bright color
    rgb = rng.random(3)
    rgb = 0.3 + 0.7 * rgb
    return rgb.astype(np.float64)


def segment_planes(
    pcd: o3d.geometry.PointCloud,
    cfg: SegConfig,
    rng: np.random.Generator,
) -> Tuple[List[PlaneMetrics], o3d.geometry.PointCloud]:
    """Iteratively extract planes using Open3D RANSAC.

    The implementation uses NumPy masks to keep a stable mapping
    between the RANSAC inliers and the original point indices.

    Returns
    -------
    list of PlaneMetrics
        Per-plane statistics and fitted models.
    o3d.geometry.PointCloud
        Full point cloud with RGB colors encoding plane ids
        (unassigned points are colored in gray).
    """
    pts_all = np.asarray(pcd.points)
    if pts_all.shape[0] < max(50, cfg.min_inliers):
        colored = o3d.geometry.PointCloud(pcd)
        colors = np.full((pts_all.shape[0], 3), 0.5, dtype=np.float64)
        colored.colors = o3d.utility.Vector3dVector(colors)
        return [], colored

    labels = np.full((pts_all.shape[0],), -1, dtype=np.int32)
    metrics: List[PlaneMetrics] = []

    remaining_mask = np.ones((pts_all.shape[0],), dtype=bool)

    rem_pcd = o3d.geometry.PointCloud()

    for plane_id in range(cfg.max_planes):
        remaining_idx = np.flatnonzero(remaining_mask)
        if remaining_idx.size < cfg.min_inliers:
            break
        if remaining_idx.size / pts_all.shape[0] < 0.05:
            break

        remaining_pts = pts_all[remaining_idx]
        rem_pcd.clear()
        rem_pcd.points = o3d.utility.Vector3dVector(remaining_pts)

        model, inliers_local = rem_pcd.segment_plane(
            distance_threshold=cfg.distance_threshold,
            ransac_n=cfg.ransac_n,
            num_iterations=cfg.num_iterations,
        )

        if len(inliers_local) < cfg.min_inliers:
            break

        inliers_local = np.array(inliers_local, dtype=np.int64)
        inliers_global = remaining_idx[inliers_local]
        inlier_pts = pts_all[inliers_global]

        # Refit with SVD for a more stable model
        n_refit, d_refit = refit_plane_svd(inlier_pts)

        # Distance statistics
        signed = point_plane_distances(inlier_pts, n_refit, d_refit)
        absd = np.abs(signed)

        rmse = float(math.sqrt(float(np.mean(absd ** 2))))
        mean_abs = float(np.mean(absd))
        std_abs = float(np.std(absd))

        centroid = inlier_pts.mean(axis=0).astype(float).tolist()
        dx, dy = plane_extent_on_basis(inlier_pts, n_refit)
        extent_xy = [dx, dy]

        inlier_ratio_remaining = float(
            inliers_global.size / max(1, remaining_idx.size)
        )
        inlier_ratio_total = float(
            inliers_global.size / max(1, pts_all.shape[0])
        )

        metrics.append(
            PlaneMetrics(
                plane_id=int(plane_id),
                num_inliers=int(inliers_global.size),
                inlier_ratio=inlier_ratio_remaining,
                inlier_ratio_total=inlier_ratio_total,
                rmse=rmse,
                mean_abs_dist=mean_abs,
                std_abs_dist=std_abs,
                extent_xy=extent_xy,
                centroid=centroid,
                model=PlaneModel(
                    n=n_refit.astype(float).tolist(),
                    d=float(d_refit),
                ),
            )
        )

        labels[inliers_global] = plane_id
        remaining_mask[inliers_global] = False

    # Build colored cloud
    colors = np.full(
        (pts_all.shape[0], 3),
        0.5,
        dtype=np.float64,
    )  # gray for unassigned
    for m in metrics:
        pid = m.plane_id
        colors[labels == pid] = _color_for_plane_id(pid, rng)

    colored = o3d.geometry.PointCloud()
    colored.points = o3d.utility.Vector3dVector(pts_all)
    colored.colors = o3d.utility.Vector3dVector(colors)
    return metrics, colored


# ---------------------------------------------------------------------------
# I/O helpers and high-level orchestration
# ---------------------------------------------------------------------------


def load_pcd(path: Path) -> o3d.geometry.PointCloud:
    """Load a point cloud from a .ply file and validate it."""
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise ValueError(f"Empty point cloud: {path}")
    return pcd


def list_frames(input_path: Path) -> List[Path]:
    """Return a sorted list of frame paths from a file or directory."""
    if input_path.is_dir():
        frames = sorted(input_path.glob("*.ply"))
        if not frames:
            raise FileNotFoundError(f"No .ply files found in: {input_path}")
        return frames
    if input_path.is_file():
        return [input_path]
    raise FileNotFoundError(f"Input path not found: {input_path}")


def write_json(path: Path, data: Dict) -> None:
    """Write JSON data to disk, creating parent directories if needed."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def process_frame(
    frame_path: Path,
    frame_idx: int,
    cfg: SegConfig,
) -> Dict:
    """Process a single frame: load, filter, segment and classify."""
    rng = np.random.default_rng(cfg.seed + frame_idx)

    pcd = load_pcd(frame_path)
    pcd = _apply_sor(
        pcd,
        nb_neighbors=cfg.sor_nb_neighbors,
        std_ratio=cfg.sor_std_ratio,
    )

    metrics, colored = segment_planes(pcd, cfg, rng)
    classification = classify_floor_and_wall(metrics)

    cfg.out_dir.mkdir(parents=True, exist_ok=True)

    out_ply = cfg.out_dir / f"colored_planes_{frame_idx:05d}.ply"
    o3d.io.write_point_cloud(
        str(out_ply),
        colored,
        write_ascii=False,
        compressed=False,
    )

    out_json = cfg.out_dir / f"planes_{frame_idx:05d}.json"
    num_points = int(np.asarray(pcd.points).shape[0])

    payload = {
        "frame_idx": frame_idx,
        "input_file": str(frame_path),
        "preset": cfg.preset,
        "params": {
            "max_planes": cfg.max_planes,
            "min_inliers": cfg.min_inliers,
            "distance_threshold": cfg.distance_threshold,
            "ransac_n": cfg.ransac_n,
            "num_iterations": cfg.num_iterations,
            "sor_nb_neighbors": cfg.sor_nb_neighbors,
            "sor_std_ratio": cfg.sor_std_ratio,
        },
        "num_points": num_points,
        "planes": [asdict(m) for m in metrics],
        "classification": classification,
    }
    write_json(out_json, payload)

    return {
        "frame_idx": frame_idx,
        "input_file": str(frame_path),
        "num_points": payload["num_points"],
        "num_planes": len(metrics),
        "planes": payload["planes"],
        "colored_ply": str(out_ply),
        "planes_json": str(out_json),
    }


def run_segmentation(cfg: SegConfig) -> Dict:
    """Run plane segmentation on all frames described by *cfg*.

    This function is the main entry point for non-CLI callers. It
    returns the same summary dictionary that the CLI writes to
    ``summary.json`` so that tests and notebooks can inspect it
    directly.
    """
    frames = list_frames(cfg.input_path)

    results: List[Dict] = []
    for i, frame in enumerate(frames):
        res = process_frame(frame, i, cfg)
        results.append(res)

    summary = {
        "input": str(cfg.input_path),
        "out_dir": str(cfg.out_dir),
        "preset": cfg.preset,
        "num_frames": len(frames),
        "processed_frames": len(results),
        "frames": results,
    }
    write_json(cfg.out_dir / "summary.json", summary)
    return summary
