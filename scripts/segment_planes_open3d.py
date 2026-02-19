# scripts/segment_planes_open3d.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-plane segmentation on point clouds using Open3D (iterative RANSAC).

Outputs (per frame):
- colored_planes_XXXXX.ply: points colored by plane ID
- planes_XXXXX.json: plane parameters and metrics

Outputs (global):
- summary.json: aggregated info across frames

Designed for a fast MVP (ZED-only now, easy to reuse for LiDAR later).
"""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import open3d as o3d


@dataclass(frozen=True)
class PlaneModel:
    """
    Plane in Hessian form:
      n^T x + d = 0, where ||n|| = 1
    """
    n: List[float]  # [nx, ny, nz]
    d: float


@dataclass(frozen=True)
class PlaneMetrics:
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


def _safe_acos(x: float) -> float:
    """Clamp x to [-1, 1] before calling acos, to avoid NaNs."""
    return math.acos(max(min(x, 1.0), -1.0))


def _estimate_vertical_axis(planes: List[PlaneMetrics]) -> np.ndarray:
    """
    Estimate a dominant 'vertical' axis from plane normals.

    We assume that either floor/ceiling or walls are large and
    their normals will cluster around one canonical axis (x, y or z).
    """
    if not planes:
        raise ValueError("Cannot estimate vertical axis with no planes.")

    normals = np.array([p.model.n for p in planes], dtype=np.float64)
    weights = np.array([p.inlier_ratio_total for p in planes], dtype=np.float64)

    axes = np.eye(3, dtype=np.float64)  # ex, ey, ez
    # scores[i, k] = |n_i . e_k| * weight_i
    scores = np.abs(normals @ axes) * weights[:, None]
    total_scores = scores.sum(axis=0)

    best_axis_idx = int(np.argmax(total_scores))
    return axes[best_axis_idx]


def _classify_floor_and_wall(
    planes: List[PlaneMetrics],
    ground_angle_deg: float = 25.0,
    wall_angle_deg: float = 65.0,
) -> Dict[str, object]:
    """
    Classify floor-like (horizontal) and main wall (vertical) planes.

    Returns a dict with:
      - vertical_axis: [vx, vy, vz]
      - floor_plane_id: int or None
      - main_wall_plane_id: int or None
    """
    if not planes:
        return {
            "vertical_axis": None,
            "floor_plane_id": None,
            "main_wall_plane_id": None,
        }

    v = _estimate_vertical_axis(planes)
    v_norm = np.linalg.norm(v)

    best_floor_id: int | None = None
    best_floor_score = -1.0

    best_wall_id: int | None = None
    best_wall_score = -1.0

    for p in planes:
        n = np.array(p.model.n, dtype=np.float64)
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-9 or v_norm < 1e-9:
            continue

        cos_theta = abs(float(n.dot(v)) / (n_norm * v_norm))
        theta_deg = math.degrees(_safe_acos(cos_theta))

        # Horizontal candidate (floor/table)
        if theta_deg <= ground_angle_deg:
            score = p.inlier_ratio_total
            if score > best_floor_score:
                best_floor_score = score
                best_floor_id = p.plane_id

        # Vertical candidate (wall)
        if theta_deg >= wall_angle_deg:
            score = p.inlier_ratio_total
            if score > best_wall_score:
                best_wall_score = score
                best_wall_id = p.plane_id

    return {
        "vertical_axis": v.tolist(),
        "floor_plane_id": best_floor_id,
        "main_wall_plane_id": best_wall_id,
    }


def _unit_normal_and_d(a: float, b: float, c: float, d: float) -> Tuple[np.ndarray, float]:
    """Normalize plane coefficients (a,b,c,d) so that ||n||=1."""
    n = np.array([a, b, c], dtype=np.float64)
    norm = np.linalg.norm(n)
    if norm < 1e-12:
        raise ValueError("Degenerate plane normal (near-zero norm).")
    n_unit = n / norm
    d_unit = float(d / norm)
    return n_unit, d_unit


def _refit_plane_svd(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Refit plane using SVD (least squares):
      - normal is the smallest singular vector
      - plane passes through centroid
    """
    if points.shape[0] < 3:
        raise ValueError("Need at least 3 points to fit a plane.")
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    n = vh[-1, :]
    n = n / (np.linalg.norm(n) + 1e-12)
    d = -float(n.dot(centroid))
    return n, d


def _point_plane_distances(points: np.ndarray, n: np.ndarray, d: float) -> np.ndarray:
    """Signed distance of points to plane n^T x + d = 0."""
    return points @ n + d


def _plane_extent_on_basis(points: np.ndarray, n: np.ndarray) -> List[float]:
    """
    Approximate plane extent by projecting points onto an orthonormal basis (u,v)
    spanning the plane. Returns [dx, dy] as ranges along u and v.
    """
    ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(float(n.dot(ref))) > 0.9:
        ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    u = np.cross(n, ref)
    u = u / (np.linalg.norm(u) + 1e-12)
    v = np.cross(n, u)
    v = v / (np.linalg.norm(v) + 1e-12)

    proj_u = points @ u
    proj_v = points @ v
    dx = float(proj_u.max() - proj_u.min())
    dy = float(proj_v.max() - proj_v.min())
    return [dx, dy]


def _apply_sor(pcd: o3d.geometry.PointCloud, nb_neighbors: int, std_ratio: float) -> o3d.geometry.PointCloud:
    """Statistical outlier removal (lightweight cleanup)."""
    if len(pcd.points) < nb_neighbors:
        return pcd
    filtered, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return filtered


def _color_for_plane_id(plane_id: int, rng: np.random.Generator) -> np.ndarray:
    """
    Deterministic-ish color per plane. We use RNG seeded per frame for reproducibility.
    """
    c = rng.random(3)
    c = 0.15 + 0.85 * c  # avoid too dark colors
    return c


def _stabilize_normal(n: np.ndarray) -> np.ndarray:
    # Convention: keep normal pointing to +Z when possible
    if n[2] < 0.0:
        return -n
    return n


def _plane_extent_pca_2d(points: np.ndarray, n: np.ndarray) -> List[float]:
    # Build orthonormal basis (u, v) deterministically from PCA on projected points
    centroid = points.mean(axis=0)
    centered = points - centroid

    # remove normal component -> project to plane
    proj = centered - (centered @ n)[:, None] * n[None, :]

    # PCA on projected coordinates in 3D gives two principal directions in-plane
    _, _, vh = np.linalg.svd(proj, full_matrices=False)
    u = vh[0]
    v = vh[1]

    pu = proj @ u
    pv = proj @ v
    dx = float(pu.max() - pu.min())
    dy = float(pv.max() - pv.min())
    return [dx, dy]


def _segment_planes_iterative_numpy(
    pcd: o3d.geometry.PointCloud,
    cfg: SegConfig,
    rng: np.random.Generator,
) -> Tuple[List[PlaneMetrics], o3d.geometry.PointCloud]:
    """
    Iteratively extract planes via Open3D RANSAC, but keep stable mapping using NumPy masks.

    Returns:
      - metrics: list of PlaneMetrics
      - colored: full point cloud with colors by plane label (unassigned = gray)
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

    for plane_id in range(cfg.max_planes):
        remaining_idx = np.flatnonzero(remaining_mask)
        if remaining_idx.size < cfg.min_inliers:
            break

        remaining_pts = pts_all[remaining_idx]
        rem_pcd = o3d.geometry.PointCloud()
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

        # Normalize RANSAC plane (not strictly needed, but good practice)
        a, b, c, d = model
        _ = _unit_normal_and_d(float(a), float(b), float(c), float(d))

        # Refit with SVD for a more stable model
        n_refit, d_refit = _refit_plane_svd(inlier_pts)
        n_refit = _stabilize_normal(n_refit)
        d_refit = -float(n_refit.dot(inlier_pts.mean(axis=0)))

        extent_xy = _plane_extent_pca_2d(inlier_pts, n_refit)

        signed = _point_plane_distances(inlier_pts, n_refit, d_refit)
        absd = np.abs(signed)

        rmse = float(math.sqrt(float(np.mean(absd ** 2))))
        mean_abs = float(np.mean(absd))
        std_abs = float(np.std(absd))

        centroid = inlier_pts.mean(axis=0).astype(float).tolist()
        extent_xy = _plane_extent_on_basis(inlier_pts, n_refit)

        inlier_ratio_remaining = float(inliers_global.size / max(1, remaining_idx.size))
        inlier_ratio_total = float(inliers_global.size / max(1, pts_all.shape[0]))

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
                model=PlaneModel(n=n_refit.astype(float).tolist(), d=float(d_refit)),
            )
        )

        labels[inliers_global] = plane_id
        remaining_mask[inliers_global] = False

    # Build colored cloud
    colors = np.full((pts_all.shape[0], 3), 0.5, dtype=np.float64)  # gray for unassigned
    for m in metrics:
        pid = m.plane_id
        colors[labels == pid] = _color_for_plane_id(pid, rng)

    colored = o3d.geometry.PointCloud()
    colored.points = o3d.utility.Vector3dVector(pts_all)
    colored.colors = o3d.utility.Vector3dVector(colors)
    return metrics, colored


def _load_pcd(path: Path) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise ValueError(f"Empty point cloud: {path}")
    return pcd


def _list_frames(input_path: Path) -> List[Path]:
    if input_path.is_dir():
        frames = sorted(input_path.glob("*.ply"))
        if not frames:
            raise FileNotFoundError(f"No .ply files found in: {input_path}")
        return frames
    if input_path.is_file():
        return [input_path]
    raise FileNotFoundError(f"Input path not found: {input_path}")


def _write_json(path: Path, data: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def _preset_defaults(preset: str) -> Dict[str, float]:
    """
    MVP presets. Tweak quickly without changing code.
    """
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
    raise ValueError("Invalid preset. Use: indoor|outdoor")


def _parse_args() -> SegConfig:
    parser = argparse.ArgumentParser(description="Iterative plane segmentation (Open3D RANSAC).")
    parser.add_argument("--input", required=True, help="Input .ply file or directory with .ply frames.")
    parser.add_argument("--out_dir", required=True, help="Output directory for results.")
    parser.add_argument("--preset", default="indoor", choices=["indoor", "outdoor"], help="Parameter preset.")
    parser.add_argument("--max_planes", type=int, default=4, help="Max number of planes to extract per frame.")
    parser.add_argument("--min_inliers", type=int, default=1500, help="Minimum inliers to accept a plane.")
    parser.add_argument("--distance_threshold", type=float, default=-1.0, help="RANSAC distance threshold (m).")
    parser.add_argument("--ransac_n", type=int, default=3, help="RANSAC sample size.")
    parser.add_argument("--num_iterations", type=int, default=2000, help="RANSAC iterations.")
    parser.add_argument("--sor_nb_neighbors", type=int, default=20, help="SOR neighbors.")
    parser.add_argument("--sor_std_ratio", type=float, default=-1.0, help="SOR std ratio.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed for deterministic coloring.")

    args = parser.parse_args()
    defaults = _preset_defaults(args.preset)

    dist_th = float(args.distance_threshold if args.distance_threshold > 0 else defaults["distance_threshold"])
    min_inl = int(args.min_inliers if args.min_inliers > 0 else defaults["min_inliers"])
    sor_std = float(args.sor_std_ratio if args.sor_std_ratio > 0 else defaults["sor_std_ratio"])

    return SegConfig(
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


def _process_frame(frame_path: Path, frame_idx: int, cfg: SegConfig) -> Dict:
    rng = np.random.default_rng(cfg.seed + frame_idx)

    pcd = _load_pcd(frame_path)
    pcd = _apply_sor(pcd, nb_neighbors=cfg.sor_nb_neighbors, std_ratio=cfg.sor_std_ratio)

    metrics, colored = _segment_planes_iterative_numpy(pcd, cfg, rng)
    classification = _classify_floor_and_wall(metrics)

    cfg.out_dir.mkdir(parents=True, exist_ok=True)

    out_ply = cfg.out_dir / f"colored_planes_{frame_idx:05d}.ply"
    o3d.io.write_point_cloud(str(out_ply), colored, write_ascii=False, compressed=False)

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
    _write_json(out_json, payload)

    return {
        "frame_idx": frame_idx,
        "input_file": str(frame_path),
        "num_points": payload["num_points"],
        "num_planes": len(metrics),
        "planes": payload["planes"],
        "colored_ply": str(out_ply),
        "planes_json": str(out_json),
    }


def main() -> None:
    cfg = _parse_args()
    frames = _list_frames(cfg.input_path)

    results: List[Dict] = []
    for i, frame in enumerate(frames):
        print(f"[INFO] Processing frame {i+1}/{len(frames)}: {frame.name}")
        try:
            res = _process_frame(frame, i, cfg)
            results.append(res)
            print(f"[INFO]  -> planes={res['num_planes']} points={res['num_points']}")
        except Exception as exc:
            print(f"[ERROR] Failed on {frame}: {exc}")

    summary = {
        "input": str(cfg.input_path),
        "out_dir": str(cfg.out_dir),
        "preset": cfg.preset,
        "num_frames": len(frames),
        "processed_frames": len(results),
        "frames": results,
    }

    _write_json(cfg.out_dir / "summary.json", summary)
    print(f"[INFO] Done. Summary: {cfg.out_dir / 'summary.json'}")


if __name__ == "__main__":
    main()
