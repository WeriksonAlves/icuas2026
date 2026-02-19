#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Temporal stability analysis for plane segmentation results.

This script reads planes_XXXXX.json files produced by segment_planes_open3d.py
and computes temporal stability metrics for:

- floor plane (classification.floor_plane_id)
- table/bed plane (classification.table_plane_id)
- main wall plane (classification.main_wall_plane_id)

For each class, it:
  - builds time series of normals, offsets (d), centroids and inlier_ratio_total
  - computes angular drift and per-frame variations
  - writes a JSON summary with statistics
  - saves plots as PNG files for visual inspection
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np


@dataclass(frozen=True)
class PlaneTimeSeries:
    """Time series for a specific semantic plane (floor, table, wall)."""
    name: str
    frames: List[int]
    normals: np.ndarray        # shape (N, 3)
    offsets: np.ndarray        # shape (N,)
    centroids: np.ndarray      # shape (N, 3)
    inlier_ratios: np.ndarray  # shape (N,)


@dataclass(frozen=True)
class TemporalStabilityMetrics:
    """Summary metrics for temporal stability."""
    name: str
    num_samples: int

    mean_angle_to_first_deg: Optional[float]
    std_angle_to_first_deg: Optional[float]
    max_angle_to_first_deg: Optional[float]

    mean_delta_angle_deg: Optional[float]
    std_delta_angle_deg: Optional[float]
    max_delta_angle_deg: Optional[float]

    mean_delta_d: Optional[float]
    std_delta_d: Optional[float]
    max_delta_d: Optional[float]

    mean_delta_centroid: Optional[float]
    std_delta_centroid: Optional[float]
    max_delta_centroid: Optional[float]

    mean_inlier_ratio: Optional[float]
    std_inlier_ratio: Optional[float]


def _slugify_name(name: str) -> str:
    """Convert a human-readable name into a safe file name prefix."""
    return (
        name.lower()
        .replace(" ", "_")
        .replace("/", "_")
        .replace("\\", "_")
    )


def _safe_acos(x: float) -> float:
    """Clamp input to [-1, 1] before calling acos to avoid NaNs."""
    return math.acos(max(min(x, 1.0), -1.0))


def _angle_between_normals_deg(n1: np.ndarray, n2: np.ndarray) -> float:
    """Return angle in degrees between two normals (ignoring sign)."""
    n1 = np.asarray(n1, dtype=np.float64)
    n2 = np.asarray(n2, dtype=np.float64)
    n1_norm = np.linalg.norm(n1)
    n2_norm = np.linalg.norm(n2)
    if n1_norm < 1e-9 or n2_norm < 1e-9:
        return float("nan")
    cos_theta = abs(float(n1.dot(n2)) / (n1_norm * n2_norm))
    return math.degrees(_safe_acos(cos_theta))


def _load_plane_series(
    input_dir: Path,
    class_key: str,
    name: str,
) -> Optional[PlaneTimeSeries]:
    """
    Build time series for a given semantic plane class.

    Parameters
    ----------
    input_dir : Path
        Directory containing planes_XXXXX.json files.
    class_key : str
        Key inside "classification" dict:
          - "floor_plane_id"
          - "table_plane_id"
          - "main_wall_plane_id"
    name : str
        Human-readable name for this class (for plots/summary).

    Returns
    -------
    PlaneTimeSeries or None
        None if no frame had this plane class defined.
    """
    json_paths = sorted(input_dir.glob("planes_*.json"))
    if not json_paths:
        raise FileNotFoundError(f"No planes_XXXXX.json found in: {input_dir}")

    frames: List[int] = []
    normals: List[np.ndarray] = []
    offsets: List[float] = []
    centroids: List[np.ndarray] = []
    inlier_ratios: List[float] = []

    for path in json_paths:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        classification = data.get("classification", {})
        plane_id = classification.get(class_key)
        if plane_id is None:
            continue

        planes = data.get("planes", [])
        selected: Optional[Dict] = None
        for p in planes:
            if int(p.get("plane_id", -1)) == int(plane_id):
                selected = p
                break

        if selected is None:
            # Inconsistent JSON; skip this frame.
            continue

        frame_idx = int(data.get("frame_idx", -1))
        model = selected["model"]
        n = np.asarray(model["n"], dtype=np.float64)
        d = float(model["d"])
        c = np.asarray(selected["centroid"], dtype=np.float64)
        ir = float(selected.get("inlier_ratio_total", 0.0))

        frames.append(frame_idx)
        normals.append(n)
        offsets.append(d)
        centroids.append(c)
        inlier_ratios.append(ir)

    if not frames:
        return None

    normals_arr = np.vstack(normals)
    offsets_arr = np.asarray(offsets, dtype=np.float64)
    centroids_arr = np.vstack(centroids)
    inlier_ratios_arr = np.asarray(inlier_ratios, dtype=np.float64)

    return PlaneTimeSeries(
        name=name,
        frames=frames,
        normals=normals_arr,
        offsets=offsets_arr,
        centroids=centroids_arr,
        inlier_ratios=inlier_ratios_arr,
    )


def _compute_temporal_metrics(series: PlaneTimeSeries) -> TemporalStabilityMetrics:
    """Compute temporal stability metrics for a given plane time series."""
    n_samples = len(series.frames)
    if n_samples == 0:
        raise ValueError("Empty time series.")

    normals = series.normals
    offsets = series.offsets
    centroids = series.centroids
    inliers = series.inlier_ratios

    # --- angle to first frame ---
    base_n = normals[0]
    angles_to_first = np.array(
        [_angle_between_normals_deg(base_n, n) for n in normals],
        dtype=np.float64,
    )

    # --- angle between consecutive frames ---
    if n_samples >= 2:
        delta_angles = np.array(
            [
                _angle_between_normals_deg(normals[i - 1], normals[i])
                for i in range(1, n_samples)
            ],
            dtype=np.float64,
        )
        delta_offsets = np.abs(np.diff(offsets))
        delta_centroids = np.linalg.norm(np.diff(centroids, axis=0), axis=1)
    else:
        delta_angles = np.array([], dtype=np.float64)
        delta_offsets = np.array([], dtype=np.float64)
        delta_centroids = np.array([], dtype=np.float64)

    def _stats(arr: np.ndarray) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        if arr.size == 0:
            return None, None, None
        return float(np.nanmean(arr)), float(np.nanstd(arr)), float(np.nanmax(arr))

    mean_a1, std_a1, max_a1 = _stats(angles_to_first)
    mean_da, std_da, max_da = _stats(delta_angles)
    mean_dd, std_dd, max_dd = _stats(delta_offsets)
    mean_dc, std_dc, max_dc = _stats(delta_centroids)

    mean_ir: Optional[float]
    std_ir: Optional[float]
    if inliers.size == 0:
        mean_ir = None
        std_ir = None
    else:
        mean_ir = float(np.mean(inliers))
        std_ir = float(np.std(inliers))

    return TemporalStabilityMetrics(
        name=series.name,
        num_samples=n_samples,
        mean_angle_to_first_deg=mean_a1,
        std_angle_to_first_deg=std_a1,
        max_angle_to_first_deg=max_a1,
        mean_delta_angle_deg=mean_da,
        std_delta_angle_deg=std_da,
        max_delta_angle_deg=max_da,
        mean_delta_d=mean_dd,
        std_delta_d=std_dd,
        max_delta_d=max_dd,
        mean_delta_centroid=mean_dc,
        std_delta_centroid=std_dc,
        max_delta_centroid=max_dc,
        mean_inlier_ratio=mean_ir,
        std_inlier_ratio=std_ir,
    )


def _plot_time_series(
    series: PlaneTimeSeries,
    out_dir: Path,
) -> None:
    """Create and save plots for a plane time series."""
    frames = np.asarray(series.frames, dtype=int)
    normals = series.normals
    offsets = series.offsets
    centroids = series.centroids
    inliers = series.inlier_ratios

    n_samples = len(frames)
    base_n = normals[0]
    angles_to_first = np.array(
        [_angle_between_normals_deg(base_n, n) for n in normals],
        dtype=np.float64,
    )

    if n_samples >= 2:
        delta_angles = np.array(
            [
                _angle_between_normals_deg(normals[i - 1], normals[i])
                for i in range(1, n_samples)
            ],
            dtype=np.float64,
        )
        delta_offsets = np.abs(np.diff(offsets))
        delta_centroids = np.linalg.norm(np.diff(centroids, axis=0), axis=1)
        frames_deltas = frames[1:]
    else:
        delta_angles = np.array([], dtype=np.float64)
        delta_offsets = np.array([], dtype=np.float64)
        delta_centroids = np.array([], dtype=np.float64)
        frames_deltas = np.array([], dtype=int)

    # Figure 1: orientation and offset over time
    fig1, axes1 = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    fig1.suptitle(f"{series.name} - Orientation & Offset", fontsize=14)

    axes1[0].plot(frames, angles_to_first, marker="o")
    axes1[0].set_ylabel("Angle to first (deg)")
    axes1[0].grid(True, linestyle="--", alpha=0.4)

    axes1[1].plot(frames, offsets, marker="o")
    axes1[1].set_ylabel("Offset d (m)")
    axes1[1].grid(True, linestyle="--", alpha=0.4)

    axes1[2].plot(frames, inliers, marker="o")
    axes1[2].set_ylabel("Inlier ratio (total)")
    axes1[2].set_xlabel("Frame index")
    axes1[2].grid(True, linestyle="--", alpha=0.4)

    fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
    out_dir.mkdir(parents=True, exist_ok=True)
    prefix = _slugify_name(series.name)
    fig1_path = out_dir / f"{prefix}_orientation_offset.png"
    fig1.savefig(fig1_path, dpi=150)
    plt.close(fig1)

    # Figure 2: per-frame deltas
    if frames_deltas.size > 0:
        fig2, axes2 = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
        fig2.suptitle(f"{series.name} - Per-frame deltas", fontsize=14)

        axes2[0].plot(frames_deltas, delta_angles, marker="o")
        axes2[0].set_ylabel("Δ angle (deg)")
        axes2[0].grid(True, linestyle="--", alpha=0.4)

        axes2[1].plot(frames_deltas, delta_offsets, marker="o")
        axes2[1].set_ylabel("Δ d (m)")
        axes2[1].grid(True, linestyle="--", alpha=0.4)

        axes2[2].plot(frames_deltas, delta_centroids, marker="o")
        axes2[2].set_ylabel("Δ centroid (m)")
        axes2[2].set_xlabel("Frame index")
        axes2[2].grid(True, linestyle="--", alpha=0.4)

        fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig2_path = out_dir / f"{prefix}_deltas.png"
        fig2.savefig(fig2_path, dpi=150)
        plt.close(fig2)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Temporal stability analysis for segmented planes.",
    )
    parser.add_argument(
        "--input_dir",
        required=True,
        help="Directory containing planes_XXXXX.json files.",
    )
    parser.add_argument(
        "--out_dir",
        required=True,
        help="Output directory for summary JSON and plots.",
    )
    parser.add_argument(
        "--no_plots",
        action="store_true",
        help="If set, do not generate PNG plots (only JSON summary).",
    )
    return parser.parse_args()


def _fmt(value: Optional[float]) -> str:
    if value is None or np.isnan(value):
        return "nan"
    return f"{value:.3f}"


def main() -> None:
    args = _parse_args()
    input_dir = Path(args.input_dir).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    series_list: List[PlaneTimeSeries] = []

    floor_series = _load_plane_series(input_dir, "floor_plane_id", "Floor")
    if floor_series is not None:
        series_list.append(floor_series)

    table_series = _load_plane_series(input_dir, "table_plane_id", "Table/Bed")
    if table_series is not None:
        series_list.append(table_series)

    wall_series = _load_plane_series(input_dir, "main_wall_plane_id", "Main wall")
    if wall_series is not None:
        series_list.append(wall_series)

    if not series_list:
        raise RuntimeError("No valid plane time series found in the input directory.")

    summary: Dict[str, Dict] = {
        "input_dir": str(input_dir),
        "num_series": len(series_list),
        "series": {},
    }

    for series in series_list:
        metrics = _compute_temporal_metrics(series)
        summary["series"][series.name] = asdict(metrics)

        mean_a = metrics.mean_angle_to_first_deg
        max_da = metrics.max_delta_angle_deg

        print(
            f"[INFO] {series.name}: "
            f"samples={metrics.num_samples}, "
            f"mean_angle_to_first={_fmt(metrics.mean_angle_to_first_deg)} deg, "
            f"max_delta_angle={_fmt(metrics.max_delta_angle_deg)} deg"
        )


        if not args.no_plots:
            _plot_time_series(series, out_dir)

    summary_path = out_dir / "temporal_stability_summary.json"
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"[INFO] Summary written to: {summary_path}")


if __name__ == "__main__":
    main()
