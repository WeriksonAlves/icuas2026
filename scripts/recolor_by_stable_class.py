#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Recolor point clouds using stable plane classification.

Uses classification_stable.json (produced by classify_tracks_stably.py)
to assign fixed colors per semantic class:

- floor     -> red    [1, 0, 0]
- table     -> green  [0, 1, 0]
- main_wall -> blue   [0, 0, 1]

For each frame:
  - load frame_XXXXX.ply (raw or pre-filtered point cloud)
  - for each semantic class present at that frame:
      * recover the track's mean plane (normal, centroid)
      * compute distance of all points to that plane
      * assign the class color to points close to that plane

Points not assigned to any class remain gray.

This is independent of the order in which RANSAC extracted planes.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import open3d as o3d


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _load_classification(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    return data


def _build_track_summary_map(classif: Dict) -> Dict[int, Dict]:
    """
    Build mapping: track_id -> track_summary entry.
    """
    summary_list: List[Dict] = classif.get("tracks_summary", [])
    mapping: Dict[int, Dict] = {}
    for ts in summary_list:
        tid = int(ts["track_id"])
        mapping[tid] = ts
    return mapping


def _plane_from_track_summary(ts: Dict) -> tuple[np.ndarray, float]:
    """
    Given a track summary, reconstruct plane parameters (n, d)
    from mean_normal and mean_centroid.

    We assume Hesse-like form: n^T x + d = 0, with ||n|| ~ 1.
    """
    n = np.asarray(ts["mean_normal"], dtype=np.float64)
    c = np.asarray(ts["mean_centroid"], dtype=np.float64)
    # Normalize normal to be safe
    n_norm = np.linalg.norm(n)
    if n_norm > 1e-9:
        n = n / n_norm
    d = -float(n.dot(c))
    return n, d


def _color_for_class(cls: str) -> np.ndarray:
    """
    Fixed RGB colors in [0,1] for each semantic class.
    """
    if cls == "floor":
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)  # red
    if cls == "table":
        return np.array([0.0, 1.0, 0.0], dtype=np.float64)  # green
    if cls == "main_wall":
        return np.array([0.0, 0.0, 1.0], dtype=np.float64)  # blue
    # Fallback: gray
    return np.array([0.5, 0.5, 0.5], dtype=np.float64)


# ---------------------------------------------------------------------------
# Core recoloring
# ---------------------------------------------------------------------------


def recolor_frames(
    input_dir: Path,
    classif: Dict,
    out_dir: Path,
    distance_threshold: float,
) -> None:
    """
    Recolor all frames listed in classification_stable.json.

    Parameters
    ----------
    input_dir : Path
        Directory containing frame_XXXXX.ply files.
    classif : Dict
        Parsed classification_stable.json.
    out_dir : Path
        Directory to write recolored clouds.
    distance_threshold : float
        Max absolute distance (m) to decide if a point belongs to a plane.
    """
    out_dir.mkdir(parents=True, exist_ok=True)

    frames = classif.get("frames", {})
    track_summary_map = _build_track_summary_map(classif)

    classes = ["floor", "table", "main_wall"]

    for frame_key, roles in frames.items():
        frame_idx = int(frame_key)
        in_path = input_dir / f"frame_{frame_idx:05d}.ply"
        if not in_path.exists():
            print(f"[WARN] Input cloud not found for frame {frame_idx}: {in_path}")
            continue

        print(f"[INFO] Recoloring frame {frame_idx} from {in_path.name}")
        pcd = o3d.io.read_point_cloud(str(in_path))
        if pcd.is_empty():
            print(f"[WARN] Empty point cloud: {in_path}")
            continue

        pts = np.asarray(pcd.points)
        n_points = pts.shape[0]
        if n_points == 0:
            print(f"[WARN] No points in: {in_path}")
            continue

        # Initialize colors as gray and "best distance" as +inf
        colors = np.full((n_points, 3), 0.5, dtype=np.float64)
        best_dist = np.full((n_points,), np.inf, dtype=np.float64)

        # For each semantic class, try to assign points that are closer to this
        # plane than to any previously processed plane.
        for cls in classes:
            entry: Optional[Dict] = roles.get(cls)
            if entry is None:
                continue

            track_id = int(entry["track_id"])
            ts = track_summary_map.get(track_id)
            if ts is None:
                print(f"[WARN] No track summary for track_id={track_id}")
                continue

            n, d = _plane_from_track_summary(ts)
            dist = np.abs(pts @ n + d)

            mask_candidate = dist <= distance_threshold
            better = np.logical_and(mask_candidate, dist < best_dist)

            cls_color = _color_for_class(cls)
            colors[better, :] = cls_color
            best_dist[better] = dist[better]

        # Save recolored cloud
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(pts)
        out_pcd.colors = o3d.utility.Vector3dVector(colors)

        out_path = out_dir / f"stable_semantic_{frame_idx:05d}.ply"
        o3d.io.write_point_cloud(str(out_path), out_pcd, write_ascii=False)
        print(f"[INFO]  -> wrote {out_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Recolor point clouds using stable plane classification "
            "(floor/table/main_wall with fixed colors)."
        )
    )
    parser.add_argument(
        "--input_dir",
        required=True,
        help="Directory with frame_XXXXX.ply point clouds.",
    )
    parser.add_argument(
        "--classification",
        required=True,
        help="classification_stable.json file from classify_tracks_stably.py.",
    )
    parser.add_argument(
        "--out_dir",
        required=True,
        help="Output directory for recolored point clouds.",
    )
    parser.add_argument(
        "--distance_threshold",
        type=float,
        default=0.04,
        help="Max distance (m) to assign a point to a plane (default: 0.04).",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    input_dir = Path(args.input_dir).resolve()
    classif_path = Path(args.classification).resolve()
    out_dir = Path(args.out_dir).resolve()
    dist_th = float(args.distance_threshold)

    print(f"[INFO] Loading classification from: {classif_path}")
    classif = _load_classification(classif_path)

    recolor_frames(
        input_dir=input_dir,
        classif=classif,
        out_dir=out_dir,
        distance_threshold=dist_th,
    )


if __name__ == "__main__":
    main()