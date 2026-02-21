# src/planes/recolor.py
"""Recolor point clouds using stable plane classification.

This module consumes the JSON payload produced by
:mod:`planes.classification` (classification_stable.json) and recolors
per-frame point clouds according to semantic classes:

- floor     -> red    [1, 0, 0]
- table     -> green  [0, 1, 0]
- main_wall -> blue   [0, 0, 1]

For each frame:

1. Load frame_XXXXX.ply from the given input directory.
2. For each semantic class present in that frame, recover the plane
   parameters (normal, d) from the global track summary.
3. Compute the point-to-plane distance for all points.
4. Assign the point to the closest semantic plane, under a distance
   threshold, using fixed class colors.

The main entry point is :func:`run_recolor`.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import open3d as o3d


SemanticFrames = Dict[str, Dict[str, Optional[Dict]]]
TrackSummaryMap = Dict[int, Dict]
TrackPlaneMap = Dict[int, Tuple[np.ndarray, float]]  # track_id -> (n, d)


@dataclass(frozen=True)
class RecolorConfig:
    """Configuration parameters for semantic recoloring."""

    input_dir: Path
    classification_path: Path
    out_dir: Path
    distance_threshold: float = 0.04

    @classmethod
    def from_cli_args(cls, args) -> "RecolorConfig":
        """Build config from an argparse namespace."""
        return cls(
            input_dir=Path(args.input_dir).expanduser().resolve(),
            classification_path=Path(
                args.classification
            ).expanduser().resolve(),
            out_dir=Path(args.out_dir).expanduser().resolve(),
            distance_threshold=float(args.distance_threshold),
        )


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------


def load_classification(path: Path) -> Dict:
    """Load classification_stable.json into a Python dictionary."""
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    return data


def _build_track_summary_map(classif: Dict) -> TrackSummaryMap:
    """Build mapping: track_id -> track_summary entry."""
    summary_list: List[Dict] = classif.get("tracks_summary", [])
    mapping: TrackSummaryMap = {}
    for ts in summary_list:
        tid = int(ts["track_id"])
        mapping[tid] = ts
    return mapping


def _plane_from_track_summary(ts: Dict) -> Tuple[np.ndarray, float]:
    """Reconstruct plane parameters (n, d) from a track summary entry.

    We assume Hesse-like form: n^T x + d = 0, with ||n|| ~ 1.
    """
    n = np.asarray(ts["mean_normal"], dtype=np.float64)
    c = np.asarray(ts["mean_centroid"], dtype=np.float64)

    n_norm = np.linalg.norm(n)
    if n_norm > 1e-9:
        n = n / n_norm

    d = -float(n.dot(c))
    return n, d


def _build_track_plane_map(classif: Dict) -> TrackPlaneMap:
    """
    Precompute (normal, d) for each track from the classification summary.
    """
    summary_map = _build_track_summary_map(classif)
    plane_map: TrackPlaneMap = {}
    for tid, ts in summary_map.items():
        plane_map[tid] = _plane_from_track_summary(ts)
    return plane_map


def _color_for_class(cls: str) -> np.ndarray:
    """Fixed RGB colors in [0, 1] for each semantic class."""
    if cls == "floor":
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)  # red
    if cls == "table":
        return np.array([0.0, 1.0, 0.0], dtype=np.float64)  # green
    if cls == "main_wall":
        return np.array([0.0, 0.0, 1.0], dtype=np.float64)  # blue
    # Fallback: gray
    return np.array([0.5, 0.5, 0.5], dtype=np.float64)


# ---------------------------------------------------------------------------
# Core recoloring logic
# ---------------------------------------------------------------------------


def _recolor_single_frame(
    frame_idx: int,
    roles: Dict[str, Optional[Dict]],
    track_plane_map: TrackPlaneMap,
    cfg: RecolorConfig,
) -> Optional[Path]:
    """Recolor a single frame based on semantic roles for that frame.

    Parameters
    ----------
    frame_idx :
        Index of the frame (used to build file names).
    roles :
        Dictionary mapping semantic class ("floor", "table", "main_wall")
        to entries of the form::

            { "track_id": <int>, "plane_id": <int> }

        or None if the class is not present in that frame.
    track_plane_map :
        Mapping from track_id to (normal, d) plane parameters.
    cfg :
        Recolor configuration, including input_dir, out_dir and
        distance_threshold.

    Returns
    -------
    Path | None
        Output .ply path if a recolored cloud was written, otherwise None.
    """
    in_path = cfg.input_dir / f"frame_{frame_idx:05d}.ply"
    if not in_path.exists():
        # Missing input frame: nothing to do.
        return None

    pcd = o3d.io.read_point_cloud(str(in_path))
    if pcd.is_empty():
        return None

    pts = np.asarray(pcd.points)
    n_points = int(pts.shape[0])
    if n_points == 0:
        return None

    # Initialize colors as gray and best distances as +inf.
    colors = np.full((n_points, 3), 0.5, dtype=np.float64)
    best_dist = np.full((n_points,), np.inf, dtype=np.float64)

    classes = ["floor", "table", "main_wall"]

    for cls in classes:
        entry = roles.get(cls)
        if entry is None:
            continue

        track_id = int(entry["track_id"])
        plane = track_plane_map.get(track_id)
        if plane is None:
            # Track not found in summary; skip this role.
            continue

        n, d = plane
        dist = np.abs(pts @ n + d)

        # Points close enough to this plane.
        mask_candidate = dist <= cfg.distance_threshold
        # Among candidates, keep only those closer than current best.
        better = np.logical_and(mask_candidate, dist < best_dist)

        cls_color = _color_for_class(cls)
        colors[better, :] = cls_color
        best_dist[better] = dist[better]

    # If no point was assigned to any class, we still write the gray cloud.
    out_pcd = o3d.geometry.PointCloud()
    out_pcd.points = o3d.utility.Vector3dVector(pts)
    out_pcd.colors = o3d.utility.Vector3dVector(colors)

    cfg.out_dir.mkdir(parents=True, exist_ok=True)
    out_path = cfg.out_dir / f"stable_semantic_{frame_idx:05d}.ply"
    o3d.io.write_point_cloud(str(out_path), out_pcd, write_ascii=False)
    return out_path


def run_recolor(cfg: RecolorConfig) -> Dict:
    """Run semantic recoloring for all frames described in the classification.

    Parameters
    ----------
    cfg :
        Recolor configuration (input_dir, classification_path, out_dir,
        threshold).

    Returns
    -------
    dict
        Summary dictionary with basic statistics, useful for logging or tests.
    """
    classif = load_classification(cfg.classification_path)
    frames: SemanticFrames = classif.get("frames", {})
    track_plane_map = _build_track_plane_map(classif)

    frame_keys = sorted(int(k) for k in frames.keys())
    num_frames = len(frame_keys)

    written: List[int] = []
    missing_input: List[int] = []
    empty_clouds: List[int] = []

    for frame_idx in frame_keys:
        roles = frames[str(frame_idx)]
        out_path = _recolor_single_frame(
            frame_idx=frame_idx,
            roles=roles,
            track_plane_map=track_plane_map,
            cfg=cfg,
        )
        if out_path is None:
            # Determine if this was due to missing/empty input.
            in_path = cfg.input_dir / f"frame_{frame_idx:05d}.ply"
            if not in_path.exists():
                missing_input.append(frame_idx)
            else:
                empty_clouds.append(frame_idx)
            continue

        written.append(frame_idx)

    summary = {
        "input_dir": str(cfg.input_dir),
        "classification_path": str(cfg.classification_path),
        "out_dir": str(cfg.out_dir),
        "distance_threshold": float(cfg.distance_threshold),
        "num_frames": num_frames,
        "num_written": len(written),
        "written_frames": written,
        "missing_input_frames": missing_input,
        "empty_cloud_frames": empty_clouds,
    }
    return summary
