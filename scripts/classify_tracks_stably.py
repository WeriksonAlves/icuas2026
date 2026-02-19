#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stable semantic classification of plane tracks.

Reads plane_tracks.json produced by track_planes_temporally.py and assigns
global semantic labels to tracks:
  - floor_track_id   (dominant horizontal plane)
  - table_track_id   (secondary horizontal plane above the floor)
  - main_wall_track_id  (dominant vertical plane)

Then, for each frame, it maps those track labels back to the local plane IDs
detected in that frame, producing a classification_stable.json file.

This replaces the volatile per-frame "classification" based only on single
frame heuristics, by a temporally consistent classification based on tracks.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _safe_acos(x: float) -> float:
    """Clamp x to [-1, 1] before calling acos."""
    return math.acos(max(min(x, 1.0), -1.0))


def angle_between_normals_deg(n1: np.ndarray, n2: np.ndarray) -> float:
    """Return angle in degrees between two normals (ignoring sign)."""
    n1 = np.asarray(n1, dtype=np.float64)
    n2 = np.asarray(n2, dtype=np.float64)
    n1_norm = np.linalg.norm(n1)
    n2_norm = np.linalg.norm(n2)
    if n1_norm < 1e-9 or n2_norm < 1e-9:
        return float("nan")
    cos_theta = abs(float(n1.dot(n2)) / (n1_norm * n2_norm))
    return math.degrees(_safe_acos(cos_theta))


@dataclass
class TrackInfo:
    """Cached metrics for each track."""
    track_id: int
    mean_normal: np.ndarray
    mean_centroid: np.ndarray
    mean_inlier_ratio: float
    num_detections: int
    angle_to_vertical_deg: float
    height_along_vertical: float


# ---------------------------------------------------------------------------
# Core logic
# ---------------------------------------------------------------------------


def _load_tracks(tracks_path: Path) -> Dict:
    with tracks_path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    return data


def _compute_track_infos(
    tracks_data: Dict,
    vertical_axis: np.ndarray,
) -> List[TrackInfo]:
    tracks_raw = tracks_data.get("tracks", [])
    infos: List[TrackInfo] = []

    v = np.asarray(vertical_axis, dtype=np.float64)
    v_norm = np.linalg.norm(v)
    if v_norm < 1e-9:
        raise ValueError("Vertical axis has near-zero norm.")
    v = v / v_norm

    for tr in tracks_raw:
        tid = int(tr["track_id"])
        summary = tr.get("summary", {})
        mean_normal = np.asarray(summary.get("mean_normal"), dtype=np.float64)
        mean_centroid = np.asarray(summary.get("mean_centroid"), dtype=np.float64)
        mean_inlier = float(summary.get("mean_inlier_ratio_total", 0.0))
        num_det = int(tr.get("num_detections", len(tr.get("detections", []))))

        angle = angle_between_normals_deg(mean_normal, v)
        height = float(mean_centroid.dot(v))

        infos.append(
            TrackInfo(
                track_id=tid,
                mean_normal=mean_normal,
                mean_centroid=mean_centroid,
                mean_inlier_ratio=mean_inlier,
                num_detections=num_det,
                angle_to_vertical_deg=angle,
                height_along_vertical=height,
            )
        )

    return infos


def _select_floor_and_table(
    infos: List[TrackInfo],
    horiz_angle_max: float,
    min_height_diff: float,
) -> Tuple[Optional[int], Optional[int]]:
    """Select floor and table tracks among horizontally oriented tracks."""
    # Horizontal tracks: normals roughly aligned with vertical axis.
    horizontals = [
        ti for ti in infos if ti.angle_to_vertical_deg <= horiz_angle_max
    ]
    if not horizontals:
        return None, None

    # Floor: highest inlier ratio among horizontals.
    floor = max(horizontals, key=lambda ti: ti.mean_inlier_ratio)
    floor_id = floor.track_id

    # Table: horizontal, above floor by some margin.
    table_candidates = [
        ti
        for ti in horizontals
        if ti.track_id != floor_id
        and (ti.height_along_vertical - floor.height_along_vertical)
        >= min_height_diff
    ]
    if not table_candidates:
        return floor_id, None

    table = max(table_candidates, key=lambda ti: ti.mean_inlier_ratio)
    table_id = table.track_id
    return floor_id, table_id


def _select_main_wall(
    infos: List[TrackInfo],
    vertical_band_center: float,
    vertical_band_half_width: float,
) -> Optional[int]:
    """Select main wall track among vertically oriented tracks."""
    lower = vertical_band_center - vertical_band_half_width
    upper = vertical_band_center + vertical_band_half_width

    verticals = [
        ti
        for ti in infos
        if lower <= ti.angle_to_vertical_deg <= upper
    ]
    if not verticals:
        return None

    main_wall = max(verticals, key=lambda ti: ti.mean_inlier_ratio)
    return main_wall.track_id


def _build_frame_classification(
    frames_index: Dict[str, List[Dict]],
    floor_track_id: Optional[int],
    table_track_id: Optional[int],
    main_wall_track_id: Optional[int],
) -> Dict[str, Dict[str, Optional[Dict]]]:
    """
    For each frame, indicate which local plane corresponds to each semantic role.
    """
    frames_out: Dict[str, Dict[str, Optional[Dict]]] = {}

    for frame_key, lst in frames_index.items():
        floor_entry: Optional[Dict] = None
        table_entry: Optional[Dict] = None
        wall_entry: Optional[Dict] = None

        for item in lst:
            tid = int(item["track_id"])
            if floor_track_id is not None and tid == floor_track_id:
                floor_entry = {"track_id": tid, "local_id": int(item["local_id"])}
            if table_track_id is not None and tid == table_track_id:
                table_entry = {"track_id": tid, "local_id": int(item["local_id"])}
            if main_wall_track_id is not None and tid == main_wall_track_id:
                wall_entry = {"track_id": tid, "local_id": int(item["local_id"])}

        frames_out[frame_key] = {
            "floor": floor_entry,
            "table": table_entry,
            "main_wall": wall_entry,
        }

    return frames_out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Assign stable semantic labels (floor, table, main wall) "
            "to plane tracks and propagate them per frame."
        )
    )
    parser.add_argument(
        "--tracks",
        required=True,
        help="Input plane_tracks.json (from track_planes_temporally.py).",
    )
    parser.add_argument(
        "--output",
        default="classification_stable.json",
        help="Output JSON file with stable classification.",
    )
    parser.add_argument(
        "--vertical_axis",
        nargs=3,
        type=float,
        default=[1.0, 0.0, 0.0],
        metavar=("VX", "VY", "VZ"),
        help="Vertical axis in camera frame (default: 1 0 0).",
    )
    parser.add_argument(
        "--horiz_angle_max",
        type=float,
        default=35.0,
        help=(
            "Max angle (deg) between plane normal and vertical axis "
            "for a plane to be considered horizontal."
        ),
    )
    parser.add_argument(
        "--min_table_height_diff",
        type=float,
        default=0.2,
        help=(
            "Minimum height difference (m) between table and floor centroids "
            "along the vertical axis."
        ),
    )
    parser.add_argument(
        "--vertical_band_center",
        type=float,
        default=90.0,
        help=(
            "Center (deg) of acceptable band for vertical planes "
            "in angle(normal, vertical_axis)."
        ),
    )
    parser.add_argument(
        "--vertical_band_half_width",
        type=float,
        default=30.0,
        help="Half-width (deg) of band for vertical planes.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    tracks_path = Path(args.tracks).resolve()
    output_path = Path(args.output).resolve()

    vertical_axis = np.array(args.vertical_axis, dtype=np.float64)

    print(f"[INFO] Loading tracks from: {tracks_path}")
    tracks_data = _load_tracks(tracks_path)
    frames_index = tracks_data.get("frames", {})

    infos = _compute_track_infos(tracks_data, vertical_axis)
    print(f"[INFO] Found {len(infos)} tracks in total.")

    # Selection of semantic tracks
    floor_id, table_id = _select_floor_and_table(
        infos,
        horiz_angle_max=float(args.horiz_angle_max),
        min_height_diff=float(args.min_table_height_diff),
    )
    wall_id = _select_main_wall(
        infos,
        vertical_band_center=float(args.vertical_band_center),
        vertical_band_half_width=float(args.vertical_band_half_width),
    )

    print(f"[INFO] floor_track_id = {floor_id}")
    print(f"[INFO] table_track_id = {table_id}")
    print(f"[INFO] main_wall_track_id = {wall_id}")

    # Build per-frame classification
    frames_out = _build_frame_classification(
        frames_index,
        floor_track_id=floor_id,
        table_track_id=table_id,
        main_wall_track_id=wall_id,
    )

    # Export a compact summary of tracks (for debugging / inspection)
    tracks_summary = [
        {
            **asdict(ti),
            "mean_normal": ti.mean_normal.tolist(),
            "mean_centroid": ti.mean_centroid.tolist(),
        }
        for ti in infos
    ]

    payload = {
        "tracks_file": str(tracks_path),
        "vertical_axis": vertical_axis.tolist(),
        "params": {
            "horiz_angle_max": float(args.horiz_angle_max),
            "min_table_height_diff": float(args.min_table_height_diff),
            "vertical_band_center": float(args.vertical_band_center),
            "vertical_band_half_width": float(args.vertical_band_half_width),
        },
        "selected_tracks": {
            "floor_track_id": floor_id,
            "table_track_id": table_id,
            "main_wall_track_id": wall_id,
        },
        "tracks_summary": tracks_summary,
        "frames": frames_out,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print(f"[INFO] Stable classification written to: {output_path}")


if __name__ == "__main__":
    main()
