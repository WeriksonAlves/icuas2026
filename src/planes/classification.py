# src/planes/classification.py
"""Stable semantic classification of plane tracks.

This module operates on the JSON payload produced by
:mod:`planes.tracking` (plane_tracks.json) and assigns *global*
semantic labels to tracks:

- floor_track_id        (dominant horizontal plane)
- table_track_id        (secondary horizontal plane above the floor)
- main_wall_track_id    (dominant vertical plane)

Then, for each frame, it maps those track labels back to the local
plane IDs detected in that frame.

The main public entry point is :func:`run_classification`.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from planes.geometry import safe_acos


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass
class TrackInfo:
    """Cached metrics for each track used during classification."""

    track_id: int
    mean_normal: np.ndarray
    mean_centroid: np.ndarray
    mean_inlier_ratio: float
    num_detections: int
    angle_to_vertical_deg: float
    height_along_vertical: float


@dataclass(frozen=True)
class ClassificationConfig:
    """Configuration parameters for stable classification.

    Parameters
    ----------
    vertical_axis :
        Approximate vertical axis in the camera/world frame. It does
        not need to be normalized; normalization is handled internally.
    horiz_angle_max :
        Maximum angle (deg) between a track normal and the vertical
        axis for the track to be considered "horizontal"
        (candidate for floor/table).
    min_table_height_diff :
        Minimum height difference (m) along the vertical axis between
        the table centroid and the floor centroid.
    vertical_band_center :
        Center (deg) of the acceptable band for vertical planes,
        in terms of angle(normal, vertical_axis).
    vertical_band_half_width :
        Half-width (deg) of the acceptable band for vertical planes.
    """

    vertical_axis: Tuple[float, float, float]
    horiz_angle_max: float = 35.0
    min_table_height_diff: float = 0.20
    vertical_band_center: float = 90.0
    vertical_band_half_width: float = 30.0

    def to_serializable_dict(self) -> Dict:
        """Return a JSON-serializable representation of this config."""
        return {
            "vertical_axis": list(self.vertical_axis),
            "horiz_angle_max": float(self.horiz_angle_max),
            "min_table_height_diff": float(self.min_table_height_diff),
            "vertical_band_center": float(self.vertical_band_center),
            "vertical_band_half_width": float(self.vertical_band_half_width),
        }


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def angle_between_normals_deg(n1: np.ndarray, n2: np.ndarray) -> float:
    """Return angle in degrees between two normals (ignoring sign)."""
    n1 = np.asarray(n1, dtype=np.float64)
    n2 = np.asarray(n2, dtype=np.float64)
    n1_norm = np.linalg.norm(n1)
    n2_norm = np.linalg.norm(n2)
    if n1_norm < 1e-9 or n2_norm < 1e-9:
        return float("nan")
    cos_theta = abs(float(n1.dot(n2)) / (n1_norm * n2_norm))
    return math.degrees(safe_acos(cos_theta))


# ---------------------------------------------------------------------------
# Core logic
# ---------------------------------------------------------------------------


def _load_tracks(tracks_path: Path) -> Dict:
    """Load tracking JSON payload produced by :mod:`planes.tracking`."""
    with tracks_path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    return data


def _compute_track_infos(
    tracks_data: Dict,
    vertical_axis: np.ndarray,
) -> List[TrackInfo]:
    """Compute TrackInfo objects from tracking data and vertical axis."""
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
        mean_centroid = np.asarray(
            summary.get("mean_centroid"),
            dtype=np.float64,
        )
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
    For each frame, indicate which local plane corresponds to each semantic
    role.

    Parameters
    ----------
    frames_index :
        Mapping ``frame_idx(str) -> list of {track_id, local_id}``
        as produced by :mod:`planes.tracking`.
    floor_track_id, table_track_id, main_wall_track_id :
        Selected global track ids for semantics.

    Returns
    -------
    dict
        Mapping ``frame_idx(str) -> { "floor": {...} | None,
                                      "table": {...} | None,
                                      "main_wall": {...} | None }``
        where each non-None entry is::

            {
                "track_id": <global track id>,
                "plane_id": <local plane id in that frame>
            }
    """

    def _lookup_role(
        assignments: List[Dict],
        target_track_id: Optional[int],
    ) -> Optional[Dict]:
        if target_track_id is None:
            return None
        for entry in assignments:
            if int(entry.get("track_id", -1)) == target_track_id:
                return {
                    "track_id": target_track_id,
                    "plane_id": int(entry.get("local_id", -1)),
                }
        return None

    frames_out: Dict[str, Dict[str, Optional[Dict]]] = {}

    for frame_key, lst in sorted(
        frames_index.items(), key=lambda kv: int(kv[0])
    ):
        frames_out[frame_key] = {
            "floor": _lookup_role(lst, floor_track_id),
            "table": _lookup_role(lst, table_track_id),
            "main_wall": _lookup_role(lst, main_wall_track_id),
        }

    return frames_out


# ---------------------------------------------------------------------------
# High-level orchestration
# ---------------------------------------------------------------------------


def run_classification(
    tracks_path: Path,
    cfg: ClassificationConfig,
) -> Dict:
    """
    Run stable semantic classification on plane tracks.

    Parameters
    ----------
    tracks_path :
        Path to ``plane_tracks.json`` generated by :mod:`planes.tracking`.
    cfg :
        Classification configuration.

    Returns
    -------
    dict
        Summary dictionary ready to be written to
        ``classification_stable.json``.
    """
    tracks_data = _load_tracks(tracks_path)
    frames_index = tracks_data.get("frames", {})

    vertical_axis = np.asarray(cfg.vertical_axis, dtype=np.float64)

    infos = _compute_track_infos(tracks_data, vertical_axis)

    floor_id, table_id = _select_floor_and_table(
        infos,
        horiz_angle_max=float(cfg.horiz_angle_max),
        min_height_diff=float(cfg.min_table_height_diff),
    )
    wall_id = _select_main_wall(
        infos,
        vertical_band_center=float(cfg.vertical_band_center),
        vertical_band_half_width=float(cfg.vertical_band_half_width),
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

    # Build per-frame classification
    frames_out = _build_frame_classification(
        frames_index,
        floor_track_id=floor_id,
        table_track_id=table_id,
        main_wall_track_id=wall_id,
    )

    payload = {
        "tracks_file": str(tracks_path),
        "config": cfg.to_serializable_dict(),
        "selected_tracks": {
            "floor_track_id": floor_id,
            "table_track_id": table_id,
            "main_wall_track_id": wall_id,
        },
        "tracks_summary": tracks_summary,
        "frames": frames_out,
    }

    return payload
