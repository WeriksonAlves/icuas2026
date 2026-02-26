# src/planes/semantic_classification.py
from __future__ import annotations

import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


@dataclass
class TrackInfo:
    """Track geometric summary and confidence score."""
    track_id: int
    mean_normal: np.ndarray
    mean_centroid: np.ndarray
    mean_inlier_ratio: float
    num_detections: int
    angle_to_vertical_deg: float
    height_along_vertical: float
    confidence_score: Optional[float]


def _safe_acos(x: float) -> float:
    """Clamp input to [-1, 1] before calling acos."""
    return math.acos(max(min(x, 1.0), -1.0))


def angle_between_normals_deg(n1: np.ndarray, n2: np.ndarray) -> float:
    """Return angle in degrees between two normals (sign-agnostic)."""
    n1 = np.asarray(n1, dtype=np.float64)
    n2 = np.asarray(n2, dtype=np.float64)
    n1_norm = np.linalg.norm(n1)
    n2_norm = np.linalg.norm(n2)
    if n1_norm < 1e-9 or n2_norm < 1e-9:
        return float("nan")
    cos_theta = abs(float(n1.dot(n2)) / (n1_norm * n2_norm))
    return math.degrees(_safe_acos(cos_theta))


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _build_confidence_map(conf_data: Dict[str, Any]) -> Dict[int, float]:
    """
    Build track_id -> confidence_score mapping from track_confidence.json.
    """
    mapping: Dict[int, float] = {}
    entries = conf_data.get("tracks_confidence", [])
    for entry in entries:
        tid = int(entry["track_id"])
        score = float(entry.get("confidence_score", 0.0))
        mapping[tid] = score
    return mapping


def _compute_track_infos(
    tracks_data: Dict[str, Any],
    vertical_axis: np.ndarray,
    confidence_map: Dict[int, float],
) -> List[TrackInfo]:
    """Compute geometric and confidence summaries for each track."""
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
        conf = confidence_map.get(tid, None)

        infos.append(
            TrackInfo(
                track_id=tid,
                mean_normal=mean_normal,
                mean_centroid=mean_centroid,
                mean_inlier_ratio=mean_inlier,
                num_detections=num_det,
                angle_to_vertical_deg=angle,
                height_along_vertical=height,
                confidence_score=conf,
            )
        )

    return infos


def _select_floor_and_table(
    infos: List[TrackInfo],
    horiz_angle_max: float,
    min_height_diff: float,
    min_confidence: float,
) -> Tuple[Optional[int], Optional[int]]:
    """
    Select floor and table tracks among horizontally oriented, confident tracks.
    """
    horizontals = [
        ti
        for ti in infos
        if ti.angle_to_vertical_deg <= horiz_angle_max
        and ti.confidence_score is not None
        and ti.confidence_score >= min_confidence
    ]
    if not horizontals:
        return None, None

    horizontals_sorted = sorted(
        horizontals,
        key=lambda ti: (ti.mean_inlier_ratio, ti.confidence_score or 0.0),
        reverse=True,
    )

    floor = horizontals_sorted[0]
    floor_id = floor.track_id

    table_candidates = [
        ti
        for ti in horizontals_sorted[1:]
        if (
            ti.height_along_vertical - floor.height_along_vertical
        ) >= min_height_diff
    ]
    if not table_candidates:
        return floor_id, None

    table = table_candidates[0]
    return floor_id, table.track_id


def _select_main_wall(
    infos: List[TrackInfo],
    vertical_band_center: float,
    vertical_band_half_width: float,
    min_confidence: float,
) -> Optional[int]:
    """
    Select main wall track among vertically oriented, confident tracks.
    """
    lower = vertical_band_center - vertical_band_half_width
    upper = vertical_band_center + vertical_band_half_width

    verticals = [
        ti
        for ti in infos
        if lower <= ti.angle_to_vertical_deg <= upper
        and ti.confidence_score is not None
        and ti.confidence_score >= min_confidence
    ]
    if not verticals:
        return None

    verticals_sorted = sorted(
        verticals,
        key=lambda ti: (ti.mean_inlier_ratio, ti.confidence_score or 0.0),
        reverse=True,
    )
    return verticals_sorted[0].track_id


def _build_frame_classification(
    frames_index: Dict[str, List[Dict[str, int]]],
    floor_track_id: Optional[int],
    table_track_id: Optional[int],
    main_wall_track_id: Optional[int],
) -> Dict[str, Dict[str, Optional[Dict[str, int]]]]:
    """
    For each frame, indicate which local plane corresponds to each semantic role.
    """
    frames_out: Dict[str, Dict[str, Optional[Dict[str, int]]]] = {}

    for frame_key, lst in frames_index.items():
        floor_entry: Optional[Dict[str, int]] = None
        table_entry: Optional[Dict[str, int]] = None
        wall_entry: Optional[Dict[str, int]] = None

        for item in lst:
            tid = int(item["track_id"])
            local_id = int(item["local_id"])

            if floor_track_id is not None and tid == floor_track_id:
                floor_entry = {"track_id": tid, "local_id": local_id}
            if table_track_id is not None and tid == table_track_id:
                table_entry = {"track_id": tid, "local_id": local_id}
            if main_wall_track_id is not None and tid == main_wall_track_id:
                wall_entry = {"track_id": tid, "local_id": local_id}

        frames_out[frame_key] = {
            "floor": floor_entry,
            "table": table_entry,
            "main_wall": wall_entry,
        }

    return frames_out


def classify_tracks_stably(
    tracks_path: Path,
    confidence_path: Path,
    vertical_axis: np.ndarray,
    horiz_angle_max: float,
    min_table_height_diff: float,
    vertical_band_center: float,
    vertical_band_half_width: float,
    min_confidence: float,
) -> Dict[str, Any]:
    """
    Core process: load tracks + confidence, select floor/table/main wall,
    and build a stable classification payload (dict ready to be dumped as JSON).
    """
    tracks_data = _load_json(tracks_path)
    conf_data = _load_json(confidence_path)

    frames_index: Dict[str, List[Dict[str, int]]] = tracks_data.get(
        "frames", {}
    )
    confidence_map = _build_confidence_map(conf_data)

    infos = _compute_track_infos(
        tracks_data=tracks_data,
        vertical_axis=vertical_axis,
        confidence_map=confidence_map,
    )

    floor_id, table_id = _select_floor_and_table(
        infos=infos,
        horiz_angle_max=horiz_angle_max,
        min_height_diff=min_table_height_diff,
        min_confidence=min_confidence,
    )
    wall_id = _select_main_wall(
        infos=infos,
        vertical_band_center=vertical_band_center,
        vertical_band_half_width=vertical_band_half_width,
        min_confidence=min_confidence,
    )

    frames_out = _build_frame_classification(
        frames_index=frames_index,
        floor_track_id=floor_id,
        table_track_id=table_id,
        main_wall_track_id=wall_id,
    )

    tracks_summary: List[Dict[str, Any]] = []
    for ti in infos:
        tracks_summary.append(
            {
                "track_id": ti.track_id,
                "num_detections": ti.num_detections,
                "mean_normal": ti.mean_normal.tolist(),
                "mean_centroid": ti.mean_centroid.tolist(),
                "mean_inlier_ratio": ti.mean_inlier_ratio,
                "angle_to_vertical_deg": ti.angle_to_vertical_deg,
                "height_along_vertical": ti.height_along_vertical,
                "confidence_score": ti.confidence_score,
            }
        )

    payload: Dict[str, Any] = {
        "tracks_file": str(tracks_path),
        "confidence_file": str(confidence_path),
        "vertical_axis": vertical_axis.tolist(),
        "selected_tracks": {
            "floor_track_id": floor_id,
            "table_track_id": table_id,
            "main_wall_track_id": wall_id,
        },
        "tracks_summary": tracks_summary,
        "frames": frames_out,
    }

    return payload
