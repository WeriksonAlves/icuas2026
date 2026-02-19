#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Temporal plane tracking module.

Reads planes_XXXXX.json files produced by segment_planes_open3d.py and builds
temporal tracks of planes across frames.

Association between detections and tracks is based on:
- angular difference between normals
- Euclidean distance between centroids
- difference in plane offset d

Outputs a single JSON file with:
- list of tracks and their per-frame detections
- summary metrics per track (mean normal, mean centroid, mean inlier ratio, etc.)
- frame-to-track associations (which local plane in each frame belongs to which track)

This module is intentionally generic: it does not impose floor/wall semantics,
but it exposes enough information so that a higher-level component can choose
which track to treat as "main wall", "floor", etc.
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
# Data structures
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PlaneDetection:
    """Single plane detection in a given frame."""
    frame_idx: int
    local_id: int  # plane_id inside that frame
    normal: np.ndarray  # shape (3,)
    d: float
    centroid: np.ndarray  # shape (3,)
    inlier_ratio_total: float


@dataclass
class PlaneTrack:
    """
    Temporal track of a plane across frames.

    The "state" (normal, d, centroid) is updated with an exponential moving
    average for robustness.
    """
    track_id: int
    detections: List[PlaneDetection]
    last_frame_idx: int
    normal: np.ndarray
    d: float
    centroid: np.ndarray
    num_updates: int

    def update(self, det: PlaneDetection, alpha: float = 0.8) -> None:
        """
        Update track state with a new detection.

        Parameters
        ----------
        det : PlaneDetection
            New observation to merge into the track.
        alpha : float
            EMA coefficient (0 < alpha <= 1). Larger alpha => more inertia.
        """
        self.detections.append(det)
        self.last_frame_idx = det.frame_idx
        self.num_updates += 1

        # Exponential moving average on normal, d and centroid
        n_new = alpha * self.normal + (1.0 - alpha) * det.normal
        n_norm = np.linalg.norm(n_new)
        if n_norm > 1e-9:
            n_new /= n_norm

        self.normal = n_new
        self.d = float(alpha * self.d + (1.0 - alpha) * det.d)
        self.centroid = alpha * self.centroid + (1.0 - alpha) * det.centroid


# ---------------------------------------------------------------------------
# Helper math
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


# ---------------------------------------------------------------------------
# Loading detections
# ---------------------------------------------------------------------------


def load_detections(input_dir: Path) -> Dict[int, List[PlaneDetection]]:
    """
    Load plane detections from planes_XXXXX.json files.

    Returns
    -------
    Dict[int, List[PlaneDetection]]
        Mapping: frame_idx -> list of detections in that frame.
    """
    json_paths = sorted(input_dir.glob("planes_*.json"))
    if not json_paths:
        raise FileNotFoundError(f"No planes_XXXXX.json found in: {input_dir}")

    detections_by_frame: Dict[int, List[PlaneDetection]] = {}

    for path in json_paths:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        frame_idx = int(data.get("frame_idx", -1))
        planes = data.get("planes", [])
        if not planes:
            continue

        frame_list: List[PlaneDetection] = []
        for p in planes:
            local_id = int(p.get("plane_id", -1))
            model = p["model"]
            normal = np.asarray(model["n"], dtype=np.float64)
            d = float(model["d"])
            centroid = np.asarray(p["centroid"], dtype=np.float64)

            # For compatibility: prefer inlier_ratio_total if present.
            ir = float(p.get("inlier_ratio_total", p.get("inlier_ratio", 0.0)))

            det = PlaneDetection(
                frame_idx=frame_idx,
                local_id=local_id,
                normal=normal,
                d=d,
                centroid=centroid,
                inlier_ratio_total=ir,
            )
            frame_list.append(det)

        detections_by_frame[frame_idx] = frame_list

    return detections_by_frame


# ---------------------------------------------------------------------------
# Tracker
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TrackerConfig:
    max_angle_deg: float = 15.0
    max_centroid_dist: float = 0.5
    max_d_diff: float = 0.7
    max_frame_gap: int = 5
    min_track_length: int = 3
    ema_alpha: float = 0.8


class PlaneTracker:
    """Greedy plane tracker based on geometric compatibility."""

    def __init__(self, cfg: TrackerConfig) -> None:
        self.cfg = cfg
        self._next_track_id: int = 0
        self._tracks: List[PlaneTrack] = []

    @property
    def tracks(self) -> List[PlaneTrack]:
        return self._tracks

    def _create_track(self, det: PlaneDetection) -> PlaneTrack:
        track = PlaneTrack(
            track_id=self._next_track_id,
            detections=[det],
            last_frame_idx=det.frame_idx,
            normal=det.normal.copy(),
            d=det.d,
            centroid=det.centroid.copy(),
            num_updates=1,
        )
        self._next_track_id += 1
        self._tracks.append(track)
        return track

    def _compute_cost(
        self,
        track: PlaneTrack,
        det: PlaneDetection,
    ) -> Tuple[float, float, float]:
        """Return (angle_deg, centroid_dist, d_diff)."""
        angle = angle_between_normals_deg(track.normal, det.normal)
        cdist = float(np.linalg.norm(track.centroid - det.centroid))
        ddiff = abs(track.d - det.d)
        return angle, cdist, ddiff

    def _is_compatible(
        self,
        angle: float,
        cdist: float,
        ddiff: float,
    ) -> bool:
        return (
            angle <= self.cfg.max_angle_deg
            and cdist <= self.cfg.max_centroid_dist
            and ddiff <= self.cfg.max_d_diff
        )

    def update_frame(self, frame_idx: int, detections: List[PlaneDetection]) -> None:
        """
        Update tracker with detections from a single frame.

        Parameters
        ----------
        frame_idx : int
            Index of current frame.
        detections : List[PlaneDetection]
            List of plane detections in this frame.
        """
        if not detections:
            return

        # Active tracks: those not too far in time.
        active_tracks: List[PlaneTrack] = [
            tr
            for tr in self._tracks
            if frame_idx - tr.last_frame_idx <= self.cfg.max_frame_gap
        ]

        # Greedy association: for each detection, pick best compatible track.
        used_tracks: set[int] = set()

        for det in detections:
            best_track: Optional[PlaneTrack] = None
            best_cost: float = float("inf")

            for tr in active_tracks:
                if tr.track_id in used_tracks:
                    continue

                angle, cdist, ddiff = self._compute_cost(tr, det)
                if not self._is_compatible(angle, cdist, ddiff):
                    continue

                # Simple scalar cost; you can tweak weights if needed.
                cost = angle + 2.0 * cdist + 1.0 * ddiff
                if cost < best_cost:
                    best_cost = cost
                    best_track = tr

            if best_track is not None:
                best_track.update(det, alpha=self.cfg.ema_alpha)
                used_tracks.add(best_track.track_id)
            else:
                # No compatible track found -> new plane track.
                self._create_track(det)

    def finalize_tracks(self) -> List[PlaneTrack]:
        """
        Return only tracks that are long enough (min_track_length).

        This does not modify internal state, just filters the list.
        """
        return [
            tr
            for tr in self._tracks
            if len(tr.detections) >= self.cfg.min_track_length
        ]


# ---------------------------------------------------------------------------
# Serialization
# ---------------------------------------------------------------------------


def _track_to_dict(track: PlaneTrack) -> Dict:
    """Serialize a PlaneTrack (including its detections) into plain Python types."""
    det_list = []
    for det in track.detections:
        det_list.append(
            {
                "frame_idx": det.frame_idx,
                "local_id": det.local_id,
                "normal": det.normal.tolist(),
                "d": det.d,
                "centroid": det.centroid.tolist(),
                "inlier_ratio_total": det.inlier_ratio_total,
            }
        )

    inlier_ratios = np.array(
        [det.inlier_ratio_total for det in track.detections], dtype=np.float64
    )

    # Aggregate summary for convenience
    mean_normal = np.mean(
        np.vstack([det.normal for det in track.detections]),
        axis=0,
    )
    n_norm = np.linalg.norm(mean_normal)
    if n_norm > 1e-9:
        mean_normal /= n_norm

    mean_centroid = np.mean(
        np.vstack([det.centroid for det in track.detections]),
        axis=0,
    )

    return {
        "track_id": track.track_id,
        "num_detections": len(track.detections),
        "last_frame_idx": track.last_frame_idx,
        "state": {
            "normal": track.normal.tolist(),
            "d": track.d,
            "centroid": track.centroid.tolist(),
        },
        "summary": {
            "mean_normal": mean_normal.tolist(),
            "mean_d": float(np.mean([det.d for det in track.detections])),
            "mean_centroid": mean_centroid.tolist(),
            "mean_inlier_ratio_total": float(np.mean(inlier_ratios)),
            "std_inlier_ratio_total": float(np.std(inlier_ratios)),
        },
        "detections": det_list,
    }


def _build_frame_index(tracks: List[PlaneTrack]) -> Dict[str, List[Dict]]:
    """
    Build a frame-wise index:
      frame_idx -> list of {track_id, local_id} appearing in that frame.
    """
    frames: Dict[int, List[Dict]] = {}
    for tr in tracks:
        for det in tr.detections:
            lst = frames.setdefault(det.frame_idx, [])
            lst.append(
                {
                    "track_id": tr.track_id,
                    "local_id": det.local_id,
                }
            )
    # Convert keys to strings for JSON
    return {str(k): v for k, v in sorted(frames.items(), key=lambda kv: kv[0])}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Temporal tracking of planes across frames.",
    )
    parser.add_argument(
        "--input_dir",
        required=True,
        help="Directory with planes_XXXXX.json files (from segment_planes_open3d.py).",
    )
    parser.add_argument(
        "--output",
        default="plane_tracks.json",
        help="Output JSON file with tracks and frame index.",
    )
    parser.add_argument(
        "--max_angle_deg",
        type=float,
        default=15.0,
        help="Max allowed angle between normals to associate (deg).",
    )
    parser.add_argument(
        "--max_centroid_dist",
        type=float,
        default=0.5,
        help="Max allowed centroid distance to associate (meters).",
    )
    parser.add_argument(
        "--max_d_diff",
        type=float,
        default=0.7,
        help="Max allowed difference in d to associate (meters).",
    )
    parser.add_argument(
        "--max_frame_gap",
        type=int,
        default=5,
        help="Max frame gap to keep a track active.",
    )
    parser.add_argument(
        "--min_track_length",
        type=int,
        default=3,
        help="Minimum number of detections for a track to be kept.",
    )
    parser.add_argument(
        "--ema_alpha",
        type=float,
        default=0.8,
        help="EMA coefficient for track state smoothing (0<alpha<=1).",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    input_dir = Path(args.input_dir).resolve()
    output_path = Path(args.output).resolve()

    cfg = TrackerConfig(
        max_angle_deg=float(args.max_angle_deg),
        max_centroid_dist=float(args.max_centroid_dist),
        max_d_diff=float(args.max_d_diff),
        max_frame_gap=int(args.max_frame_gap),
        min_track_length=int(args.min_track_length),
        ema_alpha=float(args.ema_alpha),
    )

    print(f"[INFO] Loading detections from: {input_dir}")
    dets_by_frame = load_detections(input_dir)
    frame_indices = sorted(dets_by_frame.keys())
    print(f"[INFO] Found {len(frame_indices)} frames with detections.")

    tracker = PlaneTracker(cfg)

    for frame_idx in frame_indices:
        frame_dets = dets_by_frame.get(frame_idx, [])
        print(f"[INFO] Frame {frame_idx}: {len(frame_dets)} detections")
        tracker.update_frame(frame_idx, frame_dets)

    tracks_all = tracker.finalize_tracks()
    print(f"[INFO] Built {len(tracks_all)} tracks (after length filtering).")

    tracks_payload = [_track_to_dict(tr) for tr in tracks_all]
    frame_index = _build_frame_index(tracks_all)

    payload = {
        "input_dir": str(input_dir),
        "config": asdict(cfg),
        "num_tracks": len(tracks_all),
        "tracks": tracks_payload,
        "frames": frame_index,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print(f"[INFO] Tracks written to: {output_path}")


if __name__ == "__main__":
    main()
