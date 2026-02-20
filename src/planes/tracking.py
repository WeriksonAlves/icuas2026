# src/planes/tracking.py
"""Plane tracking utilities.

This module implements temporal tracking of planar segments across
frames, using geometric compatibility between detections:

- angular difference between normals
- Euclidean distance between centroids
- difference in plane offset d

The main public entry point is :func:`run_tracking`, which loads
detections from ``planes_XXXXX.json`` files (produced by the
segmentation step), builds tracks, and returns a summary dictionary
that can be serialized to JSON by a CLI front-end or a notebook.
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

    The state (normal, d, centroid) is updated with an exponential
    moving average for robustness against noise and small tracking
    errors.
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
        det :
            New observation to merge into the track.
        alpha :
            EMA coefficient (0 < alpha <= 1). Larger alpha means more
            inertia (slower changes).
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
        self.centroid = (
            alpha * self.centroid + (1.0 - alpha) * det.centroid
        )


@dataclass(frozen=True)
class TrackerConfig:
    """Configuration parameters for temporal plane tracking."""

    max_angle_deg: float = 15.0
    max_centroid_dist: float = 0.5
    max_d_diff: float = 0.7
    max_frame_gap: int = 5
    min_track_length: int = 3
    ema_alpha: float = 0.8
    # Optional cost weights for multi-criteria association
    w_angle: float = 1.0
    w_centroid: float = 2.0
    w_d: float = 1.0


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def angle_between_normals_deg(n1: np.ndarray, n2: np.ndarray) -> float:
    """Return angle in degrees between two normals (ignoring sign)."""
    v1 = np.asarray(n1, dtype=np.float64)
    v2 = np.asarray(n2, dtype=np.float64)
    n1_norm = np.linalg.norm(v1)
    n2_norm = np.linalg.norm(v2)
    if n1_norm < 1e-9 or n2_norm < 1e-9:
        return float("nan")

    cos_theta = abs(float(v1.dot(v2)) / (n1_norm * n2_norm))
    return math.degrees(safe_acos(cos_theta))


# ---------------------------------------------------------------------------
# Loading detections
# ---------------------------------------------------------------------------


def load_detections(input_dir: Path) -> Dict[int, List[PlaneDetection]]:
    """
    Load plane detections from ``planes_XXXXX.json`` files.

    Returns
    -------
    dict
        Mapping: frame_idx -> list of detections in that frame.
    """
    detections_by_frame: Dict[int, List[PlaneDetection]] = {}

    json_paths = sorted(input_dir.glob("planes_*.json"))
    if not json_paths:
        raise FileNotFoundError(
            f"No planes_XXXXX.json files found in: {input_dir}"
        )

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
            ir = float(
                p.get("inlier_ratio_total", p.get("inlier_ratio", 0.0))
            )

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
# Core tracker
# ---------------------------------------------------------------------------


class PlaneTracker:
    """Greedy plane tracker based on geometric compatibility."""

    def __init__(self, cfg: TrackerConfig) -> None:
        self.cfg = cfg
        self._next_track_id: int = 0
        self._tracks: List[PlaneTrack] = []

    @property
    def tracks(self) -> List[PlaneTrack]:
        """Return current list of tracks (including short ones)."""
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
        """Return (angle_deg, centroid_dist, d_diff) for a candidate pair."""
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
        """Check if a candidate association satisfies hard thresholds."""
        return (
            angle <= self.cfg.max_angle_deg
            and cdist <= self.cfg.max_centroid_dist
            and ddiff <= self.cfg.max_d_diff
        )

    def update_frame(
        self,
        frame_idx: int,
        detections: List[PlaneDetection],
    ) -> None:
        """
        Update tracker with detections from a single frame.

        Parameters
        ----------
        frame_idx :
            Index of current frame.
        detections :
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

                # Multi-criteria scalar cost with configurable weights.
                cost = (
                    self.cfg.w_angle * angle
                    + self.cfg.w_centroid * cdist
                    + self.cfg.w_d * ddiff
                )
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
# Serialization helpers
# ---------------------------------------------------------------------------


def _track_to_dict(track: PlaneTrack) -> Dict:
    """Serialize a PlaneTrack (including its detections) into Python types."""
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
        [det.inlier_ratio_total for det in track.detections],
        dtype=np.float64,
    )

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
        "state": {
            "normal": track.normal.tolist(),
            "d": track.d,
            "centroid": track.centroid.tolist(),
        },
        "summary": {
            "mean_normal": mean_normal.tolist(),
            "mean_d": float(
                np.mean([det.d for det in track.detections])
            ),
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
# High-level orchestration
# ---------------------------------------------------------------------------


def run_tracking(input_dir: Path, cfg: TrackerConfig) -> Dict:
    """
    Run plane tracking on all frames in *input_dir*.

    Parameters
    ----------
    input_dir :
        Directory containing ``planes_XXXXX.json`` files generated by
        the segmentation step.
    cfg :
        Tracking configuration.

    Returns
    -------
    dict
        Summary dictionary with the same structure previously written
        to ``plane_tracks.json`` by the standalone script.
    """
    detections_by_frame = load_detections(input_dir)
    frame_indices = sorted(detections_by_frame.keys())

    tracker = PlaneTracker(cfg)

    for frame_idx in frame_indices:
        frame_dets = detections_by_frame.get(frame_idx, [])
        tracker.update_frame(frame_idx, frame_dets)

    tracks_all = tracker.finalize_tracks()
    tracks_payload = [_track_to_dict(tr) for tr in tracks_all]
    frame_index = _build_frame_index(tracks_all)

    return {
        "input_dir": str(input_dir),
        "config": asdict(cfg),
        "num_tracks": len(tracks_all),
        "tracks": tracks_payload,
        "frames": frame_index,
    }
