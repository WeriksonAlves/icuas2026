# planes/track_confidence.py

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

import numpy as np


# ---------------------------------------------------------------------------
# Data Structures
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TrackConfidenceMetrics:
    track_id: int
    num_detections: int
    persistence_ratio: float
    std_angle_deg: float
    std_d: float
    std_inlier_ratio: float
    confidence_score: float


# ---------------------------------------------------------------------------
# Helper Math
# ---------------------------------------------------------------------------


def _safe_acos(x: float) -> float:
    return math.acos(max(min(x, 1.0), -1.0))


def angle_between_normals_deg(n1: np.ndarray, n2: np.ndarray) -> float:
    n1 = n1 / (np.linalg.norm(n1) + 1e-12)
    n2 = n2 / (np.linalg.norm(n2) + 1e-12)
    cos_theta = abs(float(n1.dot(n2)))
    return math.degrees(_safe_acos(cos_theta))


# ---------------------------------------------------------------------------
# Core Computation
# ---------------------------------------------------------------------------


def compute_track_confidence(
    tracks_data: Dict,
    total_frames: int,
    w_angle: float = 0.35,
    w_d: float = 0.20,
    w_persistence: float = 0.30,
    w_inlier: float = 0.15,
) -> List[TrackConfidenceMetrics]:
    """
    Compute confidence metrics for each track.
    """

    tracks = tracks_data.get("tracks", [])
    results: List[TrackConfidenceMetrics] = []

    for tr in tracks:
        track_id = int(tr["track_id"])
        detections = tr.get("detections", [])
        num_det = len(detections)

        if num_det < 2:
            continue

        normals = np.array([d["normal"] for d in detections], dtype=np.float64)
        ds = np.array([d["d"] for d in detections], dtype=np.float64)
        inliers = np.array(
            [d.get("inlier_ratio_total", 0.0) for d in detections],
            dtype=np.float64,
        )

        # Angular deviation relative to first normal
        ref_n = normals[0]
        angles = np.array(
            [angle_between_normals_deg(ref_n, n) for n in normals]
        )

        std_angle = float(np.std(angles))
        std_d = float(np.std(ds))
        std_inlier = float(np.std(inliers))

        persistence = float(num_det / max(1, total_frames))

        # Normalized stability components
        angle_score = 1.0 / (1.0 + std_angle)
        d_score = 1.0 / (1.0 + std_d)
        inlier_score = 1.0 / (1.0 + std_inlier)

        confidence = (
            w_angle * angle_score
            + w_d * d_score
            + w_persistence * persistence
            + w_inlier * inlier_score
        )

        results.append(
            TrackConfidenceMetrics(
                track_id=track_id,
                num_detections=num_det,
                persistence_ratio=persistence,
                std_angle_deg=std_angle,
                std_d=std_d,
                std_inlier_ratio=std_inlier,
                confidence_score=float(confidence),
            )
        )

    return results
