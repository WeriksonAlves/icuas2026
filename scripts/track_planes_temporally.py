#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# scripts/track_planes_temporally.py
"""CLI front-end for temporal tracking of planes across frames."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from planes.tracking import TrackerConfig, run_tracking


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Temporal tracking of planes across frames.",
    )
    parser.add_argument(
        "--input_dir",
        required=True,
        help=(
            "Directory with planes_XXXXX.json files "
            "(from segment_planes_open3d.py)."
        ),
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
        help="Max allowed centroid distance to associate (m).",
    )
    parser.add_argument(
        "--max_d_diff",
        type=float,
        default=0.7,
        help="Max allowed difference in plane offset d (m).",
    )
    parser.add_argument(
        "--max_frame_gap",
        type=int,
        default=5,
        help="Max temporal gap (in frames) to keep a track active.",
    )
    parser.add_argument(
        "--min_track_length",
        type=int,
        default=3,
        help="Minimum number of detections to keep a track.",
    )
    parser.add_argument(
        "--ema_alpha",
        type=float,
        default=0.8,
        help="EMA coefficient for track state smoothing (0<alpha<=1).",
    )
    parser.add_argument(
        "--w_angle",
        type=float,
        default=1.0,
        help="Weight for angular component in association cost.",
    )
    parser.add_argument(
        "--w_centroid",
        type=float,
        default=2.0,
        help="Weight for centroid distance in association cost.",
    )
    parser.add_argument(
        "--w_d",
        type=float,
        default=1.0,
        help="Weight for plane offset difference in association cost.",
    )
    return parser


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    output_path = Path(args.output).resolve()

    cfg = TrackerConfig(
        max_angle_deg=float(args.max_angle_deg),
        max_centroid_dist=float(args.max_centroid_dist),
        max_d_diff=float(args.max_d_diff),
        max_frame_gap=int(args.max_frame_gap),
        min_track_length=int(args.min_track_length),
        ema_alpha=float(args.ema_alpha),
        w_angle=float(args.w_angle),
        w_centroid=float(args.w_centroid),
        w_d=float(args.w_d),
    )

    print(f"[INFO] Loading detections from: {input_dir}")
    summary = run_tracking(input_dir, cfg)

    print(
        "[INFO] Built "
        f"{summary['num_tracks']} tracks "
        f"(min_length={cfg.min_track_length})."
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"[INFO] Tracks written to: {output_path}")


if __name__ == "__main__":
    main()
