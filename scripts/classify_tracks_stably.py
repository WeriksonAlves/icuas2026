#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CLI wrapper for stable semantic classification of plane tracks.

This script:
  - parses command-line arguments
  - calls planes.semantic_classification.classify_tracks_stably
  - writes the resulting JSON file to disk
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import numpy as np

from planes.semantic_classification import classify_tracks_stably


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Assign stable semantic labels (floor, table, main wall) "
            "to plane tracks using confidence scores."
        )
    )
    parser.add_argument(
        "--tracks",
        required=True,
        help="Input plane_tracks.json (from track_planes_temporally.py).",
    )
    parser.add_argument(
        "--confidence",
        required=True,
        help="track_confidence.json (from compute_track_confidence.py).",
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
    parser.add_argument(
        "--min_confidence",
        type=float,
        default=0.3,
        help="Minimum confidence score for a track to be eligible.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    args = _parse_args(argv)

    tracks_path = Path(args.tracks).resolve()
    conf_path = Path(args.confidence).resolve()
    output_path = Path(args.output).resolve()

    vertical_axis = np.array(args.vertical_axis, dtype=np.float64)

    print(f"[INFO] Tracks file      : {tracks_path}")
    print(f"[INFO] Confidence file  : {conf_path}")
    print(f"[INFO] Output file      : {output_path}")
    print(f"[INFO] Vertical axis    : {vertical_axis.tolist()}")
    print(f"[INFO] min_confidence   : {args.min_confidence}")

    payload = classify_tracks_stably(
        tracks_path=tracks_path,
        confidence_path=conf_path,
        vertical_axis=vertical_axis,
        horiz_angle_max=float(args.horiz_angle_max),
        min_table_height_diff=float(args.min_table_height_diff),
        vertical_band_center=float(args.vertical_band_center),
        vertical_band_half_width=float(args.vertical_band_half_width),
        min_confidence=float(args.min_confidence),
    )

    # Some logging for quick feedback
    selected = payload.get("selected_tracks", {})
    print(
        "[INFO] Selected tracks -> "
        f"floor={selected.get('floor_track_id')}, "
        f"table={selected.get('table_track_id')}, "
        f"main_wall={selected.get('main_wall_track_id')}"
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print(f"[INFO] Stable classification written to: {output_path}")


if __name__ == "__main__":
    main()
