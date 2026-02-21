#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# scripts/classify_tracks_stably.py
"""CLI front-end for stable semantic classification of plane tracks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from planes.classification import ClassificationConfig, run_classification


def _build_arg_parser() -> argparse.ArgumentParser:
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
            "Maximum angle (deg) between plane normal and vertical axis "
            "to consider the plane horizontal (floor/table candidates)."
        ),
    )
    parser.add_argument(
        "--min_table_height_diff",
        type=float,
        default=0.20,
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
    return parser


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()

    tracks_path = Path(args.tracks).resolve()
    output_path = Path(args.output).resolve()

    vertical_axis = tuple(float(v) for v in args.vertical_axis)

    cfg = ClassificationConfig(
        vertical_axis=vertical_axis,
        horiz_angle_max=float(args.horiz_angle_max),
        min_table_height_diff=float(args.min_table_height_diff),
        vertical_band_center=float(args.vertical_band_center),
        vertical_band_half_width=float(args.vertical_band_half_width),
    )

    print(f"[INFO] Loading tracks from: {tracks_path}")
    print(f"[INFO] Vertical axis: {np.array(vertical_axis, dtype=float)}")

    payload = run_classification(tracks_path, cfg)

    sel = payload["selected_tracks"]
    print(f"[INFO] floor_track_id = {sel['floor_track_id']}")
    print(f"[INFO] table_track_id = {sel['table_track_id']}")
    print(f"[INFO] main_wall_track_id = {sel['main_wall_track_id']}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print(f"[INFO] Stable classification written to: {output_path}")


if __name__ == "__main__":
    main()
