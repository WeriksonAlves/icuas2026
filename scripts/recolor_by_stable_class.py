#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# scripts/recolor_by_stable_class.py
"""CLI front-end for recoloring point clouds with stable plane classes."""

from __future__ import annotations

import argparse
from pathlib import Path
import json

from planes.recolor import RecolorConfig, run_recolor


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Recolor point clouds using stable plane classification "
            "(floor/table/main_wall with fixed colors)."
        )
    )
    parser.add_argument(
        "--input_dir",
        required=True,
        help="Directory with frame_XXXXX.ply point clouds.",
    )
    parser.add_argument(
        "--classification",
        required=True,
        help="classification_stable.json file from classify_tracks_stably.py.",
    )
    parser.add_argument(
        "--out_dir",
        required=True,
        help="Output directory for recolored point clouds.",
    )
    parser.add_argument(
        "--distance_threshold",
        type=float,
        default=0.04,
        help="Max distance (m) to assign a point to a plane (default: 0.04).",
    )
    parser.add_argument(
        "--summary",
        default="recolor_summary.json",
        help="Optional JSON file to store a processing summary.",
    )
    return parser


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()

    cfg = RecolorConfig.from_cli_args(args)

    print(f"[INFO] Input dir: {cfg.input_dir}")
    print(f"[INFO] Classification: {cfg.classification_path}")
    print(f"[INFO] Output dir: {cfg.out_dir}")
    print(f"[INFO] Distance threshold: {cfg.distance_threshold:.4f} m")

    summary = run_recolor(cfg)

    print(
        "[INFO] Recolored "
        f"{summary['num_written']}/{summary['num_frames']} frames "
        f"to {cfg.out_dir}"
    )
    if summary["missing_input_frames"]:
        print(
            "[WARN] Missing input frames: "
            f"{sorted(summary['missing_input_frames'])}"
        )
    if summary["empty_cloud_frames"]:
        print(
            "[WARN] Empty input clouds: "
            f"{sorted(summary['empty_cloud_frames'])}"
        )

    # Optional summary file, if requested
    summary_path = Path(args.summary).expanduser().resolve()
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(f"[INFO] Summary written to: {summary_path}")


if __name__ == "__main__":
    main()
