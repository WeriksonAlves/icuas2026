#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# scripts/segment_planes_open3d.py
"""CLI front-end for multi-plane segmentation with Open3D.

This script parses command-line arguments and delegates all heavy work
to :mod:`planes.segmentation`. It is intentionally small to keep
application logic inside the library code, which is easier to test and
reuse from notebooks or other tools.
"""

from __future__ import annotations

import argparse

from planes.segmentation import SegConfig, run_segmentation


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Iterative plane segmentation (Open3D RANSAC).",
    )
    parser.add_argument(
        "--input",
        required=True,
        help="Input .ply file or directory with .ply frames.",
    )
    parser.add_argument(
        "--out_dir",
        required=True,
        help="Output directory for results.",
    )
    parser.add_argument(
        "--preset",
        default="indoor",
        choices=["indoor", "outdoor"],
        help="Parameter preset.",
    )
    parser.add_argument(
        "--max_planes",
        type=int,
        default=4,
        help="Max number of planes to extract per frame.",
    )
    parser.add_argument(
        "--min_inliers",
        type=int,
        default=1500,
        help="Minimum inliers to accept a plane.",
    )
    parser.add_argument(
        "--distance_threshold",
        type=float,
        default=-1.0,
        help=(
            "RANSAC distance threshold (m). "
            "If <= 0, a preset-dependent default is used."
        ),
    )
    parser.add_argument(
        "--ransac_n",
        type=int,
        default=3,
        help="RANSAC sample size.",
    )
    parser.add_argument(
        "--num_iterations",
        type=int,
        default=2000,
        help="RANSAC iterations.",
    )
    parser.add_argument(
        "--sor_nb_neighbors",
        type=int,
        default=20,
        help="Number of neighbors for statistical outlier removal.",
    )
    parser.add_argument(
        "--sor_std_ratio",
        type=float,
        default=-1.0,
        help=(
            "Std ratio for statistical outlier removal. "
            "If <= 0, a preset-dependent default is used."
        ),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=7,
        help="Random seed for deterministic coloring.",
    )
    return parser


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()
    cfg = SegConfig.from_cli_args(args)

    print(f"[INFO] Input: {cfg.input_path}")
    print(f"[INFO] Output dir: {cfg.out_dir}")
    print(f"[INFO] Preset: {cfg.preset}")

    summary = run_segmentation(cfg)

    print(
        "[INFO] Done. "
        f"Frames: {summary['processed_frames']}/{summary['num_frames']} "
        f"Summary file: {cfg.out_dir / 'summary.json'}"
    )


if __name__ == "__main__":
    main()
