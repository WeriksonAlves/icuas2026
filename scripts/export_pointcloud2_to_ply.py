#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# scripts/export_pointcloud2_to_ply.py
"""CLI front-end to export ROS 2 PointCloud2 frames to PLY with Open3D."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from planes.ros_export import ExportConfig, run_export


def _parse_args() -> ExportConfig:
    parser = argparse.ArgumentParser(
        description="Export PointCloud2 frames to PLY using Open3D.",
    )
    parser.add_argument(
        "--topic",
        required=True,
        help="PointCloud2 topic name.",
    )
    parser.add_argument(
        "--out_dir",
        required=True,
        help="Output directory for .ply files.",
    )
    parser.add_argument(
        "--max_frames",
        type=int,
        default=50,
        help="Number of frames to save (default: 50).",
    )
    parser.add_argument(
        "--every_k",
        type=int,
        default=10,
        help="Save one frame every K received messages (default: 10).",
    )
    parser.add_argument(
        "--voxel",
        type=float,
        default=0.02,
        help="Voxel size in meters (use 0 to disable downsampling).",
    )
    parser.add_argument(
        "--keep_nans",
        action="store_true",
        help="Do not remove NaN/Inf points before exporting.",
    )

    args = parser.parse_args()

    out_dir = Path(os.path.expanduser(args.out_dir)).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    return ExportConfig(
        topic=args.topic,
        out_dir=out_dir,
        max_frames=max(1, args.max_frames),
        every_k=max(1, args.every_k),
        voxel=max(0.0, args.voxel),
        remove_nans=not args.keep_nans,
    )


def main() -> None:
    cfg = _parse_args()

    print(f"[INFO] Topic: {cfg.topic}")
    print(f"[INFO] Output dir: {cfg.out_dir}")
    print(f"[INFO] Max frames: {cfg.max_frames}")
    print(f"[INFO] Every K messages: {cfg.every_k}")
    print(f"[INFO] Voxel size: {cfg.voxel:.4f} m")
    print(f"[INFO] Remove NaNs: {cfg.remove_nans}")

    run_export(cfg)


if __name__ == "__main__":
    main()
