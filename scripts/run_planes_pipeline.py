#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Orchestrate the planes pipeline for a single experiment.

Given the name of the main experiment folder under data/zed_ply
(e.g. "home_02_T1_90_05_02"), this script will:

1) Run plane segmentation with Open3D (segment_planes_open3d.py)
2) Track planes temporally (track_planes_temporally.py)
3) Compute track confidence (compute_track_confidence.py)
4) Classify tracks into stable semantic classes (classify_tracks_stably.py)
5) Recolor point clouds according to the stable classes (recolor_by_stable_class.py)
6) Analyze temporal stability of the selected planes (analyze_temporal_stability.py)

Assumptions
-----------
- You have already exported the .ply frames from ROS2 as:
    data/zed_ply/<experiment_name>/
  For example:
    data/zed_ply/home_02_T1_90_05_02/

- The "base id" for the experiment (used in output/planes) is obtained from
  the first three components of the experiment name split by "_":
    experiment_name = "home_02_T1_90_05_02"
    base_id = "home_02_T1"

  So the planes output directory becomes:
    output/planes/home_02_T1_p8   (if --max_planes 8)

Notes
-----
- This script is intentionally thin and uses the existing CLI scripts via
  subprocess, to keep behavior identical to the commands you already tested.
- It must be executed from the repository root, inside the proper venv and
  with all dependencies installed.

Example
-------
    python scripts/run_planes_pipeline.py \\
        --experiment home_02_T1_90_05_02 \\
        --max_planes 8
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import List


def run_command(command: List[str]) -> None:
    """
    Run a shell command using subprocess.run and propagate failures.

    Parameters
    ----------
    command : list of str
        Command and arguments, e.g. ["python", "scripts/segment_planes_open3d.py", ...]
    """
    print(f"[RUN] {' '.join(command)}")
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as exc:
        print(f"[ERROR] Command failed with return code {exc.returncode}:")
        print(f"        {' '.join(command)}")
        sys.exit(exc.returncode)


def parse_args() -> argparse.Namespace:
    """
    Parse CLI arguments for the planes pipeline orchestrator.

    Returns
    -------
    argparse.Namespace
        Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Run the planes pipeline for a given experiment."
    )

    parser.add_argument(
        "--experiment",
        required=True,
        help=(
            "Name of the main experiment folder in data/zed_ply, e.g. "
            "'home_02_T1_90_05_02'."
        ),
    )

    parser.add_argument(
        "--max_planes",
        type=int,
        default=8,
        help="Maximum number of planes per frame (default: 8).",
    )

    parser.add_argument(
        "--topic",
        default="/zed/zed_node/point_cloud/cloud_registered",
        help=(
            "Point cloud topic used during export (only for documentation here). "
            "This script assumes .ply frames are already stored in data/zed_ply."
        ),
    )

    parser.add_argument(
        "--no_temporal_analysis",
        action="store_true",
        help="If set, skip the temporal stability analysis step.",
    )

    return parser.parse_args()


def derive_paths(experiment_name: str, max_planes: int) -> dict:
    """
    Derive all paths used in the pipeline from the experiment name.

    The experiment name is the directory under data/zed_ply, e.g.:
        'home_02_T1_90_05_02'

    The base id is the first three components:
        base_id = 'home_02_T1'

    Parameters
    ----------
    experiment_name : str
        Folder name under data/zed_ply containing the .ply frames.
    max_planes : int
        Maximum number of planes per frame (used in planes output folder name).

    Returns
    -------
    dict
        Dictionary with the main paths:
            - ply_dir
            - planes_dir
            - tracks_path
            - confidence_path
            - classification_path
            - json_planes_dir
    """
    experiment_parts = experiment_name.split("_")
    if len(experiment_parts) >= 3:
        base_id = "_".join(experiment_parts[:3])
    else:
        # Fallback: if naming convention changes, use the full name
        base_id = experiment_name

    ply_dir = Path("data") / "ply" / experiment_name
    planes_dir = Path("output") / "planes" / f"{base_id}_p{max_planes}"
    json_planes_dir = planes_dir / "json"
    tracks_path = planes_dir / "plane_tracks.json"
    confidence_path = planes_dir / "track_confidence.json"
    classification_path = planes_dir / "classification_stable.json"

    return {
        "ply_dir": ply_dir,
        "planes_dir": planes_dir,
        "json_planes_dir": json_planes_dir,
        "tracks_path": tracks_path,
        "confidence_path": confidence_path,
        "classification_path": classification_path,
    }


def main() -> None:
    """
    Main entry point for the planes pipeline orchestrator.
    """
    args = parse_args()
    paths = derive_paths(args.experiment, args.max_planes)

    ply_dir: Path = paths["ply_dir"]
    planes_dir: Path = paths["planes_dir"]
    json_planes_dir: Path = paths["json_planes_dir"]
    tracks_path: Path = paths["tracks_path"]
    confidence_path: Path = paths["confidence_path"]
    classification_path: Path = paths["classification_path"]

    # -------------------------------------------------------------------------
    # Sanity checks
    # -------------------------------------------------------------------------
    if not ply_dir.is_dir():
        print(
            f"[ERROR] PLY directory does not exist: {ply_dir}\n"
            "        Make sure you exported the frames with export_pointcloud2_to_ply.py."
        )
        sys.exit(1)

    print("[INFO] Experiment name     :", args.experiment)
    print("[INFO] Input PLY directory :", ply_dir)
    print("[INFO] Planes output dir   :", planes_dir)
    print("[INFO] Max planes per frame:", args.max_planes)

    # -------------------------------------------------------------------------
    # 1) Plane segmentation (Open3D)
    # -------------------------------------------------------------------------
    run_command(
        [
            sys.executable,
            "scripts/segment_planes_open3d.py",
            "--input",
            str(ply_dir),
            "--out_dir",
            str(planes_dir),
            "--max_planes",
            str(args.max_planes),
        ]
    )

    # -------------------------------------------------------------------------
    # 2) Temporal tracking of planes
    # -------------------------------------------------------------------------
    run_command(
        [
            sys.executable,
            "scripts/track_planes_temporally.py",
            "--input_dir",
            str(planes_dir),
            "--output",
            str(tracks_path),
        ]
    )

    # -------------------------------------------------------------------------
    # 3) Track confidence computation
    # -------------------------------------------------------------------------
    run_command(
        [
            sys.executable,
            "scripts/compute_track_confidence.py",
            "--tracks",
            str(tracks_path),
            "--output",
            str(confidence_path),
        ]
    )

    # -------------------------------------------------------------------------
    # 4) Stable semantic classification of tracks
    # -------------------------------------------------------------------------
    run_command(
        [
            sys.executable,
            "scripts/classify_tracks_stably.py",
            "--tracks",
            str(tracks_path),
            "--confidence",
            str(confidence_path),
            "--output",
            str(classification_path),
            # All other thresholds / vertical axis will use the defaults
            # defined inside classify_tracks_stably.py.
        ]
    )

    # -------------------------------------------------------------------------
    # 5) Recolor by stable class (chão, parede, mesa, etc.)
    # -------------------------------------------------------------------------
    run_command(
        [
            sys.executable,
            "scripts/recolor_by_stable_class.py",
            "--input_dir",
            str(ply_dir),
            "--classification",
            str(classification_path),
            "--out_dir",
            str(planes_dir),
        ]
    )

    # -------------------------------------------------------------------------
    # 6) Temporal stability analysis (optionally)
    # -------------------------------------------------------------------------
    if not args.no_temporal_analysis:
        if not json_planes_dir.is_dir():
            print(
                f"[WARN] json_planes directory not found: {json_planes_dir}\n"
                "       Skipping temporal stability analysis."
            )
        else:
            run_command(
                [
                    sys.executable,
                    "scripts/analyze_temporal_stability.py",
                    "--input_dir",
                    str(json_planes_dir),
                    "--out_dir",
                    str(planes_dir),
                ]
            )

    print("[INFO] Planes pipeline finished successfully.")


if __name__ == "__main__":
    main()
