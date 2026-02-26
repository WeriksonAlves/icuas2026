#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize sequences of point clouds as an animation with up to four views.

Typical usage (following the suggested repo layout):

  python scripts/visualize_pointcloud_sequence.py \
      --raw_dir data/ply/zed_outdoor_01 \
      --colored_dir output/planes/zed_outdoor_01/ply_colored \
      --semantic_dir output/planes/zed_outdoor_01/ply_stable_semantic \
      --fps 5 \
      --max_points 30000

Each directory must contain one .ply file per frame. Files are sorted
lexicographically (frame_00000.ply, planes_00000.ply, etc.) and the number
of frames used is the minimum among the provided views.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from matplotlib.animation import FuncAnimation


ViewSpec = Tuple[str, Path]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Animate point cloud sequences with up to four subplots "
            "(raw, colored, semantic, stable, etc.)."
        )
    )

    parser.add_argument(
        "--raw_dir",
        type=str,
        default=None,
        help="Directory with raw .ply frames (e.g., data/ply/zed_outdoor_01).",
    )
    parser.add_argument(
        "--colored_dir",
        type=str,
        default=None,
        help=(
            "Directory with per-plane colored .ply "
            "(e.g., output/planes/zed_outdoor_01/ply_colored)."
        ),
    )
    parser.add_argument(
        "--semantic_dir",
        type=str,
        default=None,
        help=(
            "Directory with semantic .ply (one-shot or unstable semantics)."
        ),
    )
    parser.add_argument(
        "--stable_dir",
        type=str,
        default=None,
        help=(
            "Directory with stable semantic .ply "
            "(e.g., output/planes/zed_outdoor_01/ply_stable_semantic)."
        ),
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=5.0,
        help="Frames per second for the animation (default: 5.0).",
    )
    parser.add_argument(
        "--max_frames",
        type=int,
        default=0,
        help=(
            "Maximum number of frames to animate (0 = use all common frames)."
        ),
    )
    parser.add_argument(
        "--max_points",
        type=int,
        default=40000,
        help=(
            "Maximum number of points per view/frame (downsample if needed)."
        ),
    )
    parser.add_argument(
        "--rotate_z_180",
        action="store_true",
        help="Apply 180-degree rotation around Z axis."
    )

    return parser.parse_args()


def _collect_views(args: argparse.Namespace) -> List[ViewSpec]:
    """Build the list of active views based on CLI arguments."""
    views: List[ViewSpec] = []

    if args.raw_dir is not None:
        views.append(("Raw", Path(args.raw_dir).expanduser().resolve()))
    if args.colored_dir is not None:
        views.append(("Colored", Path(args.colored_dir).expanduser().resolve()))
    if args.semantic_dir is not None:
        views.append(("Semantic", Path(args.semantic_dir).expanduser().resolve()))
    if args.stable_dir is not None:
        views.append(("Stable semantic", Path(args.stable_dir).expanduser().resolve()))

    if not views:
        raise SystemExit(
            "No input directory provided. "
            "Use at least one of --raw_dir, --colored_dir, --semantic_dir, "
            "--stable_dir."
        )
    if len(views) > 4:
        raise SystemExit("At most four views are supported.")

    return views


def _list_ply_frames(root: Path) -> List[Path]:
    """Return a sorted list of .ply files in a directory."""
    if not root.is_dir():
        raise FileNotFoundError(f"Directory not found: {root}")
    frames = sorted(root.glob("*.ply"))
    if not frames:
        raise FileNotFoundError(f"No .ply files found in: {root}")
    return frames


def _build_frame_index(
    views: List[ViewSpec],
    max_frames: int,
) -> Tuple[Dict[str, List[Path]], int]:
    """
    For each view, list PLY frames and determine the common number of frames.

    The animation length is the minimum number of frames among views,
    optionally capped by max_frames.
    """
    frame_paths: Dict[str, List[Path]] = {}
    lengths: List[int] = []

    for label, root in views:
        frames = _list_ply_frames(root)
        frame_paths[label] = frames
        lengths.append(len(frames))
        print(f"[INFO] View '{label}': {len(frames)} frames from {root}")

    if not lengths:
        raise RuntimeError("No frames found in any view.")

    num_frames = min(lengths)
    if max_frames > 0:
        num_frames = min(num_frames, max_frames)

    print(f"[INFO] Using {num_frames} frame(s) (min across views).")
    return frame_paths, num_frames


def _load_cloud(path: Path) -> Tuple[np.ndarray, np.ndarray | None]:
    """Load a .ply as (points, colors) using Open3D."""
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        return np.empty((0, 3), dtype=np.float64), None

    pts = np.asarray(pcd.points, dtype=np.float64)

    if pcd.has_colors():
        cols = np.asarray(pcd.colors, dtype=np.float64)
    else:
        cols = None

    return pts, cols


def _downsample(
    pts: np.ndarray,
    cols: np.ndarray | None,
    max_points: int,
) -> Tuple[np.ndarray, np.ndarray | None]:
    """Randomly subsample points (and colors) if needed."""
    n_pts = pts.shape[0]
    if max_points <= 0 or n_pts <= max_points:
        return pts, cols

    idx = np.random.choice(n_pts, size=max_points, replace=False)
    pts_ds = pts[idx]

    if cols is not None:
        cols_ds = cols[idx]
    else:
        cols_ds = None

    return pts_ds, cols_ds


def _compute_layout(n_views: int) -> Tuple[int, int]:
    """Choose a grid layout (rows, cols) for up to four views."""
    if n_views <= 1:
        return 1, 1
    if n_views == 2:
        return 1, 2
    # 3 or 4 views
    return 2, 2


def _rotate_z_180(pts: np.ndarray) -> np.ndarray:
    """
    Apply a 180-degree rotation around Z axis.
    Equivalent to (x, y, z) -> (-x, -y, z).
    """
    if pts.size == 0:
        return pts

    pts_rot = pts.copy()
    pts_rot[:, 0] *= -1.0
    pts_rot[:, 1] *= -1.0
    return pts_rot


def main() -> None:
    args = _parse_args()
    views = _collect_views(args)

    frame_paths, num_frames = _build_frame_index(views, args.max_frames)
    if num_frames <= 0:
        raise SystemExit("No frames available for animation.")

    n_views = len(views)
    rows, cols = _compute_layout(n_views)

    # Create figure and axes
    fig = plt.figure(figsize=(5 * cols, 5 * rows))
    axes: List[plt.Axes] = []

    for idx in range(n_views):
        ax = fig.add_subplot(rows, cols, idx + 1, projection="3d")
        axes.append(ax)

    fig.suptitle("Point cloud sequence (planes pipeline)", fontsize=16)

    # To keep aspect ratio approximately equal, we will recompute limits per frame
    def _set_equal_aspect(ax: plt.Axes, pts: np.ndarray) -> None:
        if pts.size == 0:
            return
        x_min, y_min, z_min = pts.min(axis=0)
        x_max, y_max, z_max = pts.max(axis=0)
        max_range = float(
            max(x_max - x_min, y_max - y_min, z_max - z_min)
        )
        if max_range <= 0.0:
            max_range = 1.0
        x_mid = (x_max + x_min) * 0.5
        y_mid = (y_max + y_min) * 0.5
        z_mid = (z_max + z_min) * 0.5

        half = 0.5 * max_range
        ax.set_xlim(x_mid - half, x_mid + half)
        ax.set_ylim(y_mid - half, y_mid + half)
        ax.set_zlim(z_mid - half, z_mid + half)
        try:
            ax.set_box_aspect((1.0, 1.0, 1.0))
        except Exception:
            # set_box_aspect not available in older Matplotlib versions
            pass

    def _update(frame_idx: int) -> None:
        fig.suptitle(
            f"Point cloud sequence (frame {frame_idx + 1}/{num_frames})",
            fontsize=16,
        )

        for ax, (label, root) in zip(axes, views):
            ax.cla()

            paths = frame_paths[label]
            path = paths[frame_idx]

            pts, cols = _load_cloud(path)
            pts, cols = _downsample(pts, cols, args.max_points)

            if args.rotate_z_180:
                pts = _rotate_z_180(pts)

            if pts.size == 0:
                ax.set_title(f"{label} (empty) - {path.name}")
                ax.set_xlabel("X")
                ax.set_ylabel("Y")
                ax.set_zlabel("Z")
                ax.grid(True, linestyle="--", alpha=0.3)
                continue

            if cols is not None:
                # Matplotlib expects colors in [0,1]
                ax.scatter(
                    pts[:, 0],
                    pts[:, 1],
                    pts[:, 2],
                    s=1,
                    c=cols,
                    marker=".",
                )
            else:
                ax.scatter(
                    pts[:, 0],
                    pts[:, 1],
                    pts[:, 2],
                    s=1,
                    c="k",
                    marker=".",
                )

            ax.set_title(f"{label} - {path.name}")
            ax.set_xlabel("X [m]")
            ax.set_ylabel("Y [m]")
            ax.set_zlabel("Z [m]")
            ax.grid(True, linestyle="--", alpha=0.3)
            ax.view_init(elev=20.0, azim=-60.0)
            _set_equal_aspect(ax, pts)

    # Build animation
    interval_ms = 1000.0 / max(args.fps, 1e-3)
    _ = FuncAnimation(
        fig,
        _update,
        frames=num_frames,
        interval=interval_ms,
        repeat=True,
    )

    print("[INFO] Starting interactive visualization window...")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.", file=sys.stderr)