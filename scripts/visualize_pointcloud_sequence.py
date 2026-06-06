#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize point-cloud sequences as a synchronized animation.

The script supports up to four synchronized views, for example raw,
segmented/colored, semantic and stable semantic point clouds. It also supports
presentation-oriented coordinate remapping, fixed camera limits and GIF export.

Example:

  python scripts/visualize_pointcloud_sequence.py \
    --raw_dir data/ply/zed_outdoor_02_T2_279_01_05 \
    --colored_dir output/planes/zed_outdoor_02_T3_p3/ply_colored \
    --fps 5 \
    --max_points 100000 \
    --coordinate_frame presentation \
    --limit_mode global \
    --bounds_sample_step 5 \
    --view_elev 18 \
    --view_azim -90 \
    --point_size 0.8 \
    --hide_axes \
    --save_gif output/planes/zed_outdoor_02_T3_p3/segmentation_preview.gif
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  # Required to register projection="3d".


ViewSpec = Tuple[str, Path]
AxisLimits = Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]


_VIEW_LABELS = {
    "Raw": "Raw cloud",
    "Colored": "Segmented planes",
    "Semantic": "Semantic planes",
    "Stable semantic": "Stable semantic planes",
}


_AXIS_LABELS = {
    "native": ("X [m]", "Y [m]", "Z [m]"),
    # Presentation frame used only for visualization:
    #   plot_x = real Y
    #   plot_y = real X  (depth axis / approximately out of screen)
    #   plot_z = real Z  (vertical axis on screen)
    "presentation": ("Y_real [m]", "X_real [m]", "Z_real [m]"),
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Animate point-cloud sequences with up to four synchronized "
            "subplots: raw, colored, semantic and stable semantic."
        )
    )

    parser.add_argument(
        "--raw_dir",
        type=str,
        default=None,
        help="Directory with raw .ply frames.",
    )
    parser.add_argument(
        "--colored_dir",
        type=str,
        default=None,
        help="Directory with segmented/colored .ply frames.",
    )
    parser.add_argument(
        "--semantic_dir",
        type=str,
        default=None,
        help="Directory with semantic .ply frames.",
    )
    parser.add_argument(
        "--stable_dir",
        type=str,
        default=None,
        help="Directory with stable semantic .ply frames.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=5.0,
        help="Frames per second for interactive playback or GIF export.",
    )
    parser.add_argument(
        "--max_frames",
        type=int,
        default=0,
        help="Maximum number of frames to animate (0 = all common frames).",
    )
    parser.add_argument(
        "--max_points",
        type=int,
        default=40000,
        help="Maximum number of points per view/frame. Use 0 to disable.",
    )
    parser.add_argument(
        "--coordinate_frame",
        choices=["native", "presentation"],
        default="presentation",
        help=(
            "Coordinate frame used for visualization. 'presentation' maps "
            "real Z to the vertical plot axis and real X to the depth axis."
        ),
    )
    parser.add_argument(
        "--rotate_z_180",
        action="store_true",
        help=(
            "Apply a 180-degree rotation around the real Z axis before the "
            "visualization coordinate-frame remapping."
        ),
    )
    parser.add_argument(
        "--limit_mode",
        choices=["global", "first", "frame"],
        default="global",
        help=(
            "Axis-limit strategy. 'global' keeps limits fixed over the whole "
            "sequence; 'first' uses the first frame; 'frame' recomputes limits."
        ),
    )
    parser.add_argument(
        "--bounds_sample_step",
        type=int,
        default=5,
        help=(
            "Frame stride used to estimate global limits. Lower values are "
            "more accurate but slower. Used only with --limit_mode global."
        ),
    )
    parser.add_argument(
        "--padding_ratio",
        type=float,
        default=0.08,
        help="Relative padding added to fixed axis limits.",
    )
    parser.add_argument(
        "--view_elev",
        type=float,
        default=18.0,
        help="Matplotlib camera elevation angle in degrees.",
    )
    parser.add_argument(
        "--view_azim",
        type=float,
        default=-90.0,
        help="Matplotlib camera azimuth angle in degrees.",
    )
    parser.add_argument(
        "--point_size",
        type=float,
        default=0.8,
        help="Scatter marker size.",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=1.0,
        help="Point opacity in [0, 1].",
    )
    parser.add_argument(
        "--fig_width",
        type=float,
        default=13.5,
        help="Figure width in inches.",
    )
    parser.add_argument(
        "--fig_height",
        type=float,
        default=6.8,
        help="Figure height in inches.",
    )
    parser.add_argument(
        "--hide_axes",
        action="store_true",
        help="Hide ticks, labels, panes and grid for cleaner presentation GIFs.",
    )
    parser.add_argument(
        "--save_gif",
        type=str,
        default=None,
        help="Optional output path to save the animation as a GIF.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=120,
        help="DPI used when saving GIF.",
    )

    return parser.parse_args()


def _collect_views(args: argparse.Namespace) -> List[ViewSpec]:
    """Build active views based on CLI arguments."""
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
            "No input directory provided. Use at least one of --raw_dir, "
            "--colored_dir, --semantic_dir or --stable_dir."
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
    """List frames for each view and determine the common sequence length."""
    frame_paths: Dict[str, List[Path]] = {}
    lengths: List[int] = []

    for label, root in views:
        frames = _list_ply_frames(root)
        frame_paths[label] = frames
        lengths.append(len(frames))
        print(f"[INFO] View '{label}': {len(frames)} frames from {root}")

    num_frames = min(lengths)
    if max_frames > 0:
        num_frames = min(num_frames, max_frames)

    print(f"[INFO] Using {num_frames} frame(s) (min across views).")
    return frame_paths, num_frames


def _load_cloud(path: Path) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Load a PLY point cloud as NumPy arrays: points and optional colors."""
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        return np.empty((0, 3), dtype=np.float64), None

    pts = np.asarray(pcd.points, dtype=np.float64)
    cols = np.asarray(pcd.colors, dtype=np.float64) if pcd.has_colors() else None
    return pts, cols


def _stable_seed(label: str, frame_idx: int) -> int:
    """Return a deterministic seed independent of Python's hash randomization."""
    label_code = sum((i + 1) * ord(ch) for i, ch in enumerate(label))
    return int((frame_idx + 1) * 1_000_003 + label_code) % (2**32 - 1)


def _downsample(
    pts: np.ndarray,
    cols: Optional[np.ndarray],
    max_points: int,
    seed: int,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Randomly subsample points and colors using a deterministic seed."""
    n_pts = pts.shape[0]
    if max_points <= 0 or n_pts <= max_points:
        return pts, cols

    rng = np.random.default_rng(seed)
    idx = rng.choice(n_pts, size=max_points, replace=False)
    pts_ds = pts[idx]
    cols_ds = cols[idx] if cols is not None else None
    return pts_ds, cols_ds


def _compute_layout(n_views: int) -> Tuple[int, int]:
    """Choose a compact subplot layout for up to four synchronized views."""
    if n_views <= 1:
        return 1, 1
    if n_views == 2:
        return 1, 2
    return 2, 2


def _rotate_real_z_180(pts: np.ndarray) -> np.ndarray:
    """Apply Rz(pi): (x, y, z) -> (-x, -y, z) in the real frame."""
    if pts.size == 0:
        return pts

    pts_rot = pts.copy()
    pts_rot[:, 0] *= -1.0
    pts_rot[:, 1] *= -1.0
    return pts_rot


def _transform_points(
    pts: np.ndarray,
    coordinate_frame: str,
    rotate_z_180: bool,
) -> np.ndarray:
    """Transform points for visualization without modifying files on disk."""
    if pts.size == 0:
        return pts

    pts_view = _rotate_real_z_180(pts) if rotate_z_180 else pts

    if coordinate_frame == "native":
        return pts_view

    if coordinate_frame == "presentation":
        out = np.empty_like(pts_view)
        out[:, 0] = pts_view[:, 1]  # screen horizontal axis: real Y
        out[:, 1] = pts_view[:, 0]  # depth axis: real X
        out[:, 2] = pts_view[:, 2]  # vertical axis: real Z
        return out

    raise ValueError(f"Unknown coordinate frame: {coordinate_frame}")


def _limits_from_points(
    pts: np.ndarray,
    padding_ratio: float,
) -> AxisLimits:
    """Compute cubic limits with equal scale in x, y and z."""
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = 0.5 * (mins + maxs)
    span = float(np.max(maxs - mins))
    if span <= 0.0:
        span = 1.0

    half = 0.5 * span * (1.0 + max(0.0, padding_ratio))
    return (
        (float(center[0] - half), float(center[0] + half)),
        (float(center[1] - half), float(center[1] + half)),
        (float(center[2] - half), float(center[2] + half)),
    )


def _merge_limits(points_list: Iterable[np.ndarray], padding_ratio: float) -> AxisLimits:
    """Compute cubic limits from multiple point arrays."""
    valid = [pts for pts in points_list if pts.size > 0]
    if not valid:
        return ((-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0))

    pts_all = np.vstack(valid)
    return _limits_from_points(pts_all, padding_ratio)


def _estimate_fixed_limits(
    frame_paths: Dict[str, List[Path]],
    views: List[ViewSpec],
    num_frames: int,
    args: argparse.Namespace,
) -> Optional[AxisLimits]:
    """Estimate fixed limits according to the selected limit mode."""
    if args.limit_mode == "frame":
        return None

    if args.limit_mode == "first":
        frame_indices = [0]
    else:
        step = max(1, int(args.bounds_sample_step))
        frame_indices = list(range(0, num_frames, step))
        if (num_frames - 1) not in frame_indices:
            frame_indices.append(num_frames - 1)

    print(
        "[INFO] Estimating fixed axis limits using "
        f"{len(frame_indices)} sampled frame(s)."
    )

    sampled_points: List[np.ndarray] = []
    for frame_idx in frame_indices:
        for label, _ in views:
            path = frame_paths[label][frame_idx]
            pts, _ = _load_cloud(path)
            pts = _transform_points(
                pts,
                coordinate_frame=args.coordinate_frame,
                rotate_z_180=args.rotate_z_180,
            )
            sampled_points.append(pts)

    return _merge_limits(sampled_points, float(args.padding_ratio))


def _apply_axis_limits(ax: plt.Axes, limits: AxisLimits) -> None:
    """Apply axis limits and enforce approximately equal 3D aspect ratio."""
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_zlim(*limits[2])
    try:
        ax.set_box_aspect((1.0, 1.0, 1.0))
    except Exception:
        pass


def _clean_axis(ax: plt.Axes, hide_axes: bool) -> None:
    """Apply presentation-oriented axis styling."""
    ax.set_proj_type("ortho")

    if hide_axes:
        ax.set_axis_off()
        return

    ax.grid(True, linestyle="--", alpha=0.25)
    ax.tick_params(axis="both", which="major", labelsize=8, pad=0)
    ax.xaxis.pane.set_alpha(0.0)
    ax.yaxis.pane.set_alpha(0.0)
    ax.zaxis.pane.set_alpha(0.0)


def _set_axis_labels(ax: plt.Axes, coordinate_frame: str, hide_axes: bool) -> None:
    """Set axis labels unless axes are hidden."""
    if hide_axes:
        return

    x_label, y_label, z_label = _AXIS_LABELS[coordinate_frame]
    ax.set_xlabel(x_label, labelpad=3)
    ax.set_ylabel(y_label, labelpad=3)
    ax.set_zlabel(z_label, labelpad=3)


def main() -> None:
    args = _parse_args()
    views = _collect_views(args)

    frame_paths, num_frames = _build_frame_index(views, args.max_frames)
    if num_frames <= 0:
        raise SystemExit("No frames available for animation.")

    n_views = len(views)
    rows, cols = _compute_layout(n_views)

    fig = plt.figure(figsize=(args.fig_width, args.fig_height))
    axes: List[plt.Axes] = [
        fig.add_subplot(rows, cols, idx + 1, projection="3d")
        for idx in range(n_views)
    ]

    fixed_limits = _estimate_fixed_limits(frame_paths, views, num_frames, args)

    def _update(frame_idx: int) -> None:
        fig.suptitle(
            f"Point-cloud sequence | frame {frame_idx + 1:03d}/{num_frames:03d}",
            fontsize=15,
            y=0.98,
        )

        frame_points_for_limits: List[np.ndarray] = []
        frame_payloads: List[Tuple[plt.Axes, str, Path, np.ndarray, Optional[np.ndarray]]] = []

        for ax, (label, _) in zip(axes, views):
            path = frame_paths[label][frame_idx]
            pts, cols = _load_cloud(path)
            pts, cols = _downsample(
                pts,
                cols,
                max_points=int(args.max_points),
                seed=_stable_seed(label, frame_idx),
            )
            pts = _transform_points(
                pts,
                coordinate_frame=args.coordinate_frame,
                rotate_z_180=args.rotate_z_180,
            )

            frame_payloads.append((ax, label, path, pts, cols))
            frame_points_for_limits.append(pts)

        current_limits = fixed_limits
        if current_limits is None:
            current_limits = _merge_limits(
                frame_points_for_limits,
                padding_ratio=float(args.padding_ratio),
            )

        for ax, label, path, pts, cols in frame_payloads:
            ax.cla()
            _clean_axis(ax, hide_axes=bool(args.hide_axes))

            if pts.size == 0:
                ax.set_title(f"{_VIEW_LABELS.get(label, label)} | empty", fontsize=11)
                _apply_axis_limits(ax, current_limits)
                ax.view_init(elev=float(args.view_elev), azim=float(args.view_azim))
                continue

            scatter_kwargs = {
                "s": float(args.point_size),
                "marker": ".",
                "linewidths": 0,
                "alpha": float(args.alpha),
                "depthshade": False,
            }
            if cols is not None:
                ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=cols, **scatter_kwargs)
            else:
                ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c="0.25", **scatter_kwargs)

            ax.set_title(
                f"{_VIEW_LABELS.get(label, label)} | {path.stem}",
                fontsize=11,
                pad=2,
            )
            _set_axis_labels(ax, args.coordinate_frame, bool(args.hide_axes))
            _apply_axis_limits(ax, current_limits)
            ax.view_init(elev=float(args.view_elev), azim=float(args.view_azim))

    interval_ms = 1000.0 / max(float(args.fps), 1e-3)
    anim = FuncAnimation(
        fig,
        _update,
        frames=num_frames,
        interval=interval_ms,
        repeat=True,
    )

    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.95), pad=0.4, w_pad=0.4, h_pad=0.4)

    if args.save_gif is not None:
        out_path = Path(args.save_gif).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        print(f"[INFO] Saving GIF to: {out_path}")
        writer = PillowWriter(fps=max(1, int(round(float(args.fps)))))
        anim.save(str(out_path), writer=writer, dpi=int(args.dpi))
        print(f"[INFO] GIF saved to: {out_path}")
    else:
        print("[INFO] Starting interactive visualization window...")
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.", file=sys.stderr)
