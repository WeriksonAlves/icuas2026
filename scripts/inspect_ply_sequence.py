#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive viewer for a sequence of PLY point clouds.

Usage:
  python scripts/inspect_ply_sequence.py --ply_dir data/ply/outdoor_02_flight_5

Controls:
  A  -> previous frame
  D  -> next frame
  Q  -> quit
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Browse a sequence of PLY point clouds interactively."
    )
    parser.add_argument(
        "--ply_dir",
        required=True,
        help="Directory containing frame_XXXXX.ply files.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ply_dir = Path(args.ply_dir).expanduser().resolve()

    files = sorted(ply_dir.glob("frame_*.ply"))
    if not files:
        raise SystemExit(f"No PLY files matching 'frame_*.ply' in {ply_dir}")

    print(f"[INFO] Found {len(files)} frames in {ply_dir}")
    print("[INFO] Controls: A = previous, D = next, Q = quit")

    state = {"idx": 0}

    # Load first frame
    pcd = o3d.io.read_point_cloud(str(files[state["idx"]]))
    if pcd.is_empty():
        print(f"[WARN] Empty point cloud for {files[state['idx']].name}")

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="PLY sequence inspector")
    vis.add_geometry(pcd)

    def print_status():
        fname = files[state["idx"]].name
        pts = np.asarray(pcd.points)
        if pts.size == 0:
            num_pts = 0
            z_min = z_max = float("nan")
        else:
            num_pts = pts.shape[0]
            z_min = float(pts[:, 2].min())
            z_max = float(pts[:, 2].max())
        print(
            f"[FRAME] idx={state['idx']:04d}  file={fname}  "
            f"points={num_pts}  z_range=[{z_min:.2f}, {z_max:.2f}]"
        )

    def load_frame(new_idx: int):
        new_idx = max(0, min(len(files) - 1, new_idx))
        state["idx"] = new_idx
        new_pcd = o3d.io.read_point_cloud(str(files[new_idx]))
        if new_pcd.is_empty():
            print(f"[WARN] Empty point cloud for {files[new_idx].name}")

        pcd.points = new_pcd.points
        if new_pcd.has_colors():
            pcd.colors = new_pcd.colors
        else:
            # optional: reset colors to white
            pcd.colors = o3d.utility.Vector3dVector()

        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        print_status()

    def next_cb(vis_):
        load_frame(state["idx"] + 1)
        return False

    def prev_cb(vis_):
        load_frame(state["idx"] - 1)
        return False

    def quit_cb(vis_):
        vis_.close()
        return False

    vis.register_key_callback(ord("D"), next_cb)
    vis.register_key_callback(ord("A"), prev_cb)
    vis.register_key_callback(ord("Q"), quit_cb)

    print_status()
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()