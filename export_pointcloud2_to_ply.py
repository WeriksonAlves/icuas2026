#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Export ROS 2 PointCloud2 frames to PLY files using Open3D.

Designed for ZED2i point clouds (fields: x, y, z, rgb), but handles clouds
without rgb as well.

Typical usage:
  1) ros2 bag play <bag.mcap> --loop
  2) python export_pointcloud2_to_ply.py --topic <topic> --out_dir <dir>

Notes:
- Uses SensorDataQoS (best effort) to match common depth/point cloud publishers.
- 'rgb' is expected as packed float32 (common ROS convention).
"""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


@dataclass(frozen=True)
class ExportConfig:
    topic: str
    out_dir: Path
    max_frames: int
    every_k: int
    voxel: float
    remove_nans: bool


def _make_sensor_qos() -> QoSProfile:
    """Create a QoS profile suitable for sensor streams (best effort)."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
    )


def _field_dtype(field: PointField) -> np.dtype:
    """
    Map ROS PointField datatype to NumPy dtype.

    We implement the datatypes most commonly found in point clouds.
    """
    mapping = {
        PointField.INT8: np.int8,
        PointField.UINT8: np.uint8,
        PointField.INT16: np.int16,
        PointField.UINT16: np.uint16,
        PointField.INT32: np.int32,
        PointField.UINT32: np.uint32,
        PointField.FLOAT32: np.float32,
        PointField.FLOAT64: np.float64,
    }
    if field.datatype not in mapping:
        raise ValueError(f"Unsupported PointField datatype: {field.datatype}")
    return np.dtype(mapping[field.datatype])


def _build_struct_dtype(msg: PointCloud2) -> np.dtype:
    """Build a structured NumPy dtype from PointCloud2 fields."""
    names = []
    formats = []
    offsets = []

    for f in msg.fields:
        names.append(f.name)
        formats.append(_field_dtype(f))
        offsets.append(f.offset)

    return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})


def _extract_xyz_rgb(
    msg: PointCloud2,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    Convert PointCloud2 binary blob to xyz (+ optional rgb) arrays.

    Returns:
      xyz: (N, 3) float32
      rgb: (N, 3) uint8 in [0..255] or None
    """
    cloud_dtype = _build_struct_dtype(msg)
    raw = np.frombuffer(msg.data, dtype=cloud_dtype)

    if "x" not in raw.dtype.names or "y" not in raw.dtype.names or "z" not in raw.dtype.names:
        raise ValueError("PointCloud2 must contain x, y, z fields.")

    xyz = np.vstack((raw["x"], raw["y"], raw["z"])).T.astype(np.float32, copy=False)

    rgb = None
    if "rgb" in raw.dtype.names:
        # ROS convention: packed RGB stored in a float32 field.
        rgb_f = raw["rgb"].astype(np.float32, copy=False)
        rgb_u32 = rgb_f.view(np.uint32)
        r = (rgb_u32 >> 16) & 0xFF
        g = (rgb_u32 >> 8) & 0xFF
        b = rgb_u32 & 0xFF
        rgb = np.vstack((r, g, b)).T.astype(np.uint8, copy=False)

    return xyz, rgb


def _to_open3d_cloud(
    xyz: np.ndarray,
    rgb: Optional[np.ndarray],
    remove_nans: bool,
    voxel: float,
) -> o3d.geometry.PointCloud:
    """Build an Open3D point cloud with optional filtering and voxel downsampling."""
    if remove_nans:
        finite_mask = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite_mask]
        if rgb is not None:
            rgb = rgb[finite_mask]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64, copy=False))

    if rgb is not None:
        # Open3D expects colors as float in [0, 1]
        colors = (rgb.astype(np.float32) / 255.0).astype(np.float64)
        pcd.colors = o3d.utility.Vector3dVector(colors)

    if voxel > 0.0:
        pcd = pcd.voxel_down_sample(voxel_size=float(voxel))

    return pcd


class PointCloudExporter(Node):
    """ROS2 node that subscribes to PointCloud2 and exports frames to PLY."""

    def __init__(self, cfg: ExportConfig) -> None:
        super().__init__("pointcloud2_to_ply_exporter")
        self._cfg = cfg
        self._msg_count = 0
        self._saved = 0

        cfg.out_dir.mkdir(parents=True, exist_ok=True)

        self._sub = self.create_subscription(
            PointCloud2,
            cfg.topic,
            self._callback,
            _make_sensor_qos(),
        )

        self.get_logger().info(f"Subscribed to: {cfg.topic}")
        self.get_logger().info(f"Output dir: {str(cfg.out_dir)}")
        self.get_logger().info(
            f"Settings: max_frames={cfg.max_frames}, every_k={cfg.every_k}, "
            f"voxel={cfg.voxel:.3f}, remove_nans={cfg.remove_nans}"
        )

    def _callback(self, msg: PointCloud2) -> None:
        self._msg_count += 1
        if self._msg_count % self._cfg.every_k != 0:
            return
        if self._saved >= self._cfg.max_frames:
            return

        try:
            xyz, rgb = _extract_xyz_rgb(msg)
            pcd = _to_open3d_cloud(
                xyz=xyz,
                rgb=rgb,
                remove_nans=self._cfg.remove_nans,
                voxel=self._cfg.voxel,
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to parse cloud: {exc}")
            return

        out_path = self._cfg.out_dir / f"frame_{self._saved:05d}.ply"
        ok = o3d.io.write_point_cloud(str(out_path), pcd, write_ascii=False, compressed=False)
        if ok:
            self.get_logger().info(
                f"Saved {out_path.name}: points={len(pcd.points)} "
                f"(msg #{self._msg_count}, stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d})"
            )
            self._saved += 1

        if self._saved >= self._cfg.max_frames:
            self.get_logger().info("Reached max_frames. Shutting down...")
            rclpy.shutdown()


def _parse_args() -> ExportConfig:
    parser = argparse.ArgumentParser(description="Export PointCloud2 frames to PLY.")
    parser.add_argument("--topic", required=True, help="PointCloud2 topic name.")
    parser.add_argument("--out_dir", required=True, help="Output directory for .ply files.")
    parser.add_argument("--max_frames", type=int, default=50, help="Number of frames to save.")
    parser.add_argument(
        "--every_k",
        type=int,
        default=10,
        help="Save one frame every K received messages.",
    )
    parser.add_argument(
        "--voxel",
        type=float,
        default=0.02,
        help="Voxel size (meters). Use 0 to disable.",
    )
    parser.add_argument(
        "--keep_nans",
        action="store_true",
        help="Do not remove NaN/Inf points (not recommended).",
    )

    args = parser.parse_args()

    out_dir = Path(os.path.expanduser(args.out_dir)).resolve()
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
    rclpy.init()
    node = PointCloudExporter(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
