#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# src/planes/ros_export.py
"""ROS 2 PointCloud2 → PLY exporter based on Open3D.

This module provides a reusable ROS 2 node and helper functions to export
PointCloud2 messages to PLY files. It is designed for ZED2i-style clouds
(x, y, z, rgb) but also supports clouds without RGB or with 'rgba' fields.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ExportConfig:
    """Configuration parameters for PointCloud2 export."""

    topic: str
    out_dir: Path
    max_frames: int
    every_k: int
    voxel: float
    remove_nans: bool


# ---------------------------------------------------------------------------
# QoS helpers
# ---------------------------------------------------------------------------


def make_sensor_qos() -> QoSProfile:
    """Create a QoS profile suitable for sensor streams (best effort)."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
    )


# ---------------------------------------------------------------------------
# PointField → NumPy dtype helpers
# ---------------------------------------------------------------------------


def field_dtype(field: PointField) -> np.dtype:
    """Map ROS PointField datatype to NumPy dtype.

    Only the most common datatypes are implemented; other cases will raise.
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

    np_dtype = mapping.get(field.datatype)
    if np_dtype is None:
        raise ValueError(f"Unsupported PointField datatype: {field.datatype}")
    return np_dtype


def build_struct_dtype(msg: PointCloud2) -> np.dtype:
    """Build a NumPy structured dtype for a PointCloud2 message.

    The dtype respects the fields' offsets and names, enabling
    `np.frombuffer(msg.data, dtype=cloud_dtype)` without copies.
    """
    fields = sorted(msg.fields, key=lambda f: f.offset)
    dtype_fields = []
    offset = 0

    for f in fields:
        np_dtype = field_dtype(f)
        # Align to the field's offset as specified in the message
        if f.offset > offset:
            pad = f.offset - offset
            dtype_fields.append((f"_pad_{offset}", np.uint8, pad))
            offset += pad

        if f.count == 1:
            dtype_fields.append((f.name, np_dtype))
        else:
            dtype_fields.append((f.name, np_dtype, (f.count,)))

        itemsize = np.dtype(np_dtype).itemsize
        offset = f.offset + itemsize * max(f.count, 1)

    return np.dtype(dtype_fields)


# ---------------------------------------------------------------------------
# Extraction: raw structured array → xyz, rgb
# ---------------------------------------------------------------------------


def extract_xyz_rgb_from_raw(
    raw: np.ndarray,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Extract xyz (and optional rgb) from a structured NumPy array.

    Parameters
    ----------
    raw :
        Structured NumPy array obtained from a PointCloud2 buffer:
        `np.frombuffer(msg.data, dtype=build_struct_dtype(msg))`.

    Returns
    -------
    xyz :
        Array of shape (N, 3), dtype float32.
    rgb :
        Array of shape (N, 3), dtype uint8 in [0, 255], or None if no RGB.
    """
    dtype_names = raw.dtype.names or ()
    if not {"x", "y", "z"}.issubset(dtype_names):
        raise ValueError("PointCloud2 must contain x, y, z fields.")

    xyz = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(
        np.float32,
        copy=False,
    )

    # Try to locate an RGB-like field.
    field_name: Optional[str]
    if "rgb" in dtype_names:
        field_name = "rgb"
    elif "rgba" in dtype_names:
        field_name = "rgba"
    else:
        field_name = None

    if field_name is None:
        return xyz, None

    field = raw[field_name]
    rgb: Optional[np.ndarray]

    # Case A: packed float32 (common ROS convention)
    if field.dtype == np.float32:
        packed = field.view(np.uint32)
    # Case B: packed uint32 (already integer)
    elif field.dtype == np.uint32:
        packed = field
    # Case C: 3 or 4 separate uint8 channels (N, 3) or (N, 4)
    elif field.dtype == np.uint8 and field.ndim == 2 and field.shape[1] >= 3:
        rgb = field[:, :3].astype(np.uint8, copy=False)
        return xyz, rgb
    else:
        # Fallback: try viewing as uint32 if layout matches
        try:
            packed = field.view(np.uint32)
        except TypeError as exc:
            raise ValueError(
                "Unsupported RGB field dtype/shape: "
                f"{field.dtype}, {field.shape}"
            ) from exc

    r = (packed >> 16) & 0xFF
    g = (packed >> 8) & 0xFF
    b = packed & 0xFF
    rgb = np.column_stack((r, g, b)).astype(np.uint8, copy=False)

    return xyz, rgb


# ---------------------------------------------------------------------------
# Open3D conversion
# ---------------------------------------------------------------------------


def to_open3d_cloud(
    xyz: np.ndarray,
    rgb: Optional[np.ndarray],
    remove_nans: bool,
    voxel: float,
) -> o3d.geometry.PointCloud:
    """
    Build an Open3D point cloud with optional filtering and voxel downsampling.
    """
    if remove_nans:
        finite_mask = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite_mask]
        if rgb is not None:
            rgb = rgb[finite_mask]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(
        xyz.astype(np.float64, copy=False)
    )

    if rgb is not None:
        # Open3D expects colors as float in [0, 1]
        colors = (rgb.astype(np.float32, copy=False) / 255.0).astype(
            np.float64,
            copy=False,
        )
        pcd.colors = o3d.utility.Vector3dVector(colors)

    if voxel > 0.0:
        pcd = pcd.voxel_down_sample(voxel_size=float(voxel))

    return pcd


# ---------------------------------------------------------------------------
# Message signature and dtype cache
# ---------------------------------------------------------------------------


def cloud_signature(msg: PointCloud2) -> str:
    """Build a signature that identifies the layout of a PointCloud2 message.

    This is used to cache the structured dtype so it is only built once per
    message layout.
    """
    parts = [
        str(msg.is_bigendian),
        str(msg.point_step),
        str(len(msg.fields)),
    ]
    for f in sorted(msg.fields, key=lambda f: f.offset):
        parts.append(f"{f.name}:{f.datatype}:{f.offset}:{f.count}")
    return "|".join(parts)


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------


class PointCloudExporter(Node):
    """ROS 2 node that subscribes to PointCloud2 and exports frames to PLY."""

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
            make_sensor_qos(),
        )

        # Cache of structured dtypes keyed by message signature
        self._dtype_cache: Dict[str, np.dtype] = {}

        self.get_logger().info(
            "PointCloudExporter initialized: "
            f"topic={cfg.topic}, "
            f"out_dir={cfg.out_dir}, "
            f"max_frames={cfg.max_frames}, "
            f"every_k={cfg.every_k}, "
            f"voxel={cfg.voxel:.4f}, "
            f"remove_nans={cfg.remove_nans}"
        )

    @property
    def num_saved(self) -> int:
        """Return the number of frames saved so far."""
        return self._saved

    def _callback(self, msg: PointCloud2) -> None:
        """Subscription callback: convert and save a PLY frame if needed."""
        self._msg_count += 1

        if self._msg_count % self._cfg.every_k != 0:
            return

        if self._saved >= self._cfg.max_frames:
            # We do not destroy the node here; the executor should stop
            # externally based on num_saved, keeping responsibilities separate.
            return

        try:
            sig = cloud_signature(msg)
            if sig not in self._dtype_cache:
                self._dtype_cache[sig] = build_struct_dtype(msg)

            cloud_dtype = self._dtype_cache[sig]
            raw = np.frombuffer(msg.data, dtype=cloud_dtype)

            xyz, rgb = extract_xyz_rgb_from_raw(raw)
            pcd = to_open3d_cloud(
                xyz=xyz,
                rgb=rgb,
                remove_nans=self._cfg.remove_nans,
                voxel=self._cfg.voxel,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to parse cloud: {exc}")
            return

        frame_idx = self._saved
        out_path = self._cfg.out_dir / f"frame_{frame_idx:05d}.ply"

        try:
            o3d.io.write_point_cloud(str(out_path), pcd, write_ascii=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to write PLY: {exc}")
            return

        self._saved += 1
        self.get_logger().info(
            f"Saved frame {frame_idx} to {out_path} "
            f"({self._saved}/{self._cfg.max_frames})"
        )


# ---------------------------------------------------------------------------
# High-level orchestration
# ---------------------------------------------------------------------------


def run_export(cfg: ExportConfig) -> None:
    """Run the PointCloudExporter node until max_frames or Ctrl+C.

    This function is the recommended entry point for CLI scripts and
    notebooks. It encapsulates rclpy initialization and shutdown.
    """
    rclpy.init()
    node = PointCloudExporter(cfg)

    try:
        # Spin until the node has saved all frames or until Ctrl+C.
        while rclpy.ok() and node.num_saved < cfg.max_frames:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user (Ctrl+C).")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
