#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudVoxelDebug(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_voxel_debug")

        self.input_topic = self.declare_parameter("input_topic", "/hesai_lidar/points").value
        self.output_topic = self.declare_parameter("output_topic", "/debug/hesai_points_voxel").value
        self.voxel_size = float(self.declare_parameter("voxel_size", 0.30).value)
        self.max_rate_hz = float(self.declare_parameter("max_rate_hz", 2.0).value)
        self.max_points = int(self.declare_parameter("max_points", 25000).value)
        self.x_min = float(self.declare_parameter("x_min", -2.0).value)
        self.x_max = float(self.declare_parameter("x_max", 12.0).value)
        self.y_abs = float(self.declare_parameter("y_abs", 6.0).value)
        self.z_min = float(self.declare_parameter("z_min", -1.0).value)
        self.z_max = float(self.declare_parameter("z_max", 3.0).value)

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, sensor_qos)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self._on_cloud, sensor_qos)
        self.last_publish_wall = 0.0

        self.get_logger().info(
            f"voxel debug cloud: {self.input_topic} -> {self.output_topic} "
            f"voxel={self.voxel_size:.2f}m max_rate={self.max_rate_hz:.1f}Hz "
            f"roi=[{self.x_min:.1f},{self.x_max:.1f}]x+/-{self.y_abs:.1f} "
            f"z=[{self.z_min:.1f},{self.z_max:.1f}]"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        min_period = 1.0 / self.max_rate_hz if self.max_rate_hz > 0.0 else 0.0
        if now - self.last_publish_wall < min_period:
            return
        self.last_publish_wall = now

        xyz = self._extract_xyz(msg)
        if xyz is None or xyz.size == 0:
            return

        mask = (
            np.isfinite(xyz).all(axis=1)
            & (xyz[:, 0] >= self.x_min)
            & (xyz[:, 0] <= self.x_max)
            & (np.abs(xyz[:, 1]) <= self.y_abs)
            & (xyz[:, 2] >= self.z_min)
            & (xyz[:, 2] <= self.z_max)
        )
        xyz = xyz[mask]
        if xyz.size == 0:
            return

        xyz = self._voxel_downsample(xyz, self.voxel_size)
        if xyz.shape[0] > self.max_points:
            step = int(math.ceil(xyz.shape[0] / self.max_points))
            xyz = xyz[::step]

        out = point_cloud2.create_cloud_xyz32(msg.header, xyz.astype(np.float32, copy=False).tolist())
        self.pub.publish(out)

    def _extract_xyz(self, msg: PointCloud2) -> Optional[np.ndarray]:
        try:
            points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=False)
            return np.asarray(points, dtype=np.float32).reshape(-1, 3)
        except Exception:
            try:
                points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
                return np.asarray(list(points), dtype=np.float32).reshape(-1, 3)
            except Exception as exc:
                self.get_logger().warning(f"failed to extract xyz from {self.input_topic}: {exc}")
                return None

    @staticmethod
    def _voxel_downsample(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
        if voxel_size <= 0.0 or xyz.shape[0] <= 1:
            return xyz
        keys = np.floor(xyz / voxel_size).astype(np.int64)
        _, unique_idx = np.unique(keys, axis=0, return_index=True)
        unique_idx.sort()
        return xyz[unique_idx]


def main() -> None:
    rclpy.init()
    node = PointCloudVoxelDebug()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
