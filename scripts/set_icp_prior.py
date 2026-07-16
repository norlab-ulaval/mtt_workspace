#!/usr/bin/env python3
"""Publish a manual pose prior to /mapping/pose_in to help ICP relocalize.

Use this when the mapper just loaded a saved map (localization-only mode) and
ICP hasn't converged yet.  The mapper immediately uses the prior as its new
pose estimate; subsequent LiDAR scans should converge within 2-3 seconds.

Usage (inside the robot Docker or with ros2 sourced):
  ./scripts/set_icp_prior.py <x> <y> <yaw_deg> [topic]

  x, y     — position in the MAP frame (metres)
  yaw_deg  — heading in the MAP frame (degrees, 0 = +X axis)
  topic    — optional, default /mapping/pose_in

Examples:
  ./scripts/set_icp_prior.py 0 0 0               # map origin, facing +X
  ./scripts/set_icp_prior.py 12.5 -3.2 90        # (12.5, -3.2) facing +Y
  ./scripts/set_icp_prior.py 5.0 2.0 180         # facing -X

Where to get x, y, yaw:
  In Foxglove Studio → 3D panel → hover over the loaded map to read coordinates,
  or check /mtt_route/start_pose to see where the teach route started.

Foxglove alternative (no script needed):
  1. In Foxglove → "Publish" panel
  2. Topic: /mapping/pose_in
  3. Schema: geometry_msgs/PoseWithCovarianceStamped
  4. Fill x, y, qz = sin(yaw/2), qw = cos(yaw/2), frame_id = "map"
  5. Click Publish

After publishing, wait 2-3 seconds and verify /mtt_route/status_text shows
"not on route: nearest=X.XX m" with X.XX decreasing as ICP converges.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def main() -> None:
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw_deg = float(sys.argv[3])
    topic = sys.argv[4] if len(sys.argv) > 4 else "/mapping/pose_in"
    yaw_rad = math.radians(yaw_deg)

    rclpy.init()
    node = Node("icp_prior_publisher")
    pub = node.create_publisher(PoseWithCovarianceStamped, topic, 1)

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = math.sin(yaw_rad * 0.5)
    msg.pose.pose.orientation.w = math.cos(yaw_rad * 0.5)

    # Large diagonal covariance = "rough estimate, please refine with ICP"
    cov = [0.0] * 36
    cov[0] = 1.00   # x  (1 m²  ≈ ±1 m)
    cov[7] = 1.00   # y
    cov[14] = 0.10  # z
    cov[21] = 0.10  # roll
    cov[28] = 0.10  # pitch
    cov[35] = 0.50  # yaw  (0.5 rad²  ≈ ±40°)
    msg.pose.covariance = cov

    # Publish several times — some latched subscribers need a short delay.
    for _ in range(5):
        pub.publish(msg)
        time.sleep(0.05)

    node.get_logger().info(
        f"Pose prior sent → {topic}: x={x:.3f} y={y:.3f} yaw={yaw_deg:.1f}°"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
