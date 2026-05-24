#!/usr/bin/env python3
"""Convert ZED detections to MarkerArray for quick Foxglove visualization."""

from __future__ import annotations

import argparse
import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from zed_msgs.msg import ObjectsStamped


BOX_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)


def finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(v) for v in values)


def point_from_xyz(xyz: Iterable[float]) -> Point:
    vals = list(xyz)
    p = Point()
    p.x = float(vals[0])
    p.y = float(vals[1])
    p.z = float(vals[2])
    return p


def color_for_label(label: str) -> tuple[float, float, float, float]:
    label = label.upper()
    if "PERSON" in label or "BODY" in label:
        return 0.1, 1.0, 0.35, 0.9
    if "VEHICLE" in label:
        return 1.0, 0.55, 0.05, 0.9
    if "ANIMAL" in label:
        return 1.0, 0.1, 0.8, 0.9
    return 0.2, 0.7, 1.0, 0.9


class ZedObjectsToMarkers(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("zed_objects_to_markers")
        self.confidence_min = args.confidence_min
        self.only_person = args.only_person
        self.stale_seconds = args.stale_seconds
        self.objects_msg: ObjectsStamped | None = None
        self.objects_time = self.get_clock().now()
        self.bodies_msg: ObjectsStamped | None = None
        self.bodies_time = self.get_clock().now()

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(MarkerArray, args.marker_topic, qos)
        self.create_subscription(ObjectsStamped, args.objects_topic, self.on_objects, qos)
        self.create_subscription(ObjectsStamped, args.bodies_topic, self.on_bodies, qos)
        self.create_timer(0.1, self.publish_markers)
        self.get_logger().info(
            f"Publishing ZED markers on {args.marker_topic} from "
            f"{args.objects_topic} and {args.bodies_topic}"
        )

    def on_objects(self, msg: ObjectsStamped) -> None:
        self.objects_msg = msg
        self.objects_time = self.get_clock().now()

    def on_bodies(self, msg: ObjectsStamped) -> None:
        self.bodies_msg = msg
        self.bodies_time = self.get_clock().now()

    def publish_markers(self) -> None:
        now = self.get_clock().now()
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 1
        if self.objects_msg and (now - self.objects_time).nanoseconds * 1e-9 <= self.stale_seconds:
            marker_id = self.add_object_markers(markers, self.objects_msg, marker_id, "obj")
        if self.bodies_msg and (now - self.bodies_time).nanoseconds * 1e-9 <= self.stale_seconds:
            marker_id = self.add_object_markers(markers, self.bodies_msg, marker_id, "body")

        self.pub.publish(markers)

    def add_object_markers(
        self,
        markers: MarkerArray,
        msg: ObjectsStamped,
        marker_id: int,
        namespace: str,
    ) -> int:
        for obj in msg.objects:
            label = str(obj.label or obj.sublabel or namespace)
            if self.only_person and "PERSON" not in label.upper() and "BODY" not in label.upper():
                continue
            if float(obj.confidence) < self.confidence_min:
                continue

            r, g, b, a = color_for_label(label)
            center = list(obj.position)
            if not finite(center):
                continue

            line = Marker()
            line.header = msg.header
            line.ns = f"zed_{namespace}_boxes"
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.035
            line.color.r = r
            line.color.g = g
            line.color.b = b
            line.color.a = a

            corners = [list(c.kp) for c in obj.bounding_box_3d.corners]
            if len(corners) == 8 and all(finite(c) for c in corners):
                for i, j in BOX_EDGES:
                    line.points.append(point_from_xyz(corners[i]))
                    line.points.append(point_from_xyz(corners[j]))
                markers.markers.append(line)

            text = Marker()
            text.header = msg.header
            text.ns = f"zed_{namespace}_labels"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = point_from_xyz(center)
            text.pose.position.z += max(0.4, float(obj.dimensions_3d[2]) * 0.6)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.28
            text.color.r = r
            text.color.g = g
            text.color.b = b
            text.color.a = 1.0
            text.text = f"{label} {float(obj.confidence):.0f}%"
            markers.markers.append(text)

            if obj.skeleton_available:
                points = Marker()
                points.header = msg.header
                points.ns = f"zed_{namespace}_skeleton_points"
                points.id = marker_id
                marker_id += 1
                points.type = Marker.POINTS
                points.action = Marker.ADD
                points.scale.x = 0.07
                points.scale.y = 0.07
                points.color.r = r
                points.color.g = g
                points.color.b = b
                points.color.a = 1.0
                for kp in obj.skeleton_3d.keypoints:
                    xyz = list(kp.kp)
                    if finite(xyz) and any(abs(v) > 1e-4 for v in xyz):
                        points.points.append(point_from_xyz(xyz))
                if points.points:
                    markers.markers.append(points)
        return marker_id


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--objects-topic", default="/zed/zed_node/obj_det/objects")
    parser.add_argument("--bodies-topic", default="/zed/zed_node/body_trk/skeletons")
    parser.add_argument("--marker-topic", default="/zed/detection_markers")
    parser.add_argument("--confidence-min", type=float, default=0.0)
    parser.add_argument("--stale-seconds", type=float, default=1.0)
    parser.add_argument("--only-person", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = ZedObjectsToMarkers(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
