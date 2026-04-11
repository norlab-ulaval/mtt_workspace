#!/usr/bin/env python3
"""Publish a constant TwistStamped command for quick live-robot checks."""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class ConstantSpeedPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mtt_live_robot_constant_speed")
        self.args = args
        self.publisher = self.create_publisher(TwistStamped, args.topic, 10)
        self.estop_publisher = None
        if args.release_estop:
            self.estop_publisher = self.create_publisher(Bool, args.estop_topic, 10)
        self.start_time = time.monotonic()
        self.timer = self.create_timer(1.0 / args.rate, self.publish_command)

    def publish_command(self) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.args.frame_id
        message.twist.linear.x = self.args.linear
        message.twist.angular.z = self.args.angular
        self.publisher.publish(message)

        if self.estop_publisher is not None:
            estop = Bool()
            estop.data = False
            self.estop_publisher.publish(estop)

        if self.args.duration > 0.0 and (time.monotonic() - self.start_time) >= self.args.duration:
            self.publish_stop()
            raise KeyboardInterrupt

    def publish_stop(self) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.args.frame_id
        for _ in range(3):
            self.publisher.publish(message)
            if self.estop_publisher is not None:
                estop = Bool()
                estop.data = False
                self.estop_publisher.publish(estop)
            time.sleep(0.05)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/controller/cmd_vel")
    parser.add_argument("--linear", type=float, default=0.2)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds. 0 keeps publishing until interrupted.")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--release-estop", action="store_true")
    parser.add_argument("--estop-topic", default="teleop_estop")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ConstantSpeedPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
