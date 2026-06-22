#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64, String


class ControlSafetyAudit(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("control_safety_audit")
        self.args = args
        self.started = time.monotonic()
        self.last_joy_time = 0.0
        self.joy_axis = 0.0
        self.deadman_button = False
        self.deadman_topic = False
        self.raw_steer = 0.0
        self.filtered_steer = 0.0
        self.final_steer = 0.0
        self.servo_steer = 0.0
        self.steering_source = "unknown"
        self.errors: set[str] = set()

        self.create_subscription(Joy, "/joy", self.on_joy, 20)
        self.create_subscription(
            TwistStamped, "/cmd_vel/manual_raw",
            lambda msg: setattr(self, "raw_steer", msg.twist.angular.z), 20)
        self.create_subscription(
            TwistStamped, "/cmd_vel/manual",
            lambda msg: setattr(self, "filtered_steer", msg.twist.angular.z), 20)
        self.create_subscription(
            TwistStamped, "/cmd_vel",
            lambda msg: setattr(self, "final_steer", msg.twist.angular.z), 20)
        self.create_subscription(
            Bool, "/mtt_control/teleop_deadman",
            lambda msg: setattr(self, "deadman_topic", msg.data), 20)
        self.create_subscription(
            Float64, "/articulation_servo/steer_cmd",
            lambda msg: setattr(self, "servo_steer", msg.data), 20)
        self.create_subscription(
            String, "/mtt/steering_source",
            lambda msg: setattr(self, "steering_source", msg.data), 20)
        self.create_timer(0.02, self.check_safety)

    def on_joy(self, msg: Joy) -> None:
        self.last_joy_time = time.monotonic()
        self.joy_axis = msg.axes[self.args.angular_axis] if len(msg.axes) > self.args.angular_axis else 0.0
        self.deadman_button = (
            bool(msg.buttons[self.args.deadman_button])
            if len(msg.buttons) > self.args.deadman_button else False
        )

    def check_safety(self) -> None:
        now = time.monotonic()
        joy_age = math.inf if self.last_joy_time == 0.0 else now - self.last_joy_time
        epsilon = self.args.command_epsilon

        if joy_age > self.args.joy_timeout and self.deadman_topic:
            self.errors.add(
                f"deadman remains active with stale joystick ({joy_age:.3f}s)")

        if not self.deadman_topic and abs(self.raw_steer) > epsilon:
            self.errors.add(
                f"manual_raw steer is non-zero without deadman ({self.raw_steer:.4f})")

        if not self.deadman_topic and abs(self.filtered_steer) > epsilon:
            self.errors.add(
                f"manual steer is non-zero without deadman ({self.filtered_steer:.4f})")

        if (
            joy_age <= self.args.joy_timeout
            and self.deadman_button
            and abs(self.joy_axis) <= self.args.axis_deadband
            and abs(self.raw_steer) > epsilon
        ):
            self.errors.add(
                f"manual_raw steer persists with centered axis ({self.raw_steer:.4f})")

    def publisher_report(self) -> list[str]:
        expected = {
            "/joy": 1,
            "/cmd_vel/manual_raw": 1,
            "/cmd_vel/manual": 1,
            "/cmd_vel": 1,
            "/mtt_control/teleop_deadman": 1,
        }
        lines = []
        for topic, expected_count in expected.items():
            infos = self.get_publishers_info_by_topic(topic)
            nodes = sorted(f"{info.node_namespace}/{info.node_name}".replace("//", "/") for info in infos)
            lines.append(f"{topic}: publishers={len(infos)} nodes={nodes}")
            if len(infos) > expected_count:
                self.errors.add(
                    f"{topic} has {len(infos)} publishers; expected at most {expected_count}")
        return lines


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Passively audit joystick, steering commands, deadman, and publisher conflicts.")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--joy-timeout", type=float, default=0.25)
    parser.add_argument("--axis-deadband", type=float, default=0.05)
    parser.add_argument("--command-epsilon", type=float, default=1e-3)
    parser.add_argument("--angular-axis", type=int, default=3)
    parser.add_argument("--deadman-button", type=int, default=5)
    args = parser.parse_args()

    rclpy.init()
    node = ControlSafetyAudit(args)
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        print("Publisher audit:")
        for line in node.publisher_report():
            print(f"  {line}")
        print(
            "Last state: "
            f"axis={node.joy_axis:.4f} deadman={node.deadman_topic} "
            f"raw={node.raw_steer:.4f} filtered={node.filtered_steer:.4f} "
            f"final={node.final_steer:.4f} servo={node.servo_steer:.4f} "
            f"source={node.steering_source}")

        if node.errors:
            print("FAILED:")
            for error in sorted(node.errors):
                print(f"  - {error}")
            return 1

        print("PASS: no steering safety violation detected.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
