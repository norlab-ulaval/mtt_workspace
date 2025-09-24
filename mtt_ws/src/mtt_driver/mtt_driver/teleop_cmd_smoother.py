#!/usr/bin/env python3
"""
MTT Command Smoother

Purpose:
- Smooth incoming Twist commands (typically from human teleop) before they go into the vehicle control stack.
- Provides simple slew-rate limiting (acceleration limiting) for linear.x and angular.z.
- Designed to be extended later with jerk limiting or PID shaping if needed.

Parameters:
- input_topic (string): input Twist topic to smooth (default: 'cmd_vel/teleop')
- output_topic (string): output Twist topic (default: 'cmd_vel/teleop_smoothed')
- max_accel_linear (double): max change in linear.x per second (default: 1.5 m/s^2)
- max_accel_angular (double): max change in angular.z per second (default: 1.5 rad/s^2)
- rate_hz (double): smoothing update rate (default: 50.0 Hz)
"""

from typing import Optional
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MttTeleopCmdSmoother(Node):
    def __init__(self) -> None:
        super().__init__('teleop_cmd_smoother')

        # Declare parameters with defaults
        self.declare_parameter('input_topic', 'cmd_vel/teleop')
        self.declare_parameter('output_topic', 'cmd_vel/teleop_smoothed')
        self.declare_parameter('max_accel_linear', 1.5)
        self.declare_parameter('max_accel_angular', 1.5)
        self.declare_parameter('rate_hz', 50.0)
        # If no new input for this many seconds, decay target to zero
        self.declare_parameter('input_timeout', 0.3)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.max_accel_linear = self.get_parameter('max_accel_linear').get_parameter_value().double_value
        self.max_accel_angular = self.get_parameter('max_accel_angular').get_parameter_value().double_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.input_timeout = self.get_parameter('input_timeout').get_parameter_value().double_value

        # I/O
        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.sub = self.create_subscription(Twist, self.input_topic, self._on_cmd, 10)

        # State
        self._target: Twist = Twist()
        self._current: Twist = Twist()
        self._last_time: Optional[float] = None
        self._last_input_time: Optional[float] = None

        # Timer for smoothing loop
        period = 1.0 / max(self.rate_hz, 1.0)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Cmd smoother started. in='{self.input_topic}' out='{self.output_topic}' "
            f"max_accel_linear={self.max_accel_linear} max_accel_angular={self.max_accel_angular}"
        )

    def _on_cmd(self, msg: Twist) -> None:
        # Update target command when a new message arrives
        self._target = msg
        self._last_input_time = time.time()

    def _tick(self) -> None:
        now = time.time()
        if self._last_time is None:
            self._last_time = now
            # Publish zeros initially
            self.pub.publish(self._current)
            return

        dt = max(min(now - self._last_time, 0.2), 0.0)  # clamp dt to avoid spikes
        self._last_time = now

        # If inputs are stale, decay target to zero
        if self._last_input_time is None or (now - self._last_input_time) > max(0.0, self.input_timeout):
            zero = Twist()
            self._target = zero

        # Slew-rate limit linear.x and angular.z towards target
        self._current.linear.x = self._slew(
            self._current.linear.x, self._target.linear.x, self.max_accel_linear, dt
        )
        self._current.angular.z = self._slew(
            self._current.angular.z, self._target.angular.z, self.max_accel_angular, dt
        )

        # Directly pass through other fields (commonly unused)
        self._current.linear.y = self._target.linear.y
        self._current.linear.z = self._target.linear.z
        self._current.angular.x = self._target.angular.x
        self._current.angular.y = self._target.angular.y

        self.pub.publish(self._current)

    @staticmethod
    def _slew(current: float, target: float, max_accel: float, dt: float) -> float:
        # Clamp step by max_accel * dt
        max_step = max(0.0, max_accel) * dt
        delta = target - current
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        return current + delta


def main(args=None):
    rclpy.init(args=args)
    node = MttTeleopCmdSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Process terminated externally (e.g., SIGTERM); ignore
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Context may already be shut down
            pass


if __name__ == '__main__':
    main()
