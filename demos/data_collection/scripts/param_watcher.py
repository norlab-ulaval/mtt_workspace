#!/usr/bin/env python3
"""param_watcher.py — Mid-bag parameter change tracker.

Subscribes to /parameter_events (rcl_interfaces/msg/ParameterEvent) and
republishes control-critical parameter changes as JSON events on
/session/events (std_msgs/msg/String, reliable + transient_local depth 10).

Purpose: give offline pipelines an exact timestamped record of every
speed / acceleration limit change that occurred during an MCAP session.
The /session/events topic is already declared in qos_override.yaml and
recorded by ros2 bag record.

Usage (compose.yaml param_watcher service):
    python3 param_watcher.py
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterEvent
from std_msgs.msg import String

# ── Which nodes / parameters to watch ──────────────────────────────────────
# Each entry: (ros_node_full_name, parameter_name)
WATCHED_PARAMS: set = {
    # Joystick speed ceiling (scales raw axis → m/s command)
    ("/mtt_operator_input_node", "max_linear_speed"),
    ("/mtt_operator_input_node", "max_angular_command"),
    # Acceleration ramp rates (m/s² applied in the slew-rate limiter)
    ("/mtt_manual_cmd_filter_node", "linear_rise_rate"),
    ("/mtt_manual_cmd_filter_node", "linear_fall_rate"),
    ("/mtt_manual_cmd_filter_node", "angular_rise_rate"),
    ("/mtt_manual_cmd_filter_node", "angular_fall_rate"),
    # Autonomous speed cap (applied in cmd_arbiter on the AUTO branch)
    ("/mtt_cmd_arbiter_node", "max_auto_speed_ms"),
    # CAN-level speed ceiling (throttle normalization denominator)
    ("/mtt_can_node", "max_linear_speed_ms"),
    # COM motor (CL86EC EtherCAT stepper) — com_position_node
    # These are already live-tunable via add_on_set_parameters_callback.
    # Silently ignored when /com_position_node is not running (e.g. data_collection).
    ("/com_position_node", "run_slew"),          # RUN mode speed (counts/s)
    ("/com_position_node", "amplitude_counts"),  # Spring half-range (counts)
    ("/com_position_node", "setup_slew"),        # SETUP mode speed (counts/s)
    ("/com_position_node", "rearm_slew"),        # Park / slide-in speed (counts/s)
    ("/com_position_node", "direction_sign"),    # +1 / -1 direction convention
    ("/com_position_node", "max_linear_speed"),  # Speed threshold for COM flick
}

# ── QoS for /session/events — must match qos_override.yaml ─────────────────
SESSION_EVENTS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── QoS for /parameter_events — publisher uses BEST_EFFORT ──────────────────
PARAM_EVENTS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)


def _param_value_to_python(param):
    """Convert a rcl_interfaces/msg/ParameterValue to a Python scalar."""
    # ParameterType enum values
    PARAM_BOOL   = 1
    PARAM_INT    = 2
    PARAM_DOUBLE = 3
    PARAM_STRING = 4
    t = param.type
    if t == PARAM_BOOL:
        return param.bool_value
    if t == PARAM_INT:
        return param.integer_value
    if t == PARAM_DOUBLE:
        return float(param.double_value)
    if t == PARAM_STRING:
        return param.string_value
    return None  # array types — ignore for control params


class ParamWatcherNode(Node):
    def __init__(self):
        super().__init__("mtt_param_watcher")

        self._events_pub = self.create_publisher(
            String, "/session/events", SESSION_EVENTS_QOS
        )

        self._param_sub = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self._on_parameter_event,
            PARAM_EVENTS_QOS,
        )

        self.get_logger().info(
            "param_watcher ready — watching %d control parameters, "
            "publishing to /session/events" % len(WATCHED_PARAMS)
        )

        # Publish a startup sentinel so the bag always has at least one event
        self._publish_event(
            node="/mtt_param_watcher",
            param="startup",
            value="param_watcher_ready",
            event_type="info",
        )

    def _on_parameter_event(self, msg):
        node = msg.node  # fully-qualified node name, e.g. "/mtt_can_node"

        for p in list(msg.changed_parameters) + list(msg.new_parameters):
            if (node, p.name) not in WATCHED_PARAMS:
                continue
            value = _param_value_to_python(p.value)
            if value is None:
                continue
            self.get_logger().warn(
                "[PARAM CHANGE] %s.%s = %s" % (node, p.name, value)
            )
            self._publish_event(
                node=node,
                param=p.name,
                value=value,
                event_type="param_changed",
            )

    def _publish_event(self, *, node, param, value, event_type):
        """Publish a JSON event to /session/events."""
        stamp = self.get_clock().now().to_msg()
        payload = {
            "event_type": event_type,
            "stamp_sec": stamp.sec,
            "stamp_nanosec": stamp.nanosec,
            "node": node,
            "param": param,
            "value": value,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._events_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParamWatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
