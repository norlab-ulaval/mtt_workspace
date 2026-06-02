#!/usr/bin/env python3
"""Deterministic ROS bag step replay for offline ICP.

This is intentionally not a general replacement for ``ros2 bag play``.  It is a
controlled feeder for the mapper: publish support topics up to a LiDAR stamp,
publish one mapping cloud, wait until the mapper publishes the matching ICP
odom, then continue.  That removes DDS backlog from offline map rebuilding.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import rosbag2_py
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message


DEFAULT_SUPPORT_TOPICS = {
    "/tf_static",
    "/tf",
    "/robot_description",
    "/joint_states",
    "/runtime_joint_states",
    "/mtt_tachometer",
    "/mtt_articulation_angle",
    "/mtt/articulation_state",
    "/mtt_articulation_state",
    "/mtt_status",
    "/mtt_health",
    "/cmd_vel",
    "/cmd_vel/teleop",
    "/cmd_vel/teleop_raw",
    "/cmd_vel/manual",
    "/cmd_vel/manual_raw",
    "/controller/cmd_vel",
    "/mtt/articulation_cmd",
    "/hardware/articulation_angle",
    "/trailer/articulation_angle",
    "/trailer/angle",
    "/mti100/data",
    "/mti100/data_raw",
    "/mti10/data",
    "/mti10/data_raw",
}


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def sec_to_time_msg(value: float) -> TimeMsg:
    sec = math.floor(value)
    out = TimeMsg()
    out.sec = int(sec)
    out.nanosec = int(round((value - sec) * 1e9))
    if out.nanosec >= 1_000_000_000:
        out.sec += 1
        out.nanosec -= 1_000_000_000
    return out


def msg_stamp_or_bag_time(msg: Any, bag_time_s: float) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and (int(stamp.sec) or int(stamp.nanosec)):
        return stamp_to_sec(stamp)
    return bag_time_s


def qos_for_topic(topic: str) -> QoSProfile:
    if topic == "/tf_static":
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
    if topic.endswith("/points") or "cloud" in topic:
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


@dataclass
class TopicState:
    topic_type: str
    msg_type: Any
    publisher: Any


class MapperLogMonitor:
    def __init__(self, path: Path | None) -> None:
        self.path = path
        self.offset = 0
        self.callbacks = 0
        self.accepted = 0
        self.rejected = 0
        self._callback_done_re = re.compile(r"\[POINTS_IN\] callback done #(?P<count>\d+)")
        self._heartbeat_re = re.compile(
            r"\[MAPPER_HEARTBEAT\] callbacks=(?P<callbacks>\d+)/\d+.*"
            r"accepted=(?P<accepted>\d+) rejected=(?P<rejected>\d+)"
        )

    def poll(self) -> int:
        if self.path is None or not self.path.exists():
            return self.callbacks
        try:
            with self.path.open("r", encoding="utf-8", errors="ignore") as f:
                f.seek(self.offset)
                chunk = f.read()
                self.offset = f.tell()
        except OSError:
            return self.callbacks
        for line in chunk.splitlines():
            done = self._callback_done_re.search(line)
            if done:
                self.callbacks = max(self.callbacks, int(done.group("count")))
                continue
            heartbeat = self._heartbeat_re.search(line)
            if heartbeat:
                self.callbacks = max(self.callbacks, int(heartbeat.group("callbacks")))
                self.accepted = max(self.accepted, int(heartbeat.group("accepted")))
                self.rejected = max(self.rejected, int(heartbeat.group("rejected")))
        return self.callbacks


class StepReplayNode(Node):
    def __init__(self, args: argparse.Namespace, topic_types: dict[str, str]) -> None:
        super().__init__("offline_icp_step_replay")
        self.args = args
        self.topic_states: dict[str, TopicState] = {}
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.last_clock_s = 0.0
        self.last_icp_stamp = 0.0
        self.icp_count = 0
        self.create_subscription(
            Odometry,
            args.icp_topic,
            self._on_icp,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=200,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        for topic, topic_type in topic_types.items():
            msg_type = get_message(topic_type)
            publisher = self.create_publisher(msg_type, topic, qos_for_topic(topic))
            self.topic_states[topic] = TopicState(topic_type, msg_type, publisher)

    def _on_icp(self, msg: Odometry) -> None:
        self.last_icp_stamp = msg_stamp_or_bag_time(msg, 0.0)
        self.icp_count += 1

    def publish_clock(self, stamp_s: float) -> None:
        # ROS time must never go backwards. Some recorded messages have header
        # stamps slightly older than bag order, and /tf2 clears its buffer when
        # /clock jumps back.
        if stamp_s < self.last_clock_s:
            stamp_s = self.last_clock_s
        else:
            self.last_clock_s = stamp_s
        msg = Clock()
        msg.clock = sec_to_time_msg(stamp_s)
        self.clock_pub.publish(msg)

    def publish_raw(self, topic: str, raw: bytes, bag_time_s: float) -> float:
        state = self.topic_states[topic]
        msg = deserialize_message(raw, state.msg_type)
        stamp_s = msg_stamp_or_bag_time(msg, bag_time_s)
        self.publish_clock(stamp_s)
        state.publisher.publish(msg)
        return stamp_s

    def try_publish_raw(self, topic: str, raw: bytes, bag_time_s: float) -> tuple[float | None, str | None]:
        try:
            return self.publish_raw(topic, raw, bag_time_s), None
        except Exception as exc:  # noqa: BLE001 - corrupt bag samples must not kill long offline batches.
            topic_type = self.topic_states[topic].topic_type
            return None, f"{type(exc).__name__}: topic={topic} type={topic_type} error={exc}"

    def try_deserialize(self, topic: str, raw: bytes) -> tuple[Any | None, str | None]:
        try:
            return deserialize_message(raw, self.topic_states[topic].msg_type), None
        except Exception as exc:  # noqa: BLE001 - report and skip bad cloud samples.
            topic_type = self.topic_states[topic].topic_type
            return None, f"{type(exc).__name__}: topic={topic} type={topic_type} error={exc}"

    def wait_for_icp(
        self,
        cloud_stamp_s: float,
        previous_count: int,
        previous_mapper_callbacks: int,
        mapper_log: MapperLogMonitor,
        clock_until_s: float,
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + self.args.icp_timeout_s
        target = cloud_stamp_s - self.args.stamp_tolerance_s
        start = time.monotonic()
        mapper_processed_at: float | None = None
        while time.monotonic() < deadline:
            elapsed = time.monotonic() - start
            self.publish_clock(min(clock_until_s, cloud_stamp_s + elapsed))
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.icp_count > previous_count and self.last_icp_stamp >= target:
                return True, "matched_stamp"
            if self.icp_count > previous_count and self.args.accept_any_new_icp:
                return True, "new_icp"
            mapper_callbacks = mapper_log.poll()
            if mapper_callbacks > previous_mapper_callbacks:
                if mapper_processed_at is None:
                    mapper_processed_at = time.monotonic()
                elif time.monotonic() - mapper_processed_at >= self.args.mapper_log_grace_s:
                    return True, "mapper_callback_processed"
        return False, "timeout"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Bag directory containing metadata.yaml and MCAP files.")
    parser.add_argument("--points-topic", required=True, help="Mapping PointCloud2 topic to step on.")
    parser.add_argument("--icp-topic", default="/mapping/icp_odom")
    parser.add_argument("--exclude-topic", action="append", default=[])
    parser.add_argument("--support-topic", action="append", default=[])
    parser.add_argument(
        "--required-prior-topic",
        action="append",
        default=[],
        help=(
            "Do not release mapping clouds older than the first message seen on this topic. "
            "Use this for odom prior topics such as /mtt_tachometer or /mti100/data."
        ),
    )
    parser.add_argument("--icp-timeout-s", type=float, default=15.0)
    parser.add_argument("--stamp-tolerance-s", type=float, default=0.20)
    parser.add_argument("--max-consecutive-timeouts", type=int, default=20)
    parser.add_argument("--accept-any-new-icp", action="store_true")
    parser.add_argument("--progress-interval", type=int, default=25)
    parser.add_argument(
        "--mapper-log",
        default="",
        help=(
            "Optional mapper stdout log. If set, a '[POINTS_IN] callback done' "
            "or MAPPER_HEARTBEAT callback increment counts as the cloud being processed "
            "even when the quality gate rejects it and no /mapping/icp_odom is published."
        ),
    )
    parser.add_argument(
        "--mapper-log-grace-s",
        type=float,
        default=0.20,
        help="After mapper-log callback progress, wait this long for a late icp_odom before continuing.",
    )
    parser.add_argument(
        "--pre-cloud-settle-s",
        type=float,
        default=0.02,
        help="Small wall-clock delay before each cloud so odom/TF publishers can process support messages.",
    )
    parser.add_argument(
        "--support-lookahead-s",
        type=float,
        default=0.35,
        help="Publish support messages this far ahead of each cloud stamp before releasing the cloud.",
    )
    parser.add_argument("--storage-id", default="mcap")
    return parser.parse_args()


def selected_topics(args: argparse.Namespace, topic_types: dict[str, str]) -> list[str]:
    excluded = set(args.exclude_topic)
    support = set(DEFAULT_SUPPORT_TOPICS)
    support.update(args.support_topic)
    wanted = {args.points_topic}
    wanted.update(topic for topic in support if topic in topic_types)
    return sorted(topic for topic in wanted if topic in topic_types and topic not in excluded)


def main() -> int:
    args = parse_args()
    bag_dir = Path(args.bag).expanduser().resolve()
    if not (bag_dir / "metadata.yaml").exists():
        print(f"ERROR: {bag_dir} is not a ROS bag directory", file=sys.stderr)
        return 2

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=args.storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    topics = selected_topics(args, topic_types)
    if args.points_topic not in topics:
        print(f"ERROR: points topic {args.points_topic} not found in {bag_dir}", file=sys.stderr)
        return 2
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    rclpy.init()
    node = StepReplayNode(args, {topic: topic_types[topic] for topic in topics})
    print(
        f"step replay: bag={bag_dir} points={args.points_topic} topics={len(topics)} "
        f"timeout={args.icp_timeout_s}s",
        flush=True,
    )

    clouds = 0
    skipped_clouds = 0
    support_messages = 0
    bad_support_messages = 0
    bad_clouds = 0
    timeouts = 0
    consecutive_timeouts = 0
    required_prior_topics = set(args.required_prior_topic)
    first_required_prior_s: float | None = None
    mapper_log = MapperLogMonitor(Path(args.mapper_log) if args.mapper_log else None)
    start = time.monotonic()
    last_cloud_stamp = 0.0
    try:
        # Let late subscribers discover publishers before the first latched/static messages.
        warmup_until = time.monotonic() + 1.0
        while time.monotonic() < warmup_until:
            rclpy.spin_once(node, timeout_sec=0.05)

        pending_clouds: list[tuple[str, bytes, int, float]] = []

        while reader.has_next() or pending_clouds:
            if pending_clouds:
                topic, raw, timestamp_ns, bag_time_s = pending_clouds.pop(0)
            else:
                topic, raw, timestamp_ns = reader.read_next()
                bag_time_s = float(timestamp_ns) * 1e-9

            if topic != args.points_topic:
                support_stamp_s, error = node.try_publish_raw(topic, raw, bag_time_s)
                if error is not None:
                    bad_support_messages += 1
                    if bad_support_messages <= 5 or bad_support_messages % args.progress_interval == 0:
                        print(
                            f"step replay bad support: count={bad_support_messages} {error}",
                            file=sys.stderr,
                            flush=True,
                        )
                    continue
                if topic in required_prior_topics and first_required_prior_s is None:
                    first_required_prior_s = support_stamp_s
                support_messages += 1
                rclpy.spin_once(node, timeout_sec=0.0)
                continue

            cloud_msg, error = node.try_deserialize(topic, raw)
            if error is not None:
                bad_clouds += 1
                if bad_clouds <= 5 or bad_clouds % args.progress_interval == 0:
                    print(
                        f"step replay bad cloud: count={bad_clouds} {error}",
                        file=sys.stderr,
                        flush=True,
                    )
                continue
            cloud_stamp_s = msg_stamp_or_bag_time(cloud_msg, bag_time_s)
            support_deadline_s = cloud_stamp_s + max(0.0, args.support_lookahead_s)

            while reader.has_next():
                next_topic, next_raw, next_ts_ns = reader.read_next()
                next_bag_time_s = float(next_ts_ns) * 1e-9
                if next_topic == args.points_topic:
                    pending_clouds.append((next_topic, next_raw, next_ts_ns, next_bag_time_s))
                    continue
                support_stamp_s, error = node.try_publish_raw(next_topic, next_raw, next_bag_time_s)
                if error is not None:
                    bad_support_messages += 1
                    if bad_support_messages <= 5 or bad_support_messages % args.progress_interval == 0:
                        print(
                            f"step replay bad support: count={bad_support_messages} {error}",
                            file=sys.stderr,
                            flush=True,
                        )
                    continue
                if next_topic in required_prior_topics and first_required_prior_s is None:
                    first_required_prior_s = support_stamp_s
                support_messages += 1
                rclpy.spin_once(node, timeout_sec=0.0)
                if support_stamp_s >= support_deadline_s:
                    break

            if first_required_prior_s is not None and cloud_stamp_s < first_required_prior_s - 0.005:
                skipped_clouds += 1
                if skipped_clouds == 1 or skipped_clouds % args.progress_interval == 0:
                    print(
                        f"step replay skip: clouds_skipped={skipped_clouds} "
                        f"cloud_stamp={cloud_stamp_s:.9f} first_prior={first_required_prior_s:.9f}",
                        flush=True,
                    )
                continue

            previous_count = node.icp_count
            previous_mapper_callbacks = mapper_log.poll()
            if args.pre_cloud_settle_s > 0.0:
                end_settle = time.monotonic() + args.pre_cloud_settle_s
                while time.monotonic() < end_settle:
                    rclpy.spin_once(node, timeout_sec=0.002)
            node.publish_clock(cloud_stamp_s)
            node.topic_states[topic].publisher.publish(cloud_msg)
            last_cloud_stamp = cloud_stamp_s
            clouds += 1
            ok, reason = node.wait_for_icp(
                cloud_stamp_s,
                previous_count,
                previous_mapper_callbacks,
                mapper_log,
                support_deadline_s,
            )
            if ok:
                consecutive_timeouts = 0
            else:
                timeouts += 1
                consecutive_timeouts += 1
                print(
                    f"step replay timeout: cloud={clouds} stamp={cloud_stamp_s:.9f} "
                    f"icp_count={node.icp_count} last_icp={node.last_icp_stamp:.9f} reason={reason}",
                    flush=True,
                )
                if consecutive_timeouts >= args.max_consecutive_timeouts:
                    print(
                        f"ERROR: {consecutive_timeouts} consecutive ICP timeouts; stopping step replay",
                        file=sys.stderr,
                        flush=True,
                    )
                    return 3

            if clouds == 1 or clouds % args.progress_interval == 0:
                elapsed = max(time.monotonic() - start, 1e-6)
                print(
                    f"step replay progress: clouds={clouds} support={support_messages} "
                    f"skipped={skipped_clouds} bad_clouds={bad_clouds} "
                    f"bad_support={bad_support_messages} icp={node.icp_count} timeouts={timeouts} "
                    f"mapper_callbacks={mapper_log.callbacks} "
                    f"last_cloud={last_cloud_stamp:.9f} wall={elapsed:.1f}s "
                    f"rate={clouds / elapsed:.2f} clouds/s",
                    flush=True,
                )

        for _ in range(20):
            node.publish_clock(last_cloud_stamp)
            rclpy.spin_once(node, timeout_sec=0.05)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    elapsed = max(time.monotonic() - start, 1e-6)
    print(
        f"step replay done: clouds={clouds} support={support_messages} "
        f"skipped={skipped_clouds} bad_clouds={bad_clouds} "
        f"bad_support={bad_support_messages} icp={node.icp_count} "
        f"timeouts={timeouts} mapper_callbacks={mapper_log.callbacks} "
        f"wall={elapsed:.1f}s "
        f"rate={clouds / elapsed:.2f} clouds/s",
        flush=True,
    )
    return 0 if clouds > 0 else 4


if __name__ == "__main__":
    raise SystemExit(main())
