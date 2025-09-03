from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
import os
import subprocess


def setup_can(context, *args, **kwargs):
    """Check for can0 and bring it up if present."""
    try:
        subprocess.run(
            ["ip", "link", "show", "can0"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        # can0 exists → bring it up
        subprocess.run(
            ["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "250000"],
            check=True,
        )
        print("[INFO] can0 found and configured (250000 bitrate)")
    except subprocess.CalledProcessError:
        print("[WARN] can0 not found or already setup, skipping setup")

    return []


def generate_launch_description():

    canbus_bringup = OpaqueFunction(function=setup_can)

    mtt_driver_node = Node(
        package="mtt_driver",
        executable="mtt_driver",
        name="mtt_driver",
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(canbus_bringup)
    ld.add_action(mtt_driver_node)

    return ld