#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "joy_device",
            default_value="/dev/input/js0",
            description="Joystick device on the operator computer.",
        ),
        DeclareLaunchArgument(
            "deadzone",
            default_value="0.15",
            description="Joystick deadzone for joy_linux.",
        ),
        DeclareLaunchArgument(
            "max_linear_speed",
            default_value="0.3",
            description="Maximum operator linear speed in m/s.",
        ),
        DeclareLaunchArgument(
            "max_angular_speed",
            default_value="0.3",
            description="Maximum operator angular speed in rad/s.",
        ),
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_node",
            parameters=[{
                "deadzone": LaunchConfiguration("deadzone"),
                "device_name": LaunchConfiguration("joy_device"),
            }],
            output="screen",
        ),
        Node(
            package="mtt_driver",
            executable="mtt_teleop_joy",
            name="mtt_operator_teleop",
            parameters=[{
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_angular_speed": LaunchConfiguration("max_angular_speed"),
            }],
            remappings=[("cmd_vel_raw", "cmd_vel/teleop")],
            output="screen",
        ),
    ])
