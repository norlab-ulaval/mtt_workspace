#!/usr/bin/env python3
"""
MTT Teleop Launch File

This launch file starts the complete MTT teleoperation system including:
- Joystick input (joy_linux)
- MTT driver wrapper with configurable logging
- Teleop controller

Usage examples:
  # Default setup for real hardware
  ros2 launch mtt_driver mtt_teleop.launch.py

  # Test mode with virtual CAN
  ros2 launch mtt_driver mtt_teleop.launch.py test_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name'
        ),
        DeclareLaunchArgument(
            'test_mode',
            default_value='false',
            description='Enable test mode (uses vcan0)'
        ),
        DeclareLaunchArgument(
            'driver_log_level',
            default_value='INFO',
            description='Driver logging level (DEBUG, INFO, WARNING, ERROR)'
        ),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.15,
                'device_name': '/dev/input/js0'
            }],
            output='screen'
        ),

        Node(
            package='mtt_driver',
            executable='mtt_ros_wrapper',
            name='mtt_driver_node',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                'test_mode': LaunchConfiguration('test_mode'),
                'driver_log_level': LaunchConfiguration('driver_log_level'),
            }],
            output='screen'
        ),

        Node(
            package='mtt_driver',
            executable='mtt_teleop_joy',
            name='mtt_teleop_joy_node',
            output='screen'
        ),

        # Multi-Mode Odometry Manager
        Node(
            package='mtt_driver',
            executable='mtt_odometry_manager',
            name='mtt_odometry_manager',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('driver_log_level')],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),
    ])
