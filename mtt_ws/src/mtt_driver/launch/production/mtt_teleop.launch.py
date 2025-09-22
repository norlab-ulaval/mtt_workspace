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
from launch_ros.substitutions import FindPackageShare
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

        # Teleop node publishes to cmd_vel/teleop (not directly to cmd_vel)
        Node(
            package='mtt_driver',
            executable='mtt_teleop_joy',
            name='mtt_teleop_joy_node',
            # teleop_joy publishes to 'cmd_vel_raw' -> pipe it into 'cmd_vel/teleop'
            remappings=[('cmd_vel_raw', 'cmd_vel/teleop')],
            output='screen'
        ),

        # Optional command smoother between teleop and wrapper
        Node(
            package='mtt_driver',
            executable='teleop_cmd_smoother',
            name='teleop_cmd_smoother',
            parameters=[{
                'input_topic': 'cmd_vel/teleop',
                'output_topic': 'cmd_vel/teleop_smoothed',
                'max_accel_linear': 1.5,
                'max_accel_angular': 1.5,
                'rate_hz': 50.0,
            }],
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
