#!/usr/bin/env python3
"""
MTT Composable System Launch File

This launch file starts the complete MTT composable architecture including:
- MTT driver wrapper (hardware abstraction + ROS integration + safety)
- MTT odometry node (dedicated composable odometry calculations)
- Joystick input (optional)
- Teleop controller (optional)

Architecture:
  Driver → Wrapper → Odometry Node
     ↓         ↓           ↓
  hardware  /mtt_tachometer  /mtt_odometry

Usage examples:
  # Complete system for real hardware
  ros2 launch mtt_driver mtt_composable_system.launch.py

  # Test mode with virtual CAN
  ros2 launch mtt_driver mtt_composable_system.launch.py test_mode:=true

  # Driver + odometry only (no teleop)
  ros2 launch mtt_driver mtt_composable_system.launch.py enable_teleop:=false

  # Custom logging
  ros2 launch mtt_driver mtt_composable_system.launch.py driver_log_level:=DEBUG
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name for real hardware'
        ),
        DeclareLaunchArgument(
            'test_mode',
            default_value='false',
            description='Enable test mode (uses vcan0 instead of real CAN)'
        ),
        DeclareLaunchArgument(
            'driver_log_level',
            default_value='INFO',
            description='Driver logging level (DEBUG, INFO, WARNING, ERROR)'
        ),
        DeclareLaunchArgument(
            'enable_teleop',
            default_value='true',
            description='Enable teleoperation (joystick + teleop controller)'
        ),
        DeclareLaunchArgument(
            'enable_joystick',
            default_value='true',
            description='Enable joystick input node'
        ),

        # Core MTT System Nodes
        
        # MTT Driver Wrapper - Hardware abstraction + ROS integration + safety
        Node(
            package='mtt_driver',
            executable='mtt_ros_wrapper',
            name='mtt_ros_wrapper',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                'test_mode': LaunchConfiguration('test_mode'),
                'driver_log_level': LaunchConfiguration('driver_log_level'),
            }],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0
        ),

        # MTT Multi-Mode Odometry Manager - Supports multiple driving modes
        Node(
            package='mtt_driver',
            executable='mtt_odometry_manager',
            name='mtt_odometry_manager',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('driver_log_level')],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0
        ),

        # Teleoperation Nodes (optional)
        
        # Joystick input node
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.15,
                'device_name': '/dev/input/js0'
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_joystick')),
            respawn=True
        ),

        # MTT Teleop controller
        Node(
            package='mtt_driver',
            executable='mtt_teleop_joy',
            name='mtt_teleop_joy_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_teleop')),
            respawn=True
        ),
    ])
