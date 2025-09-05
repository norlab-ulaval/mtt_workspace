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

  # Driver + odometry only (no teleop)
  ros2 launch mtt_driver mtt_composable_system.launch.py enable_teleop:=false

  # Custom logging
  ros2 launch mtt_driver mtt_composable_system.launch.py driver_log_level:=DEBUG
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    description_share = FindPackageShare(package='mtt_description').find('mtt_description')
    urdf_path = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name for real hardware'
        ),
        DeclareLaunchArgument(
            'driver_log_level',
            default_value='INFO',
            description='Driver logging level (DEBUG, INFO, WARNING, ERROR)'
        ),
        DeclareLaunchArgument(
            'control_frequency_hz',
            default_value='50.0',
            description='Driver control loop frequency (Hz)'
        ),
        DeclareLaunchArgument(
            'can_frame_frequency_hz',
            default_value='20.0',
            description='CAN frame frequency (Hz)'
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
        DeclareLaunchArgument(
            'publish_description',
            default_value='true',
            description='Publish robot_state_publisher for real hardware run'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Base frame of the robot (child of odom)'
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odom frame (parent of base frame)'
        ),
        DeclareLaunchArgument(
            'enable_map_frame',
            default_value='true',
            description='Publish static map->odom identity transform'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(description_share, 'rviz', 'urdf_config.rviz'),
            description='RViz config file path'
        ),

        # Core MTT System Nodes
        # Robot description: provides base_link (URDF) needed for TF tree in real hardware launch
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
            condition=IfCondition(LaunchConfiguration('publish_description'))
        ),

        # Joint controller: converts cmd_vel to joint movements and publishes joint_states
        Node(
            package='mtt_driver',
            executable='mtt_joint_controller',
            name='mtt_joint_controller',
            output='screen'
        ),

        # TF is published dynamically by odometry manager
        # Optional static map->odom identity for RViz (dead reckoning visualization)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_odom',
            arguments=['0','0','0','0','0','0','map', LaunchConfiguration('odom_frame')],
            condition=IfCondition(LaunchConfiguration('enable_map_frame'))
        ),
        
        # MTT Driver Wrapper - Hardware abstraction + ROS integration + safety
        Node(
            package='mtt_driver',
            executable='mtt_ros_wrapper',
            name='mtt_ros_wrapper',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                'driver_log_level': LaunchConfiguration('driver_log_level'),
                'control_frequency_hz': LaunchConfiguration('control_frequency_hz'),
                'can_frame_frequency_hz': LaunchConfiguration('can_frame_frequency_hz'),
                'base_frame': LaunchConfiguration('base_frame'),
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
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame')
            }],
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

        # Optional RViz (mirrors mtt_description launch capability)
        Node(
            package='rviz2',
            executable='rviz2',
            name='mtt_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])
