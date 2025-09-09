#!/usr/bin/env python3
"""
Basic MTT Test Launch File

Simple test with MTT wrapper and test node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # -------- Arguments --------
        DeclareLaunchArgument(
            'setup_vcan',
            default_value='true',
            description='Setup vcan0 interface automatically'
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='vcan0',
            description='CAN interface for testing'
        ),
        DeclareLaunchArgument(
            'driver_log_level',
            default_value='INFO',
            description='Driver logging level'
        ),

        # -------- vcan Setup --------
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                # create only if missing
                'ip link show vcan0 >/dev/null 2>&1 || sudo ip link add dev vcan0 type vcan; '
                # always bring it up
                'sudo ip link set up vcan0; '
                'echo "[TEST] vcan0 is present and UP."'
            ],
            name='setup_vcan',
            output='screen',
            condition=IfCondition(LaunchConfiguration('setup_vcan')),
        ),

        # -------- Nodes --------
        Node(
            package='mtt_driver',
            executable='mtt_ros_wrapper',
            name='mtt_ros_wrapper',
            output='screen',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                'driver_log_level': LaunchConfiguration('driver_log_level'),
                'control_frequency_hz': 10.0,  # Lower for testing
                'can_frame_frequency_hz': 5.0,  # Lower for testing
                'base_frame': 'base_footprint',
            }],
            emulate_tty=True,
        ),
        
        Node(
            package='mtt_driver',
            executable='mtt_test_node',
            name='mtt_test_node',
            output='screen',
            arguments=[LaunchConfiguration('can_interface')]
        ),
    ])
