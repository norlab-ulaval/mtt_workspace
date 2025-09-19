#!/usr/bin/env python3
"""
MTT Full System Test Launch File

This launch file tests the complete MTT system without mock server:
- Sets up vcan0 (Docker-friendly, idempotent)
- Starts MTT ROS wrapper with test parameters
- Starts test node for validation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # -------- Arguments --------
        DeclareLaunchArgument(
            'setup_vcan',
            default_value='true',
            description='Setup vcan0 interface (Docker-friendly)'
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='vcan0',
            description='CAN interface for testing'
        ),
        DeclareLaunchArgument(
            'driver_log_level',
            default_value='DEBUG',
            description='Driver logging level for testing'
        ),

        # -------- vcan Setup (Modern, Docker-friendly) --------
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
        
        # -------- MTT System Test (No Mock Server) --------
        
        # Start MTT ROS wrapper in test mode after vcan setup
        TimerAction(
            period=2.0,
            actions=[
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
                )
            ]
        ),
        
        # Start odometry manager
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='mtt_driver',
                    executable='mtt_odometry_manager',
                    name='mtt_odometry_manager',
                    output='screen',
                    parameters=[{
                        'base_frame': 'base_footprint',
                        'odom_frame': 'odom'
                    }],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('driver_log_level')],
                    emulate_tty=True,
                )
            ]
        ),
        
        # Start test node for validation
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='mtt_driver',
                    executable='mtt_test_node',
                    name='mtt_test_node',
                    output='screen',
                    arguments=[LaunchConfiguration('can_interface')],
                    on_exit=[
                        # When test node exits, terminate all other processes
                        ExecuteProcess(
                            cmd=['pkill', '-f', 'mtt_ros_wrapper'],
                            name='cleanup_wrapper',
                            output='screen'
                        ),
                        ExecuteProcess(
                            cmd=['pkill', '-f', 'mtt_odometry_manager'],
                            name='cleanup_odometry',
                            output='screen'
                        )
                    ]
                )
            ]
        ),
    ])
