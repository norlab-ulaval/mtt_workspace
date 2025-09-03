#!/usr/bin/env python3
"""
Launch file for testing MTT Driver initialization without ROS wrapper.
This launch file tests only the driver's init frame and basic functionality.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='vcan0',
        description='CAN interface to use for testing (default: vcan0 for virtual CAN)'
    )
    
    setup_vcan_arg = DeclareLaunchArgument(
        'setup_vcan',
        default_value='true',
        description='Whether to automatically setup virtual CAN interface'
    )

    # Setup virtual CAN interface (conditional)
    setup_vcan_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sudo modprobe vcan && '
            'sudo ip link add dev vcan0 type vcan && '
            'sudo ip link set up vcan0 && '
            'echo "Virtual CAN interface vcan0 created successfully" || '
            'echo "Virtual CAN interface vcan0 already exists or setup failed"'
        ],
        name='setup_vcan',
        output='screen',
        condition=IfCondition(LaunchConfiguration('setup_vcan'))
    )

    # Log info about the test
    test_info = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            'MTT DRIVER INITIALIZATION TEST LAUNCH\n',
            'This test runs the MTT driver alone without the ROS wrapper\n',
            'to verify basic initialization and frame generation.\n',
            '=' * 80, '\n'
        ]
    )

    # MTT Driver test node
    mtt_driver_test_node = Node(
        package='mtt_driver',
        executable='mtt_test_node',
        name='mtt_driver_init_test',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
        }],
        arguments=[LaunchConfiguration('can_interface')]
    )

    return LaunchDescription([
        # Launch arguments
        can_interface_arg,
        setup_vcan_arg,
        
        # Setup virtual CAN if requested
        setup_vcan_process,
        
        # Log test information
        test_info,
        
        # Run the driver test
        mtt_driver_test_node,
    ])
