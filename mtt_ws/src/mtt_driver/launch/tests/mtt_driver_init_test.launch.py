#!/usr/bin/env python3
"""
Launch file for testing MTT Driver initialization without ROS wrapper.
This launch file tests only the driver's init frame and basic functionality.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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

    driver_log_level_arg = DeclareLaunchArgument(
        'driver_log_level',
        default_value='DEBUG',
        description='Driver logging level for testing'
    )

    # Setup virtual CAN interface (modern, Docker-friendly)
    setup_vcan_process = ExecuteProcess(
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
        condition=IfCondition(LaunchConfiguration('setup_vcan'))
    )

    # Log info about the test
    test_info = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            'MTT DRIVER INITIALIZATION TEST LAUNCH\n',
            'This test runs the MTT driver test node to verify basic functionality.\n',
            '=' * 80, '\n'
        ]
    )

    # MTT Driver test node
    mtt_driver_test_node = Node(
        package='mtt_driver',
        executable='mtt_test_node',
        name='mtt_driver_init_test',
        output='screen',
        arguments=[LaunchConfiguration('can_interface')],
        emulate_tty=True,
    )

    return LaunchDescription([
        # Launch arguments
        can_interface_arg,
        setup_vcan_arg,
        driver_log_level_arg,
        
        # Setup virtual CAN if requested
        setup_vcan_process,
        
        # Log test information
        test_info,
        
        # Run the driver test
        mtt_driver_test_node,
    ])
