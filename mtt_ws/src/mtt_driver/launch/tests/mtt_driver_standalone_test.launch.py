#!/usr/bin/env python3
"""
Launch file for testing MTT Driver initialization with standalone script.
This launch file runs a standalone test script that verifies the driver's init frame and basic functionality.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get the current workspace path
    workspace_root = '/home/ws/mtt_ws'
    script_path = os.path.join(workspace_root, 'src', 'mtt_driver', 'scripts', 'test_driver_init.py')
    driver_path = os.path.join(workspace_root, 'src', 'mtt_driver', 'mtt_driver')
    
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
            'MTT DRIVER STANDALONE TEST LAUNCH\n',
            'This test runs the MTT driver standalone script\n',
            'to verify basic initialization and frame generation.\n',
            f'Script path: {script_path}\n',
            f'Driver path: {driver_path}\n',
            '=' * 80, '\n'
        ]
    )

    # MTT Driver test process
    mtt_driver_test_process = ExecuteProcess(
        cmd=[
            'python3',
            script_path,
            LaunchConfiguration('can_interface')
        ],
        name='mtt_driver_standalone_test',
        output='screen',
        cwd=driver_path
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
        mtt_driver_test_process,
    ])
