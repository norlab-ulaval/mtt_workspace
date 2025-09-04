from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration
import subprocess


def setup_can(context, *args, **kwargs):
    """Check for can0 and bring it up if present."""
    try:
        subprocess.run(
            ["ip", "link", "show", "can0"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        # can0 exists → bring it up
        subprocess.run(
            ["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "250000"],
            check=True,
        )
        print("[INFO] can0 found and configured (250000 bitrate)")
    except subprocess.CalledProcessError:
        print("[WARN] can0 not found or already setup, skipping setup")


    return []


def generate_launch_description():

    # Declare launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        # default_value='vcan0',
        description='CAN interface to use for testing (default: vcan0 for virtual CAN)'
    )
    
    setup_vcan_arg = DeclareLaunchArgument(
        'setup_vcan',
        default_value='False',
        description='Whether to automatically setup virtual CAN interface'
    )

    # Setup virtual CAN interface (conditional)
    # setup_vcan_process = ExecuteProcess(
    #     cmd=[
    #         'bash', '-c',
    #         'sudo modprobe vcan && '
    #         'sudo ip link add dev vcan0 type vcan && '
    #         'sudo ip link set up vcan0 && '
    #         'echo "Virtual CAN interface vcan0 created successfully" || '
    #         'echo "Virtual CAN interface vcan0 already exists or setup failed"'
    #     ],
    #     name='setup_vcan',
    #     output='screen',
    #     # condition=IfCondition(LaunchConfiguration('setup_vcan'))
    # )

    canbus_bringup = OpaqueFunction(
        function=setup_can,
        condition=UnlessCondition(LaunchConfiguration("setup_vcan"))
        )

    mtt_driver_node = Node(
        package='mtt_driver',
        executable='mtt_ros_wrapper',
        name='mtt_ros_wrapper',
        output='screen',
        parameters=[{
            'can_interface': 'vcan0',
            'test_mode': True
        }]
    )

    ld = LaunchDescription()

    ld.add_action(can_interface_arg)
    ld.add_action(setup_vcan_arg)
    # ld.add_action(setup_vcan_process)

    ld.add_action(canbus_bringup)
    ld.add_action(mtt_driver_node)

    return ld