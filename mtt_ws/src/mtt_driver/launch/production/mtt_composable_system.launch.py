#!/usr/bin/env python3
"""
MTT Composable System Launch File

This launch file starts the complete MTT composable architecture including:
- MTT driver wrapper (hardware abstraction + ROS integration + safety)
- MTT odometry node (dedicated composable odometry calculations)
- Joystick input (optional)
- Teleop controller (optional)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare(package='mtt_description').find('mtt_description')
    urdf_path = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')

    # --- vcan bring-up (idempotent, no modprobe) ---
    setup_vcan_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            # create only if missing
            'ip link show vcan0 >/dev/null 2>&1 || sudo ip link add dev vcan0 type vcan; '
            # always bring it up
            'sudo ip link set up vcan0; '
            'echo "[vcan] vcan0 is present and UP."'
        ],
        name='setup_vcan',
        output='screen',
        condition=IfCondition(LaunchConfiguration('setup_vcan')),
    )

    # --- real CAN bring-up (with bitrate) ---
    setup_real_can_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            # uses launch args via env-style expansion by passing them into the shell
            'IFACE="$(echo $CAN_IFACE)"; RATE="$(echo $CAN_RATE)"; '
            'sudo ip link set "$IFACE" down 2>/dev/null || true; '
            'sudo ip link set "$IFACE" up type can bitrate "$RATE"; '
            'echo "[can] ${IFACE} UP @ ${RATE} bps."'
        ],
        additional_env={
            'CAN_IFACE': LaunchConfiguration('can_interface'),
            'CAN_RATE':  LaunchConfiguration('can_bitrate'),
        },
        name='setup_real_can',
        output='screen',
        condition=IfCondition(LaunchConfiguration('setup_real_can')),
    )

    return LaunchDescription([
        # -------- Arguments --------
        DeclareLaunchArgument(
            'setup_vcan',
            default_value='false',
            description='Bring up vcan0 (Docker/testing - uses sudo).'
        ),
        DeclareLaunchArgument(
            'setup_real_can',
            default_value='false',
            description='Bring up real CAN interface with bitrate (host sudo).'
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name for real hardware'
        ),
        DeclareLaunchArgument(
            'can_bitrate',
            default_value='250000',
            description='Bitrate for real CAN interface (e.g., 250000, 500000).'
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
            description='Enable teleoperation (joystick + teleop controller + smoother)'
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
            'odometry_broadcast_tf',
            default_value='true',
            description='Whether mtt_odometry_manager should publish odom->base TF'
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

        # -------- Actions / Nodes --------

        # 0) (Optional) Ensure vcan0 exists & is up BEFORE anything else
        setup_vcan_process,

        # 0b) (Optional) Ensure real CAN is up with bitrate BEFORE anything else
        setup_real_can_process,

        # 1) Robot description (URDF → TF tree for real runs)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
            condition=IfCondition(LaunchConfiguration('publish_description'))
        ),

        # 2) Joint controller (cmd_vel → joints) and joint_states
        Node(
            package='mtt_driver',
            executable='mtt_joint_controller',
            name='mtt_joint_controller',
            # No remapping - should receive final muxed commands from twist_mux
            output='screen'
        ),

        # 3) Optional static map->odom identity TF (RViz dead-reckoning)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_odom',
            arguments=['0','0','0','0','0','0','map', LaunchConfiguration('odom_frame')],
            condition=IfCondition(LaunchConfiguration('enable_map_frame'))
        ),

        # 4) Driver (now receives commands from twist_mux on cmd_vel)
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

        # 4.5) Twist Mux - Command multiplexer (replaces internal muxing)
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[os.path.join(FindPackageShare(package='mtt_driver').find('mtt_driver'), 'config', 'twist_mux.yaml')],
            remappings=[('cmd_vel_out', 'cmd_vel')],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # 5) Odometry manager
        Node(
            package='mtt_driver',
            executable='mtt_odometry_manager',
            name='mtt_odometry_manager',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('driver_log_level')],
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                # Odometry now listens to the final muxed cmd_vel topic
                'cmd_vel_topic': 'cmd_vel',
                # Toggle TF broadcasting via launch arg
                'broadcast_tf': LaunchConfiguration('odometry_broadcast_tf'),
                # Steering control and anti-drift tuning - using centralized vehicle parameters
                'steer_control_mode': 'closed_loop',
                # max_articulation_angle and max_yaw_rate now come from centralized vehicle parameters
                'pivot_turn_enabled': False,
                'min_turn_speed_ms': 0.05,
                'yaw_slip_factor': 0.6,
            }],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0
        ),

        # 6) Joystick (joy_linux)
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

        # 6.5) Teleop command smoother (decays to zero on input inactivity)
        Node(
            package='mtt_driver',
            executable='teleop_cmd_smoother',
            name='teleop_cmd_smoother',
            parameters=[{
                'input_topic': 'cmd_vel/teleop',
                'output_topic': 'cmd_vel/teleop_smoothed',
                'input_timeout': 0.5,
                'rate_hz': 50.0,
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_teleop')),
            respawn=True
        ),

        # 7) Teleop
        Node(
            package='mtt_driver',
            executable='mtt_teleop_joy',
            name='mtt_teleop_joy_node',
            remappings=[('cmd_vel_raw', 'cmd_vel/teleop')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_teleop')),
            respawn=True
        ),

        # 8) RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='mtt_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])
