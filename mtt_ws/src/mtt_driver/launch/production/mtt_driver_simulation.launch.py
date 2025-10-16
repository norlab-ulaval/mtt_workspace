import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    description_share = FindPackageShare(package='mtt_description').find('mtt_description')
    urdf_path = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')

    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Base frame of the robot (child of odom)'
    )

    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='vcan0',
        description='CAN interface name for real hardware'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'driver_log_level',
        default_value='INFO',
        description='Driver logging level (DEBUG, INFO, WARNING, ERROR)'
    )
    
    declare_control_frequency = DeclareLaunchArgument(
        'control_frequency_hz',
        default_value='50.0',
        description='Driver control loop frequency (Hz)'
    )

    declare_can_frame_frequency = DeclareLaunchArgument(
        'can_frame_frequency_hz',
        default_value='20.0',
        description='CAN frame frequency (Hz)'
    )

    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odom frame (parent of base frame)'
    )

    declare_odometry_broadcast = DeclareLaunchArgument(
        'odometry_broadcast_tf',
        default_value='true',
        description='Whether mtt_odometry_manager should publish odom->base TF'
    )
    

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
    )

    mtt_driver = Node(
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
    )

    odometry_manager = Node(
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
    )

    # 2) Joint controller (cmd_vel → joints) and joint_states
    mtt_joint_controller = Node(
        package='mtt_driver',
        executable='mtt_joint_controller',
        name='mtt_joint_controller',
        # No remapping - should receive final muxed commands from twist_mux
        output='screen'
    )

    # 1) Robot description (URDF → TF tree for real runs)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        # condition=IfCondition(LaunchConfiguration('publish_description'))
    )

    # 3) Optional static map->odom identity TF (RViz dead-reckoning)
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom',
        arguments=['0','0','0','0','0','0','map', LaunchConfiguration('odom_frame')],
        # condition=IfCondition(LaunchConfiguration('enable_map_frame'))
    )

            # 4.5) Twist Mux - Command multiplexer (replaces internal muxing)
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(FindPackageShare(package='mtt_driver').find('mtt_driver'), 'config', 'twist_mux.yaml')],
        remappings=[('cmd_vel_out', 'cmd_vel')],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # 6) Joystick (joy_linux)
    joy_linux = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.15,
            'device_name': '/dev/input/js0'
        }],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('enable_joystick')),
        respawn=True
    )

    # 7) Teleop
    teleop_node = Node(
        package='mtt_driver',
        executable='mtt_teleop_joy',
        name='mtt_teleop_joy_node',
        remappings=[('cmd_vel_raw', 'cmd_vel/teleop')],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('enable_teleop')),
        respawn=True
    )
    
    # 6.5) Teleop command smoother (decays to zero on input inactivity)
    teleop_cmd_smoother = Node(
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
        # condition=IfCondition(LaunchConfiguration('enable_teleop')),
        respawn=True
    )
    
    ld = LaunchDescription()

    ld.add_action(declare_base_frame)
    ld.add_action(declare_can_interface)
    ld.add_action(declare_log_level)
    ld.add_action(declare_control_frequency)
    ld.add_action(declare_can_frame_frequency)
    ld.add_action(declare_odom_frame)
    ld.add_action(declare_odometry_broadcast)

    ld.add_action(setup_vcan_process)
    ld.add_action(mtt_driver)
    ld.add_action(odometry_manager)
    ld.add_action(mtt_joint_controller)
    ld.add_action(robot_state_publisher)
    ld.add_action(static_transform_publisher)
    ld.add_action(twist_mux)
    ld.add_action(joy_linux)
    ld.add_action(teleop_node)
    ld.add_action(teleop_cmd_smoother)

    return ld