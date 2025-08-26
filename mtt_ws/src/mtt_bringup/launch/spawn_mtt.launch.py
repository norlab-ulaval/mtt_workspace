import os
from pathlib import Path


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable

from launch_ros.actions import Node


def generate_launch_description():
    # mode = "turtle"
    mode = "mtt"

    mtt_description_dir = get_package_share_directory('mtt_description')
    bringup_dir = get_package_share_directory('nav2_minimal_tb3_sim')



    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='mtt_robot',
        description='name of the robot')
    

    if mode == "mtt":
        declare_robot_sdf_cmd = DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(mtt_description_dir, 'urdf', 'robot.sdf'),
            description='Full path to robot sdf file to spawn the robot in gazebo')
    else:
        declare_robot_sdf_cmd = DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(bringup_dir, 'urdf', 'gz_waffle.sdf.xacro'),
            description='Full path to robot sdf file to spawn the robot in gazebo')
  
    if mode == "mtt":
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            parameters=[
                {
                    'expand_gz_topic_names': True,
                    'use_sim_time': True,
                }
            ],
            arguments=[
                # Lidar Scan
                '/world/default/model/mtt_robot/link/base_footprint/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/model/mtt_robot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
                # Ground-truth Odometry from OdometryPublisher (gz.msgs.Odometry <-> nav_msgs/Odometry)
                '/model/mtt_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

                # ROS remappings
                '--ros-args', '-r',
                '/world/default/model/mtt_robot/link/base_footprint/sensor/lidar/scan:=/gz_scan',
                '-r', '/model/mtt_robot/pose:=/gz_pose',
                '-r', '/model/mtt_robot/odometry:=/ground_truth_odom',

                # # If only one remap this form works:
                # '/world/default/model/mtt_robot/link/base_footprint/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                # '--ros-args', '-r',
                # '/world/default/model/mtt_robot/link/base_footprint/sensor/lidar/scan:=/gz_scan',
            ],
            output='screen',
        )

        remapper_node = Node(
            package='mtt_bringup',
            executable='scan_frame_remapper.py',
            name='scan_frame_remapper',
            output='screen')
    # Removed custom odom_publisher_simul: now using Gazebo OdometryPublisher plugin bridged above.

    else:
        bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[
            {
                'config_file': os.path.join(
                    bringup_dir, 'configs', 'turtlebot3_waffle_bridge.yaml'
                ),
                'expand_gz_topic_names': True,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-name', robot_name,
            '-string', Command([
                FindExecutable(name='xacro'), ' ', 'namespace:=',
                LaunchConfiguration('namespace'), ' ', robot_sdf]),
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(bringup_dir, 'models'))
    set_env_vars_resources2 = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(bringup_dir)).parent.resolve()))

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)

    ld.add_action(bridge)
    ld.add_action(spawn_model)

    if mode == "mtt":
        ld.add_action(remapper_node)
        ld.add_action(Node(
            package='mtt_bringup',
            executable='ground_truth_odom_tf_broadcaster.py',
            name='ground_truth_odom_tf_broadcaster',
            output='screen',
            parameters=[{'odom_topic': '/ground_truth_odom', 'odom_frame': 'odom', 'base_frame': 'base_link'}]
        ))
    # Note: we republish to /odom inside the broadcaster node to avoid external dependency on topic_tools

        
    return ld
