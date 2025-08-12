from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='mtt_description').find('mtt_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    gazebo_pkg_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot']
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])