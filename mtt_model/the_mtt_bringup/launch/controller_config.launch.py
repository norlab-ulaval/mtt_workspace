import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    yaw_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["yaw_controller"],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_group_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    ld.add_action(wheel_controller_spawner)
    ld.add_action(joint_broad_spawner)
    ld.add_action(yaw_controller_spawner)

    return ld