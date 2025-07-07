from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.15}]
        ),
        Node(
            package='mtt_driver',
            executable='mtt_teleop_joy',
            name='mtt_teleop_joy'
        ),
        Node(
            package='mtt_driver',
            executable='mtt_ros_wrapper',
            name='mtt_ros_wrapper',
            output='screen'
        ),
    ])
