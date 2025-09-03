from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mtt_driver',
            executable='mtt_ros_wrapper',
            name='mtt_ros_wrapper',
            output='screen',
            parameters=[{
                'can_interface': 'vcan0',
                'test_mode': True
            }]
        ),
        
        Node(
            package='mtt_driver',
            executable='mtt_test_node',
            name='mtt_test_node',
            output='screen'
        ),
    ])
