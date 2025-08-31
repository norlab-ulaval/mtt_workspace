from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Use virtual CAN interface for testing (vcan0) instead of real hardware (can0)'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='vcan0',
        description='CAN interface to use (can0 for robot, vcan0 for testing)'
    )
    
    # Get launch configuration values
    test_mode = LaunchConfiguration('test_mode')
    can_interface = LaunchConfiguration('can_interface')
    
    return LaunchDescription([
        test_mode_arg,
        can_interface_arg,
        
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
            output='screen',
            parameters=[{
                'can_interface': can_interface,
                'test_mode': test_mode
            }]
        ),
    ])
