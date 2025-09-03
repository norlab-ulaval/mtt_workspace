from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_share = FindPackageShare('mtt_driver')
    
    return LaunchDescription([
        # Setup virtual CAN interface
        ExecuteProcess(
            cmd=['sudo', 'modprobe', 'vcan'],
            name='load_vcan_module',
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['sudo', 'ip', 'link', 'delete', 'vcan0'],
            name='delete_existing_vcan',
            output='screen',
            on_exit='continue'
        ),
        
        ExecuteProcess(
            cmd=['sudo', 'ip', 'link', 'add', 'dev', 'vcan0', 'type', 'vcan'],
            name='create_vcan',
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['sudo', 'ip', 'link', 'set', 'vcan0', 'up'],
            name='activate_vcan',
            output='screen'
        ),
        
        # Start mockserver after a small delay
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='mtt_driver',
                    executable='mtt_mock_server',
                    name='mtt_mock_server',
                    arguments=['-c', 'vcan0'],
                    output='screen'
                )
            ]
        ),
        
        # Start MTT ROS wrapper in test mode
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='mtt_driver',
                    executable='mtt_ros_wrapper',
                    name='mtt_ros_wrapper',
                    output='screen',
                    parameters=[{
                        'can_interface': 'vcan0',
                        'test_mode': True
                    }]
                )
            ]
        ),
        
        # Start test node
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='mtt_driver',
                    executable='mtt_test_node',
                    name='mtt_test_node',
                    output='screen'
                )
            ]
        ),
    ])
