import launch
from launch_ros.actions import Node


def generate_launch_description():

    # ros_joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     output='screen',
    #     parameters=[{'dev': '/dev/input/js0'}]
    # )
    
    ros_joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.15,
            'device_name': '/dev/input/js0'
        }]
    )

    
    mtt_joy_mapper = Node(
        package='mtt_bringup',
        executable='mtt_joy_mapper.py',
        name='mtt_joy_mapper',
        output='screen')
    
    ld = launch.LaunchDescription()

    ld.add_action(ros_joy_node)
    ld.add_action(mtt_joy_mapper)

    
    return ld