from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '/home/user1/mtt_ws/install/the_mtt_bringup/lib/the_mtt_bringup/wheel_group_publisher.py'],
            output='screen'
        ),
    ])
