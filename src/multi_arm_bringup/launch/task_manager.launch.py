from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi_arm_control',
            executable='task_manager',
            name='global_task_manager',
            output='screen'
        )
    ])
