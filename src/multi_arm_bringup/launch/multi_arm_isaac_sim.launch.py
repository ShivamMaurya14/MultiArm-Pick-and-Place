import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot1_moveit = get_package_share_directory('robot1_moveit_config')
    robot2_moveit = get_package_share_directory('robot2_moveit_config')
    robot3_moveit = get_package_share_directory('robot3_moveit_config')

    # Common arguments for all MoveIt nodes
    # Assuming Isaac Sim provides the joint states and we use mock hardware or direct simulation control
    # For Isaac Sim, use_sim_time must be true
    common_args = {
        'use_sim_time': use_sim_time,
        'ur_type': 'ur10e',
        'launch_rviz': 'false', # Start one global RViz manually or separate if needed
        'launch_servo': 'false'
    }

    group_robot1 = GroupAction([
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot1_moveit, 'launch', 'ur_moveit.launch.py')),
            launch_arguments=common_args.items()
        )
    ])

    group_robot2 = GroupAction([
        PushRosNamespace('robot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot2_moveit, 'launch', 'ur_moveit.launch.py')),
            launch_arguments=common_args.items()
        )
    ])

    group_robot3 = GroupAction([
        PushRosNamespace('robot3'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot3_moveit, 'launch', 'ur_moveit.launch.py')),
            launch_arguments=common_args.items()
        )
    ])

    return LaunchDescription([
        group_robot1,
        group_robot2,
        group_robot3
    ])
