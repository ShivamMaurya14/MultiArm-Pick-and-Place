import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('multi_arm_bringup')
    stations_file = os.path.join(bringup_dir, 'config', 'stations.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    nodes = []
    
    # Start 3 Action Servers, one in each namespace
    for i in range(1, 4):
        ns = f'robot{i}'
        prefix = f'robot{i}_'
        
        server_node = Node(
            package='multi_arm_control',
            executable='pick_place_server',
            namespace=ns,
            name='pick_place_server',
            output='screen',
            parameters=[
                {'stations_file': stations_file},
                {'prefix': prefix},
                {'use_sim_time': use_sim_time}
            ]
        )
        nodes.append(server_node)

    return LaunchDescription(nodes)
