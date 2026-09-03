from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('multi_arm_bringup')
    stations_file = os.path.join(bringup_dir, 'config', 'stations.yaml')

    nodes = []
    
    # Start 3 Action Servers, one in each namespace
    for i in range(1, 4):
        ns = f'robot{i}'
        server_node = Node(
            package='multi_arm_control',
            executable='pick_place_server',
            namespace=ns,
            name='pick_place_server',
            output='screen',
            parameters=[{'stations_file': stations_file}]
        )
        nodes.append(server_node)

    return LaunchDescription(nodes)
