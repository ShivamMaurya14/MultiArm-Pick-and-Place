from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('multi_arm_bringup')
    stations_file = os.path.join(bringup_dir, 'config', 'stations.yaml')

    nodes = []
    
    # Start 3 Action Servers, one in each namespace
    for i in range(1, 4):
        ns = f'robot{i}'
        pkg_name = f'robot{i}_moveit_config'
        
        # Load MoveIt configs for the specific robot
        moveit_config = MoveItConfigsBuilder(f"robot{i}", package_name=pkg_name).to_moveit_configs()
        
        server_node = Node(
            package='multi_arm_control',
            executable='pick_place_server',
            namespace=ns,
            name='pick_place_server',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'stations_file': stations_file}
            ]
        )
        nodes.append(server_node)

    return LaunchDescription(nodes)
