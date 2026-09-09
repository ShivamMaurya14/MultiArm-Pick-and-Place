import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = []
    
    # 1. Simulation clock & Gazebo parameters (Target: ROS 2 Jazzy + Gazebo Harmonic / Gz Sim 8)
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock from Gazebo Harmonic (gz-sim8)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_args",
            default_value="-r -v 3 empty.sdf",
            description="Gazebo Harmonic simulation arguments (world and playback flags)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch global multi-arm RViz workcell visualization synced to Gazebo",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servers",
            default_value="true",
            description="Launch Pick & Place action servers for all 3 robots",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_args = LaunchConfiguration("gz_args")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servers = LaunchConfiguration("launch_servers")

    bringup_dir = get_package_share_directory("multi_arm_bringup")
    stations_file = os.path.join(bringup_dir, "config", "stations.yaml")

    # 2. Full Multi-Robot Workcell Description (URDF / Xacro)
    workcell_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("multi_arm_description"), "urdf", "multi_ur10e_workcell.urdf.xacro"]
            ),
        ]
    )
    workcell_description = {
        "robot_description": ParameterValue(workcell_description_content, value_type=str)
    }

    # 3. Gazebo Harmonic (Gz Sim 8) Simulator Instance
    # In ROS 2 Jazzy, Gazebo Harmonic is launched via the ros_gz_sim package
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # 4. Spawn Multi-Robot Workcell Entity into Gazebo Harmonic
    spawn_workcell = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "multi_ur10e_workcell",
            "-string", workcell_description_content,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )

    # 5. ROS 2 <-> Gazebo Harmonic Bridge (Clock & Sensor Bridges)
    # Bridges the simulation clock and joint status topics between ROS 2 Jazzy and Gazebo Harmonic
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            # Clock Bridge: Gazebo Clock -> ROS 2 Clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Joint States Bridge
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 6. Unified Workcell State Publisher (/tf broadcaster)
    workcell_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[workcell_description, {"use_sim_time": use_sim_time}],
    )

    # 7. MoveIt 2 Instances for each of the 3 UR10e Robots
    robot_groups = []
    for i in range(1, 4):
        ns = f"robot{i}"
        pkg_name = f"robot{i}_moveit_config"
        pkg_dir = get_package_share_directory(pkg_name)

        robot_moveit = GroupAction(
            [
                PushRosNamespace(ns),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_dir, "launch", "ur_moveit.launch.py")
                    ),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "prefix": f"{ns}_",
                        "launch_rviz": "false",
                        "launch_rsp": "false",
                    }.items(),
                ),
            ]
        )
        robot_groups.append(robot_moveit)

    # 8. Pick & Place Action Servers (one per robot, with simulation clock enabled)
    action_servers = []
    for i in range(1, 4):
        ns = f"robot{i}"
        prefix = f"robot{i}_"

        server_node = Node(
            package="multi_arm_control",
            executable="pick_place_server",
            namespace=ns,
            name="pick_place_server",
            output="screen",
            condition=IfCondition(launch_servers),
            parameters=[
                {"stations_file": stations_file},
                {"prefix": prefix},
                {"use_sim_time": use_sim_time},
            ],
        )
        action_servers.append(server_node)

    # 9. Global RViz2 Workcell Display
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("multi_arm_description"), "rviz", "view_workcell.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_workcell",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        parameters=[
            workcell_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Assemble all launch actions
    nodes = (
        declared_arguments
        + [gazebo_sim, spawn_workcell, gz_bridge, workcell_state_publisher]
        + robot_groups
        + action_servers
        + [rviz_node]
    )

    return LaunchDescription(nodes)
