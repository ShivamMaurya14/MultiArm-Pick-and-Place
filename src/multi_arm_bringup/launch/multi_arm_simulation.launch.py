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
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true (e.g. for Isaac Sim)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch global multi-arm RViz workcell visualization",
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
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servers = LaunchConfiguration("launch_servers")

    bringup_dir = get_package_share_directory("multi_arm_bringup")
    stations_file = os.path.join(bringup_dir, "config", "stations.yaml")

    # Full Multi-Robot Workcell Description (Single authoritative source of TF and meshes)
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

    # Workcell State Publisher (publishes unified global /tf transforms for all 3 robots and tables)
    workcell_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[workcell_description, {"use_sim_time": use_sim_time}],
    )

    # MoveIt instances for each robot
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

    # Pick & Place Action Servers (one per robot)
    action_servers = []
    for i in range(1, 4):
        ns = f"robot{i}"
        prefix = f"robot{i}_"
        pkg_name = f"robot{i}_moveit_config"

        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("multi_arm_description"), "urdf", "ur10e_robotiq.urdf.xacro"]
                ),
                " ",
                "prefix:=",
                prefix,
                " ",
                "name:=ur10e_robotiq",
                " ",
                "use_fake_hardware:=true",
            ]
        )
        robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

        robot_description_semantic_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(pkg_name), "srdf", f"{ns}.srdf.xacro"]
                ),
                " ",
                "prefix:=",
                prefix,
            ]
        )
        robot_description_semantic = {
            "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
        }

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

    # Global RViz Workcell Display (loads view_workcell.rviz)
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

    nodes = (
        declared_arguments
        + [workcell_state_publisher]
        + robot_groups
        + action_servers
        + [rviz_node]
    )

    return LaunchDescription(nodes)
