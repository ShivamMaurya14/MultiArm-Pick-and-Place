import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Warning: Could not load YAML file '{file_path}' from package '{package_name}': {e}")
        return {}

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="robot3_",
            description="Prefix for joints and links",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with MotionPlanning display",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rsp",
            default_value="true",
            description="Launch robot_state_publisher & joint_state_publisher for standalone mode",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rsp = LaunchConfiguration("launch_rsp")

    # Robot Description
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

    # Semantic Description (SRDF)
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot3_moveit_config"), "srdf", "robot3.srdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # Kinematics config (passed as dict)
    kinematics_yaml = load_yaml("robot3_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Joint limits config (passed as dict)
    joint_limits_yaml = load_yaml("robot3_moveit_config", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # OMPL Planning Pipeline
    ompl_planning_yaml = load_yaml("robot3_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution & Controllers
    moveit_controllers = load_yaml("robot3_moveit_config", "config/moveit_controllers.yaml")
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Planning Scene Monitor
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Standalone Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        condition=IfCondition(launch_rsp),
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Standalone Joint State Publisher Node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(launch_rsp),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Move Group Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz Node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("multi_arm_description"), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": use_sim_time},
        ],
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        run_move_group_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
