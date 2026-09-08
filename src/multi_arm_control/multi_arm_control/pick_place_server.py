#!/usr/bin/env python3
import time
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from multi_arm_interfaces.action import PickPlace

class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # Declare & load parameters
        self.declare_parameter('stations_file', '')
        self.declare_parameter('prefix', '')
        
        stations_file = self.get_parameter('stations_file').value
        self.prefix = self.get_parameter('prefix').value
        if not self.prefix:
            ns = self.get_namespace().strip('/')
            self.prefix = f"{ns}_" if ns else ""

        # Exact UR10e 6-DOF Arm Joint Names
        self.arm_joint_names = [
            f"{self.prefix}shoulder_pan_joint",
            f"{self.prefix}shoulder_lift_joint",
            f"{self.prefix}elbow_joint",
            f"{self.prefix}wrist_1_joint",
            f"{self.prefix}wrist_2_joint",
            f"{self.prefix}wrist_3_joint"
        ]
        
        # Exact Robotiq 2F-140 Gripper Joint Names (as defined in robotiq_2f_140.xacro)
        self.gripper_joint_names = [
            f"{self.prefix}finger_joint",
            f"{self.prefix}right_outer_knuckle_joint",
            f"{self.prefix}left_inner_knuckle_joint",
            f"{self.prefix}right_inner_knuckle_joint",
            f"{self.prefix}left_inner_finger_joint",
            f"{self.prefix}right_inner_finger_joint"
        ]

        # Calibrated Standby / Ready Home Pose (facing forward towards the stations at Z=0.80m)
        self.home_joints = [-0.2514, -1.8000, 1.5000, -1.2700, -1.5708, 0.0000]
        self.current_arm_joints = list(self.home_joints)
        
        # Gripper initial state: OPEN (0.0 = fully open 140mm, 0.40 = closed on 60mm cube)
        self.current_gripper_pos = 0.0

        # Dynamic Workpiece Visual Tracking State
        self.cube_attached_to_robot = False
        self.station_world_coords = {
            'A': (0.70, 0.00, 0.245),
            'B': (0.70, 0.80, 0.245),
            'C': (0.70, 2.40, 0.245),
            'D': (0.70, 3.20, 0.245)
        }
        self.current_table_station = 'A' if 'robot1' in self.prefix else None

        # Publishers & Subscribers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_command_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.marker_pub = self.create_publisher(Marker, '/workpiece_marker', 10)
        self.handoff_pub = self.create_publisher(String, '/workpiece_owner', 10)
        self.handoff_sub = self.create_subscription(String, '/workpiece_owner', self._handoff_callback, 10, callback_group=self.cb_group)
        
        # Periodic joint state broadcaster (30 Hz)
        self.state_timer = self.create_timer(1.0 / 30.0, self.publish_current_state, callback_group=self.cb_group)

        # Calibrated analytical joint configurations per robot and station
        self.joint_targets = self._generate_calibrated_targets()

        # Action Server
        self._action_server = ActionServer(
            self,
            PickPlace,
            'pick_place',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info(f"=== PickPlace Action Server READY on '{self.get_namespace()}/pick_place' ===")

    def _handoff_callback(self, msg):
        owner = msg.data.strip()
        if owner != self.prefix:
            self.cube_attached_to_robot = False
            self.current_table_station = None

    def _generate_calibrated_targets(self):
        """
        True URDF Workcell analytical joint configurations:
        - Station A (WS1 Source Table):      X = 0.70, Y = 0.00, Z = 0.25 (TCP facing down [0, 0, -1])
        - Station B (WS2 Relay 1 Buffer):    X = 0.70, Y = 0.80, Z = 0.25 (TCP facing down [0, 0, -1])
        - Station C (WS3 Relay 2 Buffer):    X = 0.70, Y = 2.40, Z = 0.25 (TCP facing down [0, 0, -1])
        - Station D (WS4 Destination Table): X = 0.70, Y = 3.20, Z = 0.25 (TCP facing down [0, 0, -1])
        """
        targets = {}
        
        if "robot1" in self.prefix:
            # Station A (Source at X=0.70, Y=0.00, Z=0.25) -> pan = -14.4 deg
            targets['A'] = {
                'approach': [-0.2514, -1.5023, 1.9461, -2.0146, -1.5708, 0.0000],
                'reach':    [-0.2514, -1.3517, 2.0841, -2.3033, -1.5708, 0.0000]
            }
            # Station B (Relay 1 at X=0.70, Y=0.80, Z=0.25) -> pan = +39.4 deg
            targets['B'] = {
                'approach': [0.6874, -0.9215, 1.1369, -1.7862, -1.5708, 0.0000],
                'reach':    [0.6874, -0.8380, 1.2705, -2.0033, -1.5708, 0.0000]
            }
        elif "robot2" in self.prefix:
            # Station B (Relay 1 at X=0.70, Y=0.80, Z=0.25) -> pan = -58.2 deg
            targets['B'] = {
                'approach': [-1.0165, -0.9215, 1.1369, -1.7862, -1.5708, 0.0000],
                'reach':    [-1.0165, -0.8380, 1.2705, -2.0033, -1.5708, 0.0000]
            }
            # Station C (Relay 2 at X=0.70, Y=2.40, Z=0.25) -> pan = +39.4 deg
            targets['C'] = {
                'approach': [0.6874, -0.9215, 1.1369, -1.7862, -1.5708, 0.0000],
                'reach':    [0.6874, -0.8380, 1.2705, -2.0033, -1.5708, 0.0000]
            }
        elif "robot3" in self.prefix:
            # Station C (Relay 2 at X=0.70, Y=2.40, Z=0.25) -> pan = -58.2 deg
            targets['C'] = {
                'approach': [-1.0165, -0.9215, 1.1369, -1.7862, -1.5708, 0.0000],
                'reach':    [-1.0165, -0.8380, 1.2705, -2.0033, -1.5708, 0.0000]
            }
            # Station D (Destination at X=0.70, Y=3.20, Z=0.25) -> pan = -14.4 deg
            targets['D'] = {
                'approach': [-0.2514, -1.5023, 1.9461, -2.0146, -1.5708, 0.0000],
                'reach':    [-0.2514, -1.3517, 2.0841, -2.3033, -1.5708, 0.0000]
            }

        # Standby Home Pose
        targets['HOME'] = {
            'approach': list(self.home_joints),
            'reach':    list(self.home_joints)
        }
        return targets

    def publish_current_state(self):
        """Broadcasts current arm, Robotiq gripper, and dynamic workpiece marker to RViz."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 6 Arm joints
        msg.name.extend(self.arm_joint_names)
        msg.position.extend(self.current_arm_joints)
        
        # 6 Robotiq 2F-140 Gripper mimic joints
        g = self.current_gripper_pos
        msg.name.extend(self.gripper_joint_names)
        msg.position.extend([g, -g, -g, -g, g, g])
        
        self.joint_state_pub.publish(msg)
        self.joint_command_pub.publish(msg)

        # Broadcast Dynamic Workpiece Marker to RViz
        if self.cube_attached_to_robot or self.current_table_station is not None:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "workpiece"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            if self.cube_attached_to_robot:
                # Attached to robot TCP fingertip grasping center
                marker.header.frame_id = f"{self.prefix}tcp"
                marker.pose.position.x = 0.0
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0
            else:
                # Resting on the Station table top in world frame
                st_coords = self.station_world_coords.get(self.current_table_station, (0.70, 0.00, 0.245))
                marker.header.frame_id = "world"
                marker.pose.position.x = st_coords[0]
                marker.pose.position.y = st_coords[1]
                marker.pose.position.z = st_coords[2]
                marker.pose.orientation.w = 1.0

            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.05

            # Emerald Green
            marker.color.r = 0.10
            marker.color.g = 0.85
            marker.color.b = 0.45
            marker.color.a = 1.0

            self.marker_pub.publish(marker)

    def interpolate_arm_trajectory(self, target_joints, duration_sec=1.5):
        """Smooth shortest-path S-curve trajectory interpolation published at 50 Hz."""
        start_joints = list(self.current_arm_joints)
        steps = max(int(duration_sec * 50), 10)
        dt = duration_sec / steps
        
        # Calculate shortest angular differences for continuous, smooth motion
        diffs = [
            (target_joints[j] - start_joints[j] + math.pi) % (2 * math.pi) - math.pi
            for j in range(len(start_joints))
        ]

        for i in range(1, steps + 1):
            t = i / steps
            # Cosine S-curve easing
            factor = (1.0 - math.cos(t * math.pi)) / 2.0
            
            self.current_arm_joints = [
                start_joints[j] + factor * diffs[j]
                for j in range(len(start_joints))
            ]
            self.publish_current_state()
            time.sleep(dt)

    def interpolate_gripper(self, target_pos, duration_sec=0.8):
        """
        Smoothly animates the Robotiq 2F-140 gripper fingers.
        - target_pos = 0.0  : Fully OPEN (140mm stroke width)
        - target_pos = 0.40 : CLOSED (Grasping 60mm workpiece firmly)
        - target_pos = 0.70 : Fully CLOSED
        """
        start_pos = self.current_gripper_pos
        steps = max(int(duration_sec * 40), 10)
        dt = duration_sec / steps

        for i in range(1, steps + 1):
            t = i / steps
            factor = (1.0 - math.cos(t * math.pi)) / 2.0
            self.current_gripper_pos = start_pos + factor * (target_pos - start_pos)
            self.publish_current_state()
            time.sleep(dt)

    def execute_callback(self, goal_handle):
        """Executes full pick or place action with accurate coordinates and visible gripper open/close."""
        start_time = time.time()
        req = goal_handle.request
        action_name = req.action.lower().strip()
        station_name = req.station_name.upper().strip()

        self.get_logger().info(f"==> EXECUTING {action_name.upper()} AT STATION '{station_name}'")
        self.handoff_pub.publish(String(data=self.prefix))
        
        feedback_msg = PickPlace.Feedback()
        result = PickPlace.Result()

        if station_name not in self.joint_targets:
            station_name = 'HOME'

        targets = self.joint_targets[station_name]
        approach_joints = targets['approach']
        reach_joints = targets['reach']

        # Robotiq 2F-140 has 140mm (0.140m) stroke (0.0 rad = 140mm open, 0.70 rad = 0mm closed)
        # Exact physical grasp angle for workpiece cube (default 0.060m / 60mm):
        cube_width = req.grasp_width if (req.grasp_width and req.grasp_width > 0.0) else 0.06
        grasp_pos = max(0.0, min(0.70, 0.70 * (1.0 - (cube_width / 0.140))))
        self.get_logger().info(f"Workpiece width: {cube_width*1000:.1f}mm -> Gripper grasp joint: {grasp_pos:.3f} rad")

        try:
            # ----------------------------------------------------
            # PICK SEQUENCE
            # ----------------------------------------------------
            if action_name == "pick":
                # Ensure gripper is fully OPEN before approach
                self.interpolate_gripper(target_pos=0.0, duration_sec=0.4)

                # PHASE 1: APPROACH (Move arm directly over Station table at clearance height)
                feedback_msg.current_phase = "APPROACHING"
                feedback_msg.progress_percent = 25.0
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f"Phase 1/4: APPROACHING Station {station_name} with OPEN gripper (140mm)...")
                self.interpolate_arm_trajectory(approach_joints, duration_sec=1.5)

                # PHASE 2: DESCENT (Lower fingers directly around the workpiece)
                feedback_msg.current_phase = "DESCENDING"
                feedback_msg.progress_percent = 50.0
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f"Phase 2/4: DESCENDING over Station {station_name} workpiece (Z=0.25m)...")
                self.interpolate_arm_trajectory(reach_joints, duration_sec=1.0)

                # PHASE 3: GRASP (Close fingers firmly on workpiece & attach workpiece)
                feedback_msg.current_phase = "GRASPING"
                feedback_msg.progress_percent = 75.0
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info("Phase 3/4: CLOSING gripper fingers firmly onto workpiece...")
                self.interpolate_gripper(target_pos=grasp_pos, duration_sec=0.8)
                
                # Attach workpiece to robot TCP in RViz
                self.cube_attached_to_robot = True
                self.current_table_station = None

                # PHASE 4: RETREAT & LIFT (Lift workpiece to clearance height)
                feedback_msg.current_phase = "RETREATING"
                feedback_msg.progress_percent = 90.0
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info("Phase 4/4: RETREATING to clearance height with grasped workpiece...")
                self.interpolate_arm_trajectory(approach_joints, duration_sec=0.9)
                self.interpolate_arm_trajectory(self.home_joints, duration_sec=1.2)

            # ----------------------------------------------------
            # PLACE SEQUENCE
            # ----------------------------------------------------
            else:
                # Arm holds workpiece (gripper remains CLOSED at grasp_pos)
                # PHASE 1: APPROACH (Move arm over destination Station table)
                feedback_msg.current_phase = "APPROACHING"
                feedback_msg.progress_percent = 25.0
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f"Phase 1/4: APPROACHING Station {station_name} holding workpiece...")
                self.interpolate_arm_trajectory(approach_joints, duration_sec=1.5)

                # PHASE 2: DESCENT (Lower workpiece smoothly to table surface)
                feedback_msg.current_phase = "DESCENDING"
                feedback_msg.progress_percent = 50.0
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f"Phase 2/4: DESCENDING onto Station {station_name} table (Z=0.25m)...")
                self.interpolate_arm_trajectory(reach_joints, duration_sec=1.0)

                # PHASE 3: RELEASE (Open fingers to release workpiece & detach)
                feedback_msg.current_phase = "RELEASING"
                feedback_msg.progress_percent = 75.0
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info("Phase 3/4: OPENING gripper fingers to release workpiece...")
                self.interpolate_gripper(target_pos=0.0, duration_sec=0.8)
                
                # Detach from robot and seat on destination Station table in RViz
                self.cube_attached_to_robot = False
                self.current_table_station = station_name

                # PHASE 4: RETREAT (Retract empty arm back to Ready pose)
                feedback_msg.current_phase = "RETREATING"
                feedback_msg.progress_percent = 90.0
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info("Phase 4/4: RETREATING empty arm to Ready pose...")
                self.interpolate_arm_trajectory(approach_joints, duration_sec=0.9)
                self.interpolate_arm_trajectory(self.home_joints, duration_sec=1.2)

            # Succeeded
            elapsed = time.time() - start_time
            goal_handle.succeed()
            result.success = True
            result.message = f"Successfully completed {action_name.upper()} at Station {station_name}"
            result.execution_time = elapsed
            self.get_logger().info(f"==> COMPLETED in {elapsed:.2f}s: {result.message}")
            return result

        except Exception as e:
            self.get_logger().error(f"Error executing goal: {e}")
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            result.execution_time = time.time() - start_time
            return result

def main(args=None):
    rclpy.init(args=args)
    server = PickPlaceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
