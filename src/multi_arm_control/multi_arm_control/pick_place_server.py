import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from multi_arm_interfaces.action import PickPlace
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped

# MoveIt 2 Python API (Humble)
try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
except ImportError:
    # Dummy mock for when moveit_py is not installed locally
    MoveItPy = None

class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')
        
        # Load stations config
        self.declare_parameter('stations_file', '')
        stations_file = self.get_parameter('stations_file').value
        
        self.stations = {}
        if stations_file:
            try:
                with open(stations_file, 'r') as f:
                    config = yaml.safe_load(f)
                    self.stations = config.get('stations', {})
            except Exception as e:
                self.get_logger().error(f"Failed to load stations config: {e}")

        # MoveIt setup
        if MoveItPy:
            self.moveit = MoveItPy(node_name=self.get_name())
            # We assume the planning group for the arm is something like 'robot1_ur_manipulator'
            # We will grab the first planning group that is not a gripper
            # In a real scenario, this would be passed as a parameter.
        else:
            self.get_logger().warn("moveit_py not found. Running in MOCK mode.")
            self.moveit = None

        self.cb_group = ReentrantCallbackGroup()

        # Action Server
        self._action_server = ActionServer(
            self,
            PickPlace,
            'pick_place',
            self.execute_callback,
            callback_group=self.cb_group
        )

        # Gripper Action Client
        self._gripper_client = ActionClient(
            self, 
            GripperCommand, 
            'robotiq_gripper_controller/gripper_cmd',
            callback_group=self.cb_group
        )

        self.get_logger().info("PickPlace Action Server started.")

    def control_gripper(self, open_gripper=True):
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available!')
            return False

        goal_msg = GripperCommand.Goal()
        # 0.04m for Robotiq 2F-85 fully open, 0.0 for closed
        # For 2F-140, fully open is 0.07m
        goal_msg.command.position = 0.07 if open_gripper else 0.0
        goal_msg.command.max_effort = 50.0

        self.get_logger().info(f"Sending gripper command: {'OPEN' if open_gripper else 'CLOSE'}")
        future = self._gripper_client.send_goal_async(goal_msg)
        # We block here using spin_until_future_complete in a real node, 
        # but since we are in a callback, we just wait on the future if using MultiThreadedExecutor.
        # Simple blocking for demo:
        while not future.done():
            time.sleep(0.1)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)
            
        return True

    def move_to_pose(self, pose):
        if not self.moveit:
            self.get_logger().info(f"[MOCK] Moving to pose: {pose}")
            time.sleep(1.0)
            return True

        # Using MoveItPy
        # Replace 'ur_manipulator' with the actual planning group name if namespace is prefixed
        planning_group = None
        for group in self.moveit.get_planning_groups():
            if 'manipulator' in group:
                planning_group = group
                break
                
        if not planning_group:
            self.get_logger().error("No suitable planning group found.")
            return False

        arm = self.moveit.get_planning_component(planning_group)
        arm.set_goal_state(pose_stamped_msg=pose, pose_link=f"{planning_group.split('_')[0]}_tool0")
        
        plan_result = arm.plan()
        if plan_result:
            self.get_logger().info("Executing trajectory...")
            self.moveit.execute(plan_result.trajectory, controllers=[])
            return True
        else:
            self.get_logger().error("Planning failed.")
            return False

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(f"Received goal: {req.action} at station {req.station_name}")
        
        feedback_msg = PickPlace.Feedback()
        result = PickPlace.Result()

        if req.station_name not in self.stations:
            result.success = False
            result.message = f"Unknown station: {req.station_name}"
            goal_handle.abort()
            return result

        target_coords = self.stations[req.station_name]
        
        # Create PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = target_coords['position']['x']
        pose.pose.position.y = target_coords['position']['y']
        pose.pose.position.z = target_coords['position']['z']
        # Simplified quaternion for roll/pitch/yaw = 3.14, 0, 0 (pointing down)
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        try:
            # 1. APPROACH
            feedback_msg.current_phase = "APPROACHING"
            goal_handle.publish_feedback(feedback_msg)
            
            # Offset Z for approach
            approach_pose = PoseStamped()
            approach_pose.header.frame_id = pose.header.frame_id
            approach_pose.pose.position.x = pose.pose.position.x
            approach_pose.pose.position.y = pose.pose.position.y
            approach_pose.pose.position.z = pose.pose.position.z + 0.1
            approach_pose.pose.orientation = pose.pose.orientation
            
            if not self.move_to_pose(approach_pose):
                raise Exception("Failed approach phase")

            # 2. MOVE TO STATION
            if not self.move_to_pose(pose):
                raise Exception("Failed moving to grasp phase")

            # 3. GRASP / RELEASE
            feedback_msg.current_phase = "GRASPING" if req.action == "pick" else "RELEASING"
            goal_handle.publish_feedback(feedback_msg)
            
            self.control_gripper(open_gripper=(req.action == "place"))

            # 4. RETREAT
            feedback_msg.current_phase = "RETREATING"
            goal_handle.publish_feedback(feedback_msg)
            
            if not self.move_to_pose(approach_pose):
                raise Exception("Failed retreat phase")

            goal_handle.succeed()
            result.success = True
            result.message = "Successfully completed " + req.action
            return result

        except Exception as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            return result

def main(args=None):
    rclpy.init(args=args)
    
    server = PickPlaceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
