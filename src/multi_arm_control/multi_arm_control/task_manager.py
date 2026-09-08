#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from multi_arm_interfaces.action import PickPlace

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # Define action clients for each robot
        self.action_clients = {
            'robot1': ActionClient(self, PickPlace, '/robot1/pick_place'),
            'robot2': ActionClient(self, PickPlace, '/robot2/pick_place'),
            'robot3': ActionClient(self, PickPlace, '/robot3/pick_place'),
        }
        
        # State machine sequence: A -> B -> C -> D
        self.sequence = [
            {'robot': 'robot1', 'action': 'pick',  'station': 'A'},
            {'robot': 'robot1', 'action': 'place', 'station': 'B'},
            {'robot': 'robot2', 'action': 'pick',  'station': 'B'},
            {'robot': 'robot2', 'action': 'place', 'station': 'C'},
            {'robot': 'robot3', 'action': 'pick',  'station': 'C'},
            {'robot': 'robot3', 'action': 'place', 'station': 'D'}
        ]
        
        self.current_step = 0
        self._step_timer = None
        
        # Wait for all servers to become available
        self.get_logger().info("Connecting to robot action servers...")
        for robot, client in self.action_clients.items():
            self.get_logger().info(f"Waiting for {robot} pick/place server on '/{robot}/pick_place'...")
            client.wait_for_server()
            self.get_logger().info(f"[OK] {robot} server connected.")

        # Start sequence after 1s delay
        self.start_timer = self.create_timer(1.0, self.start_sequence)
        self.started = False

    def start_sequence(self):
        if not self.started:
            self.started = True
            self.start_timer.cancel()
            self.get_logger().info("==============================================")
            self.get_logger().info("STARTING MULTI-ARM RELAY SEQUENCE (A -> B -> C -> D)")
            self.get_logger().info("==============================================")
            self.execute_step()

    def execute_step(self):
        if self.current_step >= len(self.sequence):
            self.get_logger().info("==============================================")
            self.get_logger().info("🎉 TASK MANAGER COMPLETE! RELAY SUCCESSFUL (A -> D)")
            self.get_logger().info("==============================================")
            rclpy.shutdown()
            return
            
        step = self.sequence[self.current_step]
        self.get_logger().info(f"\n--- STEP {self.current_step+1}/{len(self.sequence)} ---")
        self.get_logger().info(f"Commanding {step['robot'].upper()} to {step['action'].upper()} at Station '{step['station']}'")
        
        client = self.action_clients[step['robot']]
        
        goal_msg = PickPlace.Goal()
        goal_msg.action = step['action']
        goal_msg.station_name = step['station']
        goal_msg.grasp_width = 0.06
        
        self._send_goal_future = client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def feedback_callback(self, feedback_msg):
        phase = feedback_msg.feedback.current_phase
        progress = feedback_msg.feedback.progress_percent
        self.get_logger().info(f"  [Progress {progress:.0f}%] Phase: {phase}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server. Aborting sequence.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted by server, executing motion...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"[SUCCESS] Step {self.current_step+1} completed in {result.execution_time:.2f}s: {result.message}")
            self.current_step += 1
            # Delay 1.0s before next step
            self._step_timer = self.create_timer(1.0, self.next_step_timer_callback)
        else:
            self.get_logger().error(f"[FAILED] Step failed: {result.message}. Aborting sequence.")
            rclpy.shutdown()

    def next_step_timer_callback(self):
        if self._step_timer:
            self._step_timer.cancel()
            self._step_timer = None
        self.execute_step()

def main(args=None):
    rclpy.init(args=args)
    task_manager = TaskManager()
    try:
        rclpy.spin(task_manager)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        task_manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
