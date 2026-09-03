import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from multi_arm_interfaces.action import PickPlace

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # Define action clients for each robot
        self.clients = {
            'robot1': ActionClient(self, PickPlace, '/robot1/pick_place'),
            'robot2': ActionClient(self, PickPlace, '/robot2/pick_place'),
            'robot3': ActionClient(self, PickPlace, '/robot3/pick_place'),
        }
        
        # State machine sequence
        self.sequence = [
            {'robot': 'robot1', 'action': 'pick',  'station': 'A'},
            {'robot': 'robot1', 'action': 'place', 'station': 'B'},
            {'robot': 'robot2', 'action': 'pick',  'station': 'B'},
            {'robot': 'robot2', 'action': 'place', 'station': 'C'},
            {'robot': 'robot3', 'action': 'pick',  'station': 'C'},
            {'robot': 'robot3', 'action': 'place', 'station': 'D'}
        ]
        
        self.current_step = 0
        
        # Wait for all servers
        for robot, client in self.clients.items():
            self.get_logger().info(f'Waiting for {robot} pick/place server...')
            client.wait_for_server()
            self.get_logger().info(f'{robot} server connected.')

        # Start sequence
        self.timer = self.create_timer(1.0, self.start_sequence)
        self.started = False

    def start_sequence(self):
        if not self.started:
            self.started = True
            self.timer.cancel()
            self.execute_step()

    def execute_step(self):
        if self.current_step >= len(self.sequence):
            self.get_logger().info("====================================")
            self.get_logger().info("TASK MANAGER SEQUENCE COMPLETE! A->D")
            self.get_logger().info("====================================")
            rclpy.shutdown()
            return
            
        step = self.sequence[self.current_step]
        self.get_logger().info(f"--- STEP {self.current_step+1}/{len(self.sequence)} ---")
        self.get_logger().info(f"Commanding {step['robot']} to {step['action'].upper()} at station {step['station']}")
        
        client = self.clients[step['robot']]
        
        goal_msg = PickPlace.Goal()
        goal_msg.action = step['action']
        goal_msg.station_name = step['station']
        
        self._send_goal_future = client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def feedback_callback(self, feedback_msg):
        phase = feedback_msg.feedback.current_phase
        self.get_logger().info(f"Feedback: {phase}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server. Aborting sequence.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted, executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Step successful: {result.message}')
            self.current_step += 1
            # Slight delay before next step
            self.create_timer(1.0, self.next_step_timer_callback)
        else:
            self.get_logger().error(f'Step failed: {result.message}. Aborting sequence.')
            rclpy.shutdown()

    def next_step_timer_callback(self):
        self.execute_step()

def main(args=None):
    rclpy.init(args=args)
    task_manager = TaskManager()
    
    try:
        rclpy.spin(task_manager)
    except KeyboardInterrupt:
        pass
    finally:
        task_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
