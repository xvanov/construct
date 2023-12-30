import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from actions_quiz_msg.action import Distance

class DistanceActionClient(Node):

    def __init__(self):
        super().__init__('distance_action_client')
        self._action_client = ActionClient(self, Distance, 'distance_as')

    def send_goal(self, seconds):
        goal_msg = Distance.Goal()
        goal_msg.seconds = seconds

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: Distance {result.total_dist}, Success: {result.status}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Distance {feedback.current_dist}')

def main(args=None):
    rclpy.init(args=args)
    action_client = DistanceActionClient()

    action_client.send_goal(20)  # Set goal for 20 seconds

    rclpy.spin(action_client)

    action_client.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
