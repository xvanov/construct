import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from actions_quiz_msg.action import Distance
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import time

class MyActionServer(Node):

    def __init__(self):
        super().__init__('my_action_server')
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self, Distance, 'distance_as', self.execute_callback, callback_group=self.callback_group)
        self.cmd = Twist()
        self.distance_publisher = self.create_publisher(Float32, '/total_distance', 10)
        self.total_dist = 0.0
        self.current_linear_velocity = 0.0

        # Subscribe to the odometry topic with the shared callback group
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10, callback_group=self.callback_group)

    def odom_callback(self, msg):
        # Update the current linear velocity based on odometry data
        self.current_linear_velocity = msg.twist.twist.linear.x

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Distance.Feedback()

        time_interval = 1  # Time interval for each loop iteration in seconds
        last_time = time.time()

        for _ in range(1, goal_handle.request.seconds):
            current_time = time.time()
            time_diff = current_time - last_time
            last_time = current_time

            # Calculate distance for this iteration
            #self.get_logger().info(f'Calc = {self.current_linear_velocity}, {time_diff}')
            distance_this_iteration = self.current_linear_velocity * time_diff
            self.total_dist += distance_this_iteration

            # Publish distance as feedback and to the /total_distance topic
            feedback_msg.current_dist = self.total_dist
            goal_handle.publish_feedback(feedback_msg)
            self.distance_publisher.publish(Float32(data=self.total_dist))
            time.sleep(1)

        # Set and return the result
        goal_handle.succeed()
        result = Distance.Result()
        result.total_dist = self.total_dist
        result.status = True
        self.get_logger().info(f'Result: Distance {result.total_dist}, Success: {result.status}')
        return result

def main(args=None):
    rclpy.init(args=args)
    my_action_server = MyActionServer()

    # Use a MultiThreadedExecutor to handle callbacks concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(my_action_server, executor=executor)

    # Shutdown ROS client library for Python
    my_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
