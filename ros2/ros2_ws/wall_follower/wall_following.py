import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower_interfaces.srv import FindWall
from actions_quiz_new.action import OdomRecord


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.wall_found = False
        self.callback_group = ReentrantCallbackGroup()

        self.find_wall_client = self.create_client(
            FindWall, 
            '/find_wall',
            callback_group=self.callback_group)

        self.record_odom_client = ActionClient(self, OdomRecord, 'record_odom', callback_group=self.callback_group)

        self.service_available = False
        self.connect_to_find_wall_service()

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10,
            callback_group=self.callback_group)
        
        self.publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10,
            callback_group=self.callback_group)
        
        self.get_logger().info('Wall follower node has started')

    def connect_to_find_wall_service(self):
        while not self.service_available:
            self.get_logger().info('Waiting for /find_wall service...')
            self.service_available = self.find_wall_client.wait_for_service(timeout_sec=5.0)
            if not self.service_available:
                self.get_logger().warn('Service /find_wall not available, retrying...')

        self.get_logger().info('Connected to /find_wall service')
        self.call_find_wall_service()

        if self.wall_found:
            self.start_odometry_recording()

    def start_odometry_recording(self):
        self.get_logger().info('Starting odometry recording...')
        goal_msg = OdomRecord.Goal()
        # You can set any goal parameters here if needed
        self.record_odom_client.wait_for_server()
        self._send_goal_future = self.record_odom_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Odometry recording goal rejected')
            return

        self.get_logger().info('Odometry recording goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Odometry recording completed: {result.total_distance} meters')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Distance {feedback.current_total} meters')

    def call_find_wall_service(self):
        request = FindWall.Request()
        future = self.find_wall_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Successfully called /find_wall service')
            self.wall_found = future.result().wallfound
        else:
            self.get_logger().error('Service call failed: %r' % (future.exception(),))

    def laser_callback(self, msg):
        if not self.wall_found:
            self.get_logger().info('Waiting for wall to be found')
            return

        left_distance = msg.ranges[270]  # Adjust this index for your robot's laser scanner
        front_distance = msg.ranges[0]
        twist = Twist()

        if front_distance < 0.7:
            twist.linear.x = 0.0  # No forward speed
            twist.angular.z = 1.0  # rotational speed
        elif left_distance > 0.3:
            # Approach the wall a little by adding rotational speed to the robot
            twist.linear.x = 0.2  # Move forward
            twist.angular.z = -0.1  # Rotate away from the wall
        elif left_distance < 0.2:
            # Move away from the wall by adding rotational speed in the opposite direction
            twist.linear.x = 0.2  # Move forward
            twist.angular.z = 0.1  # Rotate toward the wall
        else:
            # Keep the robot moving forward
            twist.linear.x = 0.2  # Maintain a constant speed
            twist.angular.z = 0.0  # No rotational speed

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
