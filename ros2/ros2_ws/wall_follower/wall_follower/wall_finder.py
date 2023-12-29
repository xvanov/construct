import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower_srv.srv import FindWall
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
import threading


class WallFinder(Node):

    def __init__(self):
        super().__init__('find_wall_server')
        self.state = None
        self.timer = None  # Timer for state machine
        self.completion_event = None
        self.callback_group = ReentrantCallbackGroup()
        self.srv_turn = self.create_service(
            FindWall, 'find_wall', self.find_wall_callback, callback_group=self.callback_group)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_data = None
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.callback_group)

    def laser_callback(self, msg):
        self.laser_data = msg


    def find_wall_callback(self, request, response):
        self.get_logger().info(f"find_wall_callback")
        self.state = 'rotate_to_wall'
        self.completion_event = threading.Event()

        # Start a timer to repeatedly call process_state_machine
        self.timer = self.create_timer(0.1, self.process_state_machine)

        self.get_logger().info(f"after timer")

        # Wait for the state machine to complete
        self.completion_event.wait()

        response.wallfound = True
        return response

    def process_state_machine(self):
        if self.state == 'rotate_to_wall':
            self.rotate_to_wall()
        elif self.state == 'move_forward':
            self.move_forward_until_close()
        elif self.state == 'align_with_wall':
            self.align_with_wall()
        elif self.state == 'idle':
            self.timer.cancel()
            self.completion_event.set()

    def rotate_to_wall(self):
        min_index = self._find_min_index()
        twist = Twist()
        self.get_logger().info(f"Rotating towards wall")

        if min_index != 0:
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
        else:
            self.state = 'move_forward'

    def move_forward_until_close(self):
        twist = Twist()
        self.get_logger().info(f"Moving towards wall")

        if self.laser_data.ranges[0] >= 0.3:
            twist.linear.x = 0.1
            self.cmd_vel_pub.publish(twist)
        else:
            self.state = 'align_with_wall'

    def align_with_wall(self):
        min_index = self._find_min_index()
        twist = Twist()

        if min_index != 270:
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"Aligning with wall")
        else:
            self.state = 'idle'
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"Finished")

    def _find_min_index(self):
        # Filter out invalid data and find min index
        valid_ranges = self.laser_data.ranges
        min_index = min(range(len(valid_ranges)), key=valid_ranges.__getitem__)
        #self.get_logger().info(f"min index = {min_index}")
        return min_index

def main(args=None):
    rclpy.init(args=args)
    node = WallFinder()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
