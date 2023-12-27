import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower_srv.srv import FindWall
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.timer import Timer



class WallFinder(Node):

    def __init__(self):
        super().__init__('find_wall_server')
        self.srv_turn = self.create_service(FindWall, 'find_wall', self.find_wall_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_data = None
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.state = 'idle'
        self.timer = self.create_timer(0.1, self.timer_callback) # Adjust the timer period as needed

    def laser_callback(self, msg):
        self.laser_data = msg

    def find_wall_callback(self, request, response):
        self.state = 'rotate_to_wall'
        response.success = True
        return response

    def timer_callback(self):
        if self.state == 'idle':
            return

        if self.state == 'rotate_to_wall':
            self.rotate_to_wall()

        elif self.state == 'move_forward':
            self.move_forward_until_close()

        elif self.state == 'align_with_wall':
            self.align_with_wall()

    def rotate_to_wall(self):
        min_index = self._find_min_index()
        twist = Twist()

        if min_index != 0:
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
        else:
            self.state = 'move_forward'

    def move_forward_until_close(self):
        twist = Twist()

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
        else:
            self.state = 'idle'
            self.get_logger().info(f"Finished")

    def _find_min_index(self):
        # Filter out invalid data and find min index
        valid_ranges = self.laser_data.ranges
        min_index = min(range(len(valid_ranges)), key=valid_ranges.__getitem__)
        self.get_logger().info(f"{min_index}")
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
