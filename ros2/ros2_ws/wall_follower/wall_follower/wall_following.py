import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Wall follower node has started')

    def laser_callback(self, msg):
        left_distance = msg.ranges[270]  # Adjust this index for your robot's laser scanner
        front_distance = msg.ranges[0]

        twist = Twist()

        if front_distance < 0.7:
            twist.linear.x = 0.0  # No forward speed
            twist.angular.z = -1.0 # rotational speed
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
