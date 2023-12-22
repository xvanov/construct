import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class TopicQuiz(Node):

    def __init__(self):
        super().__init__('topics_quiz')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # Variables to keep track of the state
        self.current_yaw = 0.0
        self.start_yaw = None
        self.is_rotating = False
        self.obstacle_to_the_left = True

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.motion)

    def odom_callback(self, msg):
        # Extract the orientation quaternion from the odometry message
        orientation_q = msg.pose.pose.orientation
        # Convert the quaternion to Euler angles
        _, _, self.current_yaw = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        # Assuming the laser provides a 360-degree view around the robot
        # For a 360-degree scanner with measurements in degrees:
        # - 0 degrees is directly ahead
        # - 90 degrees could be directly to the left
        # - 180 degrees directly behind
        # - 270 degrees directly to the right
        # These values may need to be adjusted depending on the laser scanner's configuration
        
        print(msg.ranges)
        num_readings = len(msg.ranges)
        left_start_index = 89  # This might be the left-most reading
        left_end_index = 91  # This might be the right-most reading on the left side

        left_ranges = msg.ranges[left_start_index:left_end_index]
        print(left_ranges)
        # Define the minimum safe distance to be considered an obstacle
        min_safe_distance = 99  # Meters, this should be adjusted to your robot's dimension

        # Check if there is any obstacle within the minimum safe distance on the left side
        self.obstacle_to_the_left = any(distance < min_safe_distance for distance in left_ranges)

    def motion(self):
        if self.is_rotating:
            # Calculate the difference between the current yaw and the starting yaw
            yaw_diff = self.normalize_angle(self.current_yaw - self.start_yaw)
            safety_turn_buffer = 0.1
            # Stop rotating if we've reached 90 degrees
            if abs(yaw_diff) >= np.pi / 2 - safety_turn_buffer:  # 90 degrees
                self.is_rotating = False
                cmd = Twist()
                self.publisher_.publish(cmd)
                self.get_logger().info('Rotation complete')
            else:
                # Continue rotating
                cmd = Twist()
                cmd.angular.z = 0.5  # Adjust angular velocity if necessary
                self.publisher_.publish(cmd)
        else:
            if not self.obstacle_to_the_left and self.start_yaw is None:
                # No obstacle to the left and we haven't started rotating yet
                self.is_rotating = True
                self.start_yaw = self.current_yaw  # Record the starting yaw
            elif not self.is_rotating:
                # Go straight
                self.is_rotating = False
                cmd = Twist()
                cmd.linear.x = 0.5
                self.publisher_.publish(cmd)

    def normalize_angle(self, angle):
        # Normalize the angle to the range [-pi, pi]
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def euler_from_quaternion(self, quaternion):
            """
            Converts quaternion (w in last place) to euler roll, pitch, yaw
            quaternion = [x, y, z, w]
            Below should be replaced when porting for ROS2 Python tf_conversions is done.
            """
            x = quaternion[0]
            y = quaternion[1]
            z = quaternion[2]
            w = quaternion[3]

            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (w * y - z * x)
            pitch = np.arcsin(sinp)

            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    tq = TopicQuiz()
    rclpy.spin(tq)
    tq.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()