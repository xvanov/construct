import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time

class TopicQuiz(Node):

    def __init__(self):
        super().__init__('topics_quiz_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # Variables to keep track of the state
        self.current_yaw = 0.0
        self.start_yaw = None
        self.is_moving_forward1 = False
        self.is_moving_forward2 = False
        self.is_turning = False
        self.forward_distance = 3.5  # Move forward for 3.5 meters
        self.turn_distance = np.pi / 2  # 90-degree turn in radians
        self.forward_distance_after_turn = 5.0  # Move forward for 5 meters
        self.distance_moved = 0.0
        self.start_time = None
        self.linear_velocity = 0.5  # Adjust linear velocity if necessary

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.motion)

    def odom_callback(self, msg):
        # Extract the orientation quaternion from the odometry message
        orientation_q = msg.pose.pose.orientation
        # Convert the quaternion to Euler angles
        _, _, self.current_yaw = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def motion(self):
        if not self.is_moving_forward1 and not self.is_turning:
            if self.start_yaw is None:
                # Record the starting yaw
                self.start_yaw = self.current_yaw
                self.start_time = time.time()
            else:
                # Calculate the time elapsed since starting to move forward
                elapsed_time = time.time() - self.start_time
                # Calculate the distance moved based on time and linear velocity
                delta_distance = self.linear_velocity * elapsed_time

                if delta_distance >= self.forward_distance:
                    # Stop moving forward when the desired distance is reached
                    self.is_moving_forward1 = True
                    cmd = Twist()
                    self.publisher_.publish(cmd)
                    self.get_logger().info('Forward motion complete')
                    # Start turning
                    self.is_turning = True
                    self.start_yaw = self.current_yaw  # Reset the starting yaw for turning
                else:
                    # Continue moving forward
                    cmd = Twist()
                    cmd.linear.x = 0.5
                    cmd.linear.x = self.linear_velocity
                    self.publisher_.publish(cmd)
                    self.distance_moved += delta_distance
        elif self.is_turning:
            # Calculate the difference between the current yaw and the starting yaw for turning
            yaw_diff = self.normalize_angle(self.current_yaw - self.start_yaw)
            safety_turn_buffer = 0.1
            # Stop turning when we've reached the desired turn angle
            if abs(yaw_diff) >= self.turn_distance - safety_turn_buffer:
                self.is_turning = False
                cmd = Twist()
                self.publisher_.publish(cmd)
                self.get_logger().info('Turn complete')
                # Start moving forward again
                self.is_moving_forward2 = True
                self.start_time = time.time()  # Reset the start time for forward motion
            else:
                # Continue rotating
                cmd = Twist()
                cmd.angular.z = 0.5  # Adjust angular velocity if necessary
                self.publisher_.publish(cmd)
        elif self.is_moving_forward2:
            # Calculate the time elapsed since starting to move forward again
            elapsed_time = time.time() - self.start_time
            # Calculate the distance moved based on time and linear velocity
            delta_distance = self.linear_velocity * elapsed_time

            if delta_distance >= self.forward_distance_after_turn:
                # Stop moving forward when the desired distance is reached
                cmd = Twist()
                self.publisher_.publish(cmd)
                self.get_logger().info('Final forward motion complete')
            else:
                # Continue moving forward
                cmd = Twist()
                cmd.linear.x = 0.5
                cmd.linear.x = self.linear_velocity
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