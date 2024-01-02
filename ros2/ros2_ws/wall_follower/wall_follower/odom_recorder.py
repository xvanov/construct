import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
from actions_quiz_new.action import Distance
from actions_quiz_new.action import OdomRecord


class OdomRecorder(Node):

    def __init__(self):
        super().__init__('odom_recorder')
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self, OdomRecord, 'record_odom', self.execute_callback, callback_group=self.callback_group)

        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10, callback_group=self.callback_group)

        self.last_odom = Point32()
        self.first_odom = None
        self.odom_record = []
        self.total_distance = 0.0
        self.current_linear_velocity = 0.0
        self.last_x, self.last_y = None, None

    def odom_callback(self, msg):
        # Extract x, y, theta and update last_odom
        self.last_odom.x = msg.pose.pose.position.x
        self.last_odom.y = msg.pose.pose.position.y
        # Assuming theta is the yaself.srv_turnsrv_turnsrv_turnw from quaternion
        quaternion = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.last_odom.z = theta

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = OdomRecord.Feedback()

        self.first_odom = Point32()
        self.first_odom.x, self.first_odom.y, self.first_odom.z = self.last_odom.x, self.last_odom.y, self.last_odom.z
        self.odom_record.append(self.first_odom)

        while True:
            time.sleep(1)
            current_odom = Point32(x=self.last_odom.x, y=self.last_odom.y, z=self.last_odom.z)
            self.odom_record.append(current_odom)

            # Calculate distance for this iteration
            if self.last_x is not None and self.last_y is not None:
                distance_this_iteration = math.sqrt(
                    (self.last_odom.x - self.last_x) ** 2 +
                    (self.last_odom.y - self.last_y) ** 2)
                self.total_distance += distance_this_iteration

            # Update last x, y
            self.last_x, self.last_y = self.last_odom.x, self.last_odom.y

            # Provide feedback
            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)

            # Check if the robot has returned to the starting point
            if math.sqrt(
                (self.last_odom.x - self.first_odom.x) ** 2 +
                (self.last_odom.y - self.first_odom.y) ** 2) < 0.1:
                break

        # Set and return the result
        goal_handle.succeed()
        result = OdomRecord.Result()
        
        result.list_of_odoms = self.odom_record
        return result

def euler_from_quaternion(quaternion):
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
    odom_recorder = OdomRecorder()

    executor = MultiThreadedExecutor()
    rclpy.spin(odom_recorder, executor=executor)

    odom_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
