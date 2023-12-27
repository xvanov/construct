# Import necessary modules and custom service message
from geometry_msgs.msg import Twist
from services_quiz_srv.srv import Turn  # Use your custom service message package
import rclpy
from rclpy.node import Node

class Service(Node):

    def __init__(self):
        super().__init__('service_quiz_server')
        self.srv_turn = self.create_service(Turn, 'turn', self.turn_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def turn_callback(self, request, response):
        msg = Twist()

        if request.direction.lower() == "left":
            msg.angular.z = abs(request.angular_velocity)
        elif request.direction.lower() == "right":
            msg.angular.z = -abs(request.angular_velocity)
        else:
            self.get_logger().warn(f"Invalid direction: {request.direction}")
            response.success = False
            return response

        # Publish the message to the topic for the specified time
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).to_msg().sec < request.time:
            self.publisher_.publish(msg)

        # Stop the robot after the specified time
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

        self.get_logger().info(f"Spinning {request.direction} at {request.angular_velocity} rad/s for {request.time} seconds.")
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    service = Service()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
