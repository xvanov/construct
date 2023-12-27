# Import the necessary modules and custom service message
from services_quiz_srv.srv import Turn  # Use your custom service message package
import rclpy
from rclpy.node import Node

class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_quiz_client')
        self.client = self.create_client(Turn, 'turn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /turn is not available. Waiting...')

    def send_request(self):
        request = Turn.Request()
        request.direction = "right"
        request.angular_velocity = 0.2
        request.time = 10

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Service call succeeded.')
        else:
            self.get_logger().error('Service call failed.')

def main(args=None):
    rclpy.init(args=args)
    service_client = ServiceClient()
    service_client.send_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
