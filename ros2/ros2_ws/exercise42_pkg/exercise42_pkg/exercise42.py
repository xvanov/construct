# import the SetBool module from std_srvs Service interface
from std_srvs.srv import SetBool
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        super().__init__('service_moving')
        self.srv = self.create_service(SetBool, 'moving', self.set_bool_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist() 

    def set_bool_callback(self, request, response):
        # The callback function receives the self-class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as a response        
            
        # Publish the message to the topic
        # As you see, the name of the request parameter is data, so do it
        if request.data == True:
            
            # define the linear x-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.linear.x = 0.3
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.angular.z =-0.3
            
            self.publisher_.publish(self.cmd)
            # You need a response
            response.success = True
            # You need another response, but this time, SetBool lets you put a String
            response.message = 'MOVING TO THE RIGHT RIGHT RIGHT!'

        if request.data == False:

            self.cmd.linear.x = 0.0
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.angular.z =0.0
            
            self.publisher_.publish(self.cmd)
            response.success = False

            response.message = 'It is time to stop!'       
                
        # return the response parameters
        return response


def main(args=None):
    rclpy.init(args=args)
    moving_service = Service()
    rclpy.spin(moving_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
