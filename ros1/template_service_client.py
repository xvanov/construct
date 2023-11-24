#! /usr/bin/env python
### summary
'''
'''

### external dependencies
import rospy
from std_srvs.srv import Trigger, TriggerRequest

### local dependencies

### constants

### main class

def make_request():
    rospy.loginfo("Making a request...")
    response = client(request)
    rospy.loginfo(str(response))

### run
if __name__ == '__main__':
    server_name = '/turtlebot_service_server'
    node_name = 'turtlebot_service_client'
    
    rospy.init_node(node_name)
    rospy.wait_for_service(server_name)
    request = TriggerRequest()
    client = rospy.ServiceProxy(server_name, Trigger)
    
    rate = rospy.Rate(0.5)
    ctrl_c = False
    def shutdownhook():
        global ctrl_c
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        make_request()
        rate.sleep()
