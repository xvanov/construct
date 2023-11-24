#! /usr/bin/env python

### dependencies
import rospy
import time
import datetime
from std_srvs.srv import Trigger, TriggerRequest
import actionlib

# messages
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry

### local dependencies
from my_turtlebot_actions.msg import record_odomAction, record_odomGoal, record_odomResult, record_odomFeedback

### constants
PI = 3.14

### main class
class Main():
    
    def __init__(self):
        ## constants
        self.rate = rospy.Rate(1)
        self.ctrl_c = False

        ## publishers
        self.move = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #Create a Publisher to move the drone
        self.twistMessage = Twist()

        ## subscribers
        # none

        ## service servers
        crash_server_name = '/turtlebot_service_server'
        rospy.wait_for_service(crash_server_name)
        self.crash_request = TriggerRequest()
        self.crash_client = rospy.ServiceProxy(crash_server_name, Trigger)

        ## action servers
        rec_odom_client = actionlib.SimpleActionClient('/rec_odom_as', record_odomAction)
        rospy.loginfo('Waiting for action Server')
        rec_odom_client.wait_for_server()
        rospy.loginfo('Action Server Found...')
        goal = record_odomGoal()

        ## start action servers
        self.start_rec_odom_action(rec_odom_client, goal)

    def start_rec_odom_action(self, client, goal):
        client.send_goal(goal, feedback_cb=self.feedback_callback)
        # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        state_result = client.get_state()
        rospy.loginfo("state_result: "+str(state_result))

        while state_result < 2:
            rospy.loginfo("Waiting to finish: ")
            self.decide_and_move()
            self.rate.sleep()
            state_result = client.get_state()
            rospy.loginfo("state_result: "+str(state_result))
            
        state_result = client.get_state()
        rospy.loginfo("[Result] State: "+str(state_result))
        if state_result == 4:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == 3:
            rospy.logwarn("There is a warning in the Server Side")

        rospy.loginfo("[Result] State: "+str(client.get_result()))
            
    def decide_and_move(self):
        message = self.make_request()
        if message == 'right':
            self.angle_turn_command(-30,10)
        elif message == 'left':
            self.angle_turn_command(30,10)
        else:
            self.line_command(0.5,0.1)

    def feedback_callback(self, feedback):
        # definition of the feedback callback. This will be called when feedback
        # is received from the action server
        # it just prints a message indicating a new message has been received
        rospy.loginfo("Rec Odom Feedback feedback ==>"+str(feedback))

    def count_seconds(self, seconds):
        for i in range(seconds):
            rospy.loginfo("Seconds Passed =>"+str(i))
            time.sleep(1)

    def make_request(self):
        rospy.loginfo("Making a request...")
        response = self.crash_client(self.crash_request)
        rospy.loginfo(str(response))
        return response.message
    
    def stop_command(self):
        rospy.loginfo("Stopping...")
        self.twistMessage.linear.x = 0
        self.twistMessage.linear.y = 0
        self.twistMessage.linear.z = 0
        self.twistMessage.angular.x = 0
        self.twistMessage.angular.y = 0
        self.twistMessage.angular.z = 0
        self.publish_once_in_cmd_vel(self.twistMessage)
    
    def angle_turn_command(self, speed, angle):
        rospy.loginfo("Turning...")
        angular_speed = speed*PI/180
        relative_angle = angle*PI/180 #/2 # divide by 2 because bb8????

        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        rate = rospy.Rate(2)
        while(abs(current_angle) < abs(relative_angle)):
            #breakBool = self.preempt_condition()
            #if breakBool: break
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
            self.twistMessage.angular.z = angular_speed
            rospy.loginfo("Current Angle = {}".format(current_angle*360/(2*PI)))
            self.publish_once_in_cmd_vel(self.twistMessage)
            rate.sleep()
        
        self.stop_command()
    
    def line_command(self, speed, distance):
        rospy.loginfo("Moving in straight line...")
        t0 = rospy.Time.now().to_sec()
        currentDistance = 0
        rate = rospy.Rate(2)
        while currentDistance < distance:
            # TODO: preempt condition
            #breakBool = self.preempt_condition()
            #if breakBool: break
            t1 = rospy.Time.now().to_sec()
            currentDistance = speed*(t1-t0)
            self.twistMessage.linear.x = speed
            rospy.loginfo("Current Distance = {}".format(currentDistance))
            self.publish_once_in_cmd_vel(self.twistMessage)
            rate.sleep()
        self.stop_command()
    
    def publish_once_in_cmd_vel(self, cmd):
        """
        This is because publishing in topics sometimes fails teh first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.move.get_num_connections()
            if connections > 0:
                self.move.publish(cmd)
                rospy.loginfo("Publish in cmd_vel...")
                break
            else:
                self.rate.sleep()
### run 
if __name__ == '__main__':
    rospy.init_node('turtlebot_main_node')
    m = Main()
