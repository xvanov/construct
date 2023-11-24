#! /usr/bin/env python
### summary
'''
'''

### external dependencies
import rospy
import time
import actionlib
## msgs
from template_actions.msg import templateGoal, templateFeedback, templateResult, templateAction


### local dependenies

### main class
class TemplateActionClient():
    
    def __init__(self):
        # constants
        rate = rospy.Rate(1)

        # action client
        client = actionlib.SimpleActionClient('/template_as', templateAction)
        rospy.loginfo('waiting for action server')
        client.wait_for_server()
        rospy.loginfo('action server found...')
        goal = record_odomGoal()
        client.send_goal(goal, feedback_cb=self.feedback_callback)

        # 1 = active, 2 = no error, 3 = warning, 4 = error
        state_result = client.get_state()

        rospy.loginfo("state_result: "+str(state_result))

        while state_result < 2:
            rospy.loginfo("waiting to finish: ")
            rate.sleep()
            state_result = client.get_state()
            rospy.loginfo("state_result: "+str(state_result))
            

        state_result = client.get_state()
        rospy.loginfo("[result] state: "+str(state_result))
        if state_result == 4:
            rospy.logerr("something went wrong in the server side")
        if state_result == 3:
            rospy.logwarn("there is a warning in the server side")
            
    
    def feedback_callback(self, feedback):
        # definition of the feedback callback. This will be called when feedback
        # is received from the action server
        # it just prints a message indicating a new message has been received
        rospy.loginfo("template feedback \n"+str(feedback))


### supporting classes

### execute
if __name__ == '__main__':
    # initializes the action client node
    rospy.init_node('template_action_client_node')
    tac = TemplateActionClient()

### appendix

