#! /usr/bin/env python
### summary
'''
Explain what I am all about.
'''

### external dependencies
## pylibs
import rospy
## srvs
from std_srvs.srv import Trigger, TriggerResponse
## msgs

### local dependencies
## pylibs
## srvs
## msgs

### constants

### main class
class TemplateServiceServer():
    def __init__(self):
        ## constants
        ## subscribers
        ## publishers
        ## service
        self.templateServiceService = rospy.Service("/template_service_server", Trigger, self.template_service_callback)
        self.response = TriggerResponse()
                    
    def template_service_callback(self, request):
        self.response.success = True
        self.response.message = 'what I do'
        return self.response

### supporting classes

### execute
if __name__ == '__main__':
    
    rospy.init_node('template_service_server_node', log_level = rospy.INFO)
    tss = TemplateServiceServer()
    rospy.spin()

### appendix
