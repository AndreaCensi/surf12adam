'''
Created on Oct 16, 2012

@author: adam
'''

from PIL import Image #@UnresolvedImport
from diffeoplan.library.images.uncertain_image import UncertainImage
from diffeoplan.library.logs.rosbag.bag_reader import get_image_array
import camera_actuator.srv
import numpy as np
import rospy

# Define the channels to use for connection to orbit camera.
IMAGE_SERVICE = '/logitech_cam/take_image'
PLAN_EXECUTE_SERVICE = '/logitech_cam/executePlan'

class OrbitModule():
    def __init__(self, size):
        self.cmdlist = [[0, -256, 0], [-256, 0, 0], [0, 256, 0], [256, 0, 0]]
        self.size = size
    
    def execute_plan(self, plan):
        '''
        Sends the plan to ROS service and waits for the execution to complete
        
        :param plan: as int[]
        '''
        # Connection to ROS service
        command = rospy.ServiceProxy(PLAN_EXECUTE_SERVICE, camera_actuator.srv.planCommand)
        
        # Send the plan to ROS module
        command(plan)
            
    def home(self):
        '''
        Sends the plan to ROS service and waits for the execution to complete
        
        :param plan: as int[]
        '''
        # Connection to ROS service
        command = rospy.ServiceProxy(PLAN_EXECUTE_SERVICE, camera_actuator.srv.voidService)
        
        # Send the plan to ROS module
        command(0)
    
    def set_command_list(self):
        pass
    
    def get_image(self):
        '''
        Requests an image from the ROS service specified by IMAGE_SERVICE. 
        Resize to desired size and returns as an UncertainImage
        
        :param size:    size of the image
        
        :return image_resized: as an UncertainImage
        '''
        # Connection to ROS service
        takeImage = rospy.ServiceProxy(IMAGE_SERVICE, camera_actuator.srv.imageService)
        
        # Get image and convert to numpy
        image = get_image_array(takeImage(0).image)
        
        size = self.size
        
        # Convert to PIL, resize and back to numpy
        image_resized = np.array(Image.fromarray(image).resize((size[1], size[0])))
        
        # Return the resized image as UncertainImage
        return UncertainImage(image_resized)
    
    def inverse_plan(self, plan):
        # Calculate the inverse plan
        cmdlist = self.cmdlist
        plan_inverse = ()
        for c in plan:
            cmd = cmdlist[c]
            for i in range(len(cmdlist)):
#                pdb.set_trace()
                if all(cmd == -np.array(cmdlist[i])):
                    plan_inverse += (i,)
                    
        assert(len(plan) == len(plan_inverse))
        return plan_inverse
