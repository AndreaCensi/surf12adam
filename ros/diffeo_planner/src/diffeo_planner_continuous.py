#!/usr/bin/env python
'''
Created on Jan 18, 2013


@author: adam
'''

from PIL import Image #@UnresolvedImport
from diffeo_planner import DiffeoPlanner
from diffeoplan.configuration import set_current_config
from diffeoplan.configuration.master import DiffeoplanConfigMaster
from diffeoplan.library.images.uncertain_image import UncertainImage
from diffeoplan.library.logs.rosbag.bag_reader import get_image_array
import numpy as np
import rospy
import sensor_msgs.msg
import array_msgs.msg
import camera_actuator.srv
import pdb
import yaml
import roslib
from optparse import OptionParser

STATE_RUNNING = 'STATE_RUNNING'
STATE_IDLE = 'STATE_IDLE'

class DiffeoPlannerStandalone(DiffeoPlanner):
    def __init__(self):
        opts = OptionParser()
        opts.add_option('-s', '--system', default='orbit')
        options, args = opts.parse_args()
        
        config_file = (roslib.packages.get_pkg_dir('diffeo_planner') + 
                       '/configs/' + options.system + '.yaml')
        system = yaml.load(open(config_file))
        self.system = system
        
        self.node_name = 'diffeoplan_standalone_continuous'
        rospy.init_node(self.node_name)

        self.y_goal = None
        self.last_image = None
        
        # Initiate goal image service
        s = rospy.Service(self.node_name + '/set_goal',
                          camera_actuator.srv.voidService,
                          self.set_goal)
        
        # Subscribe to /usb_cam/image_raw
        rospy.Subscriber('/usb_cam/image_raw', sensor_msgs.msg.Image, self.incoming_image)
        
        cam_name = '/' + system['name']
        self.cmd_publisher = rospy.Publisher(system['ROS_msg_topic'], array_msgs.msg.IntArray)

        self.stop_command = system['stop_command']
        
        num_cmd = system['num_actuators']
        if not hasattr(system, 'command_list'):
            self.command_list = np.zeros((2 * num_cmd, num_cmd))
            for i in range(num_cmd):
                self.command_list[2 * i, i] = system['min_actuator_values'][i]
                self.command_list[2 * i + 1, i] = system['max_actuator_values'][i]  
            
            self.command_list = self.command_list.tolist()
        else:
            self.command_list = system['command_list']
#        pdb.set_trace()
        
        self.set_default_params()
        self.init_plan()
        print('Node started')
        r = rospy.Rate(system['frequency'])
        
        
        print('Wait for first camera image')
        while not (rospy.is_shutdown() or not self.last_image is None):
            r.sleep()
        
        if not self.last_image is None:
            self.y_goal = self.last_image
            print('Image received')
            
        self.state = STATE_IDLE
        
        ti = 0
        while not rospy.is_shutdown():
            info_str = 'Timesample %g: \t' % ti + ' in state ' + self.state
            ### run planning
            # Run planning
            result = self.algo.plan(get_uncertain_image(self.last_image, self.size),
                                    get_uncertain_image(self.y_goal, self.size))
            plan = result.plan
            info_str += 'plan found: ' + str(plan) + '\t' + result.status
            
            ### in state running
            if self.state == STATE_RUNNING:
                if len(plan) <= 1:
                    # Send zero command
                    next_state = STATE_IDLE
                    send_index = -1

                if len(plan) > 1:
                    # Send command plan[1] to robot
                    next_state = STATE_RUNNING
                    send_index = plan[1]
            
            ### in state idle    
            if self.state == STATE_IDLE:
                if len(plan) == 0:
                    next_state = STATE_IDLE
                    send_index = -1

                if len(plan) > 0:
                    # Send command plan[0] to robot
                    next_state = STATE_RUNNING
                    send_index = plan[0]

            
            ### Prepare for next sample
            print(info_str)
            ti += 1
            self.state = next_state
            r.sleep()
            self.send_command(send_index)
        
        ### TODO: Send stop to robot
        self.send_command(-1)
        print('Shutting down')
        
    def send_command(self, index):
        '''
        Send the command with the given index to logitech orbit
        :param index:
        '''
        
        if index in range(len(self.command_list)):
            command = self.command_list[index]
        else:
            command = self.stop_command
            
        print('Sending command:     ' + str(command))
        msg = camera_actuator.msg.IntArray() #@UndefinedVariable
        msg.data = command
        self.cmd_publisher.publish(msg)
        
    def incoming_image(self, msg):
        '''
        callback function for incoming images on ROS topic
        :param msg:
        '''
        self.last_image = msg
        
    def set_goal(self, msg=None):
        self.y_goal = self.last_image
        return 0
        
    def set_default_params(self):
        rospy.set_param(self.node_name + '/config_path',
                        'default:/home/adam/diffeo-data/')
        
        self.size = [90, 60]
        rospy.set_param(self.node_name + '/id_discdds',
                        'orbit-pt256-160-n35s')
        
        rospy.set_param(self.node_name + '/id_algo',
                        'samp_t1_l5')
        
    def init_plan(self):
        '''
        load parameters from ROS parameter server and activate a online planner 
        from the given parameters.
        '''
        print('init_plan...')
        config_path = rospy.get_param(self.node_name + '/config_path')
        
        config = DiffeoplanConfigMaster()
        config.load(config_path)
        set_current_config(config)
        print('...')
        
        id_discdds = rospy.get_param(self.node_name + '/id_discdds')
        id_algo = rospy.get_param(self.node_name + '/id_algo')
        print('...')
#        pdb.set_trace()
        self.discdds = config.discdds.instance(id_discdds)
        self.algo = config.algos.instance(id_algo)
        self.algo.plan_time = 0.5 / self.system['frequency']
        self.algo.init(id_discdds, self.discdds)
        print('...') 
        
        size = self.discdds.actions[0].diffeo.get_shape()
        self.size = (size[1], size[0]) 
        print('planner initiated')

def get_uncertain_image(image, size):
    image = get_image_array(image)
    image_resized = np.array(Image.fromarray(image).resize(size))
    return UncertainImage(image_resized)

    
if __name__ == '__main__':
    DiffeoPlannerStandalone()
