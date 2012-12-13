#!/usr/bin/env python
from PIL import Image #@UnresolvedImport
from collections import namedtuple
from diffeo_planner import DiffeoPlanner
from diffeoplan.configuration import set_current_config
from diffeoplan.configuration.master import DiffeoplanConfigMaster
from diffeoplan.library.images.uncertain_image import UncertainImage
from diffeoplan.library.logs.rosbag.bag_reader import get_image_array
from diffeoplan.library.online.orbit_module import OrbitModule
import camera_actuator.srv
import numpy as np
import pickle
import rospy
import sensor_msgs.msg
import sys
import pdb

class DiffeoPlannerStandalone(DiffeoPlanner):
    def __init__(self):
        self.node_name = 'diffeoplan_standalone'
        rospy.init_node(self.node_name)
        self.y_last = None
        self.y_current = None
        self.y_goal = None
        
        # Subscribe to /usb_cam/image_raw
        rospy.Subscriber('/usb_cam/image_raw', sensor_msgs.msg.Image, self.incoming_image)
        
        # Publisher for distance
        self.distance_pub = rospy.Publisher(self.node_name + '/distance_graph', sensor_msgs.msg.Image)
 
        # Initiate service to execute plan
        self.executePlan = rospy.ServiceProxy('/logitech_cam/executePlan',
                                           camera_actuator.srv.planCommand)
        
        def line_match(strA, strB):
            if strA[:len(strB)] == strB:
                return True
            else:
                return False
            
#        pylab.figure()
        self.set_default_params()
        self.init_plan()
        print('Node started')
        
        while not rospy.is_shutdown():
            line = sys.stdin.readline().strip('\n')
            args = line.split(' ')
            handled = False
            successful = False

            if args[0] == 'run':
                # Register handling of command
                handled = True
                if len(args) > 1:
                    n = int(args[1])
                else:
                    n = 10
                run_result = self.run_demo(n)
                pickle.dump(run_result, open('last_result.pickle', 'wb'))
                
                # Register command handling successful
                successful = True

            
            if args[0] == 'goal':
                # Register handling of command
                handled = True
                
                self.set_goal()
                
                # Register command handling successful
                successful = True
                
            if args[0] == 'start':
                # Register handling of command
                handled = True
                
                self.set_current()
                
                # Register command handling successful
                successful = True
                
            
            # Take care of unknown or failed commands
            if not handled:
                print('Unknown command: ' + line)
            if not successful:
                print('Failed to execute command')
                
    def set_current(self):
        self.y_current = self.last_image
        
    def set_goal(self):
        self.y_goal = self.last_image
        
    def set_default_params(self):
        rospy.set_param(self.node_name + '/config_path',
                        'default:/home/adam/diffeo-data/')
        
        rospy.set_param(self.node_name + '/id_discdds',
                        'orbit-pt256-160-n35s')
        
        rospy.set_param(self.node_name + '/id_algo',
                        'samp_t1_l5')
        
    def run_demo(self, n):
        iter_results = []
        for i in range(n):
            print('run_one ' + str(i))
#            pdb.set_trace()
            y_0 = get_image_array(self.y_current)
            
            plan, plan_executed = self.run_one()
            if plan == (-1,):
                break
            
            self.set_current()
            
            iter_result = {'i':i,
                           'plan_found':tuple(plan),
                           'plan_executed':plan_executed,
                           'y_0':y_0,
                           'y_1':get_image_array(self.y_current)}
            iter_results.append(iter_result)
            
        run_result = {'iter_results':iter_results}
        return run_result
    
    def get_plan(self):
        '''
        Calculate the plan from current image to the goal. 
        '''
        print('Received a request for plan to goal')
        
        
        if self.y_current is None or self.y_goal is None:
            print('Unable to plan because missing images.')
            return [-1]
        
        
        # Plan
        instance = namedtuple('instance', ['y_stary', 'y_goal'])
        
        size = self.size
        instance.y_start = get_uncertain_image(self.y_current, size)
        instance.y_goal = get_uncertain_image(self.y_goal, size)
        
        # Run planning
        result = self.algo.plan(instance.y_start, instance.y_goal)
        plan = result.plan
        
        print('Planning done, found plan: ' + str(plan))
        
        # Fix a bug temporarily TODO: Relearn diffeos
        orbit = OrbitModule(size)
        
        return orbit.inverse_plan(plan)
        
    def init_plan(self):
        '''
        load parameters from ROS parameter server and activate a online planner 
        from the given parameters.
        '''
        print('init_plan')
        config_path = rospy.get_param(self.node_name + '/config_path')
        
        config = DiffeoplanConfigMaster()
        config.load(config_path)
        set_current_config(config)
        
        id_discdds = rospy.get_param(self.node_name + '/id_discdds')
        id_algo = rospy.get_param(self.node_name + '/id_algo')
        
        self.discdds = config.discdds.instance(id_discdds)
        self.algo = config.algos.instance(id_algo)
        self.algo.init(id_discdds, self.discdds) 
        
        size = self.discdds.actions[0].diffeo.get_shape()
        self.size = (size[1], size[0]) 
                
    def run_one(self):
        try:
            # Update current image
            self.set_current()
            
            plan = self.get_plan()
            print(str(plan))
            executed = ()
            # Execute the first step in the plan
            if len(plan) > 0:
                self.executePlan([plan[0]])
                executed = (plan[0],)
            
            return plan, executed
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

def get_uncertain_image(image, size):
    image = get_image_array(image)
    image_resized = np.array(Image.fromarray(image).resize(size))
    return UncertainImage(image_resized)

    
if __name__ == '__main__':
    DiffeoPlannerStandalone()
