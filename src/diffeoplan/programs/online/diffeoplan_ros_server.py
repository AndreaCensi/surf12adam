'''
Created on Oct 24, 2012

@author: adam
'''
from PIL import Image  # @UnresolvedImport
from collections import namedtuple
from diffeoplan.configuration import set_current_config
from diffeoplan.configuration.master import DiffeoplanConfigMaster
from diffeoplan.library.logs.rosbag.bag_reader import get_image_array
from diffeoplan.library.online.online_planning import OnlinePlanning
from std_msgs.msg import String

import camera_actuator.srv
import numpy as np
import pdb
import rospy
import sensor_msgs.msg
from diffeoplan.library.images.uncertain_image import UncertainImage
from diffeoplan.library.online import online_planning
from diffeoplan.library.online.orbit_module import OrbitModule

STARTING = 'STARTING'
IDLE = 'IDLE'
PLANNING = 'PLANNING'
WAIT_GOAL = 'WAIT_GOAL'
WAIT_CURRENT = 'WAIT_CURRENT'

class DiffeoplanRosServer():
    '''
    DiffeoplanRosServer creates a ROS node which provides diffeoplan 
    features for other ROS nodes. 
    '''
    
    def set_goal(self, msg):
        '''
        Update the image considered as the current image. 
        Called by incoming ROS msg.
        '''
        self.info('Received new goal image (y_goal)')
#        image = get_image_array(msg)
        
        self.y_goal = msg
        
        # set state to wait for current
#        if self.state in [WAIT_GOAL, WAIT_CURRENT]:
        self.state = WAIT_CURRENT
        if hasattr(self, 'y_current'):
            self.state = IDLE
    
    def set_current(self, msg):
        '''
        Update the image considered as the current image. 
        Called by incoming ROS msg.
        '''
        self.info('Received new start image (y_current)')
#        image = get_image_array(msg)
        
        self.y_current = msg
        
        # set state to wait for goal
        if hasattr(self, 'y_goal'):
            self.state = IDLE 
    
    def get_goal_service(self, req):
        res = camera_actuator.srv.imageServiceResponse()
        res.image = self.y_goal
        return res
    
    def get_current_service(self, req):
        res = camera_actuator.srv.imageServiceResponse()
        res.image = self.y_current
        return res
    
    def get_distance(self, req):
        '''
        Uses the active planner and algorithm to calculate the distance from 
        the current image to the goal image. 
        :param req: dummy number
        '''
        distance = self.online_planner.algo.metric_goal.distance
        
        res = camera_actuator.srv.floatServiceResponse()  # @UndefinedVariable
        res.res = distance(self.get_y_current(self.size),
                           self.get_y_goal(self.size))
        return res
    
    def get_predicted_distance(self, req):
        '''
        :param req:
        '''
        distance = self.online_planner.algo.metric_goal.distance
        res = camera_actuator.srv.floatArrayServiceResponse()  # @UndefinedVariable
        y_0 = self.get_y_current(self.size)
        res.array = []
        
        for i in range(len(req.array)):
            plan = tuple(np.array(req.array[:i + 1]).astype('int'))
            y_p = self.online_planner.discdds.predict(y_0, plan)
            res.array.append(distance(y_0, y_p))
        return res
        
    def get_plan(self, req):
        '''
        Calculate the plan from current image to the goal.
        Called from ROS srv. 
        '''
        self.info(str(req))
        self.info('Received a request for plan to goal')
        
        
        # Initiate response object
        res = camera_actuator.srv.planServiceResponse()  # @UndefinedVariable
        
        if self.state in [WAIT_GOAL, WAIT_CURRENT]:
            self.info('Module is in state: ' + self.state + 
                      'Unable to plan because of unspecified images.')
            res.plan = [-1]
            return res
        
        # Assert the module is idle state, otherwise refuse to plan.
        if not self.state == IDLE:
            self.info('Module is in state: ' + self.state + 
                      ' and will not accept new requests for plans right now')
            res.plan = [-1]
            return res
        
        self.info('Passed state check, OK')
        self.state = PLANNING  # indicate that the module is busy with planning
        
        # Plan
        instance = namedtuple('instance', ['y_stary', 'y_goal'])
        
        size = self.size
        y_start = self.get_y_current(size)
        y_goal = self.get_y_goal(size)
        instance.y_start = y_start
        instance.y_goal = y_goal
        
        # Run planning
        algo = self.online_planner.algo
        discdds = self.online_planner.discdds
        
        thresholds = rospy.get_param(self.node_name + '/planning_thresholds')
        if thresholds.__class__ == str:
            get_planning_thresholds = eval('online_planning.' + thresholds)
            thresh = get_planning_thresholds(algo=algo,
                                             discdds=discdds,
                                             active_instance=instance)
        else:
            thresh = thresholds
            
        precision, min_visibility = thresh
        self.info('Ready to plan with precision %g' % precision)
        
        result = self.online_planner.algo.plan(y_start,
                                               y_goal,
                                               precision,
                                               min_visibility)
        plan = result.plan
        
        self.info('Planning done, found plan: ' + str(plan))
        
        orbit = OrbitModule(size)
        
        
        res.plan = orbit.inverse_plan(plan)
        self.state = IDLE
        return res
    
    def reload_planner(self, req):
        '''
        Callback function for reloading planner from a rosservice.
        :param req:
        '''
        self.size = [160, 120]  # For now
        self.load_diffeoplan()
        res = camera_actuator.srv.voidServiceResponse()
        res.res = 0
        return res
        
    def get_y_current(self, size):
        image = get_image_array(self.y_current)
        image_resized = np.array(Image.fromarray(image).resize(size))
        return UncertainImage(image_resized)
    
    def get_y_goal(self, size):
        image = get_image_array(self.y_goal)
        image_resized = np.array(Image.fromarray(image).resize(size))
        return UncertainImage(image_resized)
    
    def __init__(self):
        self.node_name = 'diffeoplan_server'
        rospy.init_node(self.node_name)
        
        # Advertise a channel for info outputs 
        self.info_pub = rospy.Publisher(self.node_name + '/info', String)
        
        # Startup services
        plan_service = rospy.Service(self.node_name + '/get_plan',  # @UnusedVariable
                                     camera_actuator.srv.planService,  # @UndefinedVariable
                                     self.get_plan)
        
        reload_service = rospy.Service(self.node_name + '/reload_planner',  # @UnusedVariable
                                     camera_actuator.srv.voidService,  # @UndefinedVariable
                                     self.reload_planner)
        
        goal_service = rospy.Service(self.node_name + '/get_goal',  # @UnusedVariable
                                     camera_actuator.srv.imageService,
                                     self.get_goal_service)
        
        current_service = rospy.Service(self.node_name + '/get_current',  # @UnusedVariable
                                     camera_actuator.srv.imageService,
                                     self.get_current_service)
        
        distance_service = rospy.Service(self.node_name + '/get_distance',  # @UnusedVariable
                                     camera_actuator.srv.floatService,  # @UndefinedVariable
                                     self.get_distance)
        
        p_d_service = rospy.Service(self.node_name + '/get_predicted_distance',  # @UnusedVariable
                                    camera_actuator.srv.floatArrayService,  # @UndefinedVariable
                                    self.get_predicted_distance)
        
        
        # Startup message channels
        rospy.Subscriber(self.node_name + '/set_goal',
                         sensor_msgs.msg.Image,
                         self.set_goal)
        rospy.Subscriber(self.node_name + '/set_current',
                         sensor_msgs.msg.Image,
                         self.set_current)
        
        self.state = STARTING
        self.size = [160, 120]
        
          
    def set_default_params(self):
        rospy.set_param(self.node_name + '/config_path',
                        'default:/home/adam/diffeo-data/')
        
        rospy.set_param(self.node_name + '/id_discdds',
                        'orbit-pt256-160-n35s')
        
        rospy.set_param(self.node_name + '/id_algo',
                        'samp_t1_l5')
        
        rospy.set_param(self.node_name + '/planning_thresholds',
                        [0.12, 0])
    
    def load_diffeoplan(self):
        '''
        load parameters from ROS parameter server and activate a online planner 
        from the given parameters.
        '''
        config_path = rospy.get_param(self.node_name + '/config_path')
        
        config = DiffeoplanConfigMaster()
        config.load(config_path)
        set_current_config(config)
        
        id_discdds = rospy.get_param(self.node_name + '/id_discdds')
        id_algo = rospy.get_param(self.node_name + '/id_algo')

        diffeo_structure_threshold = 0.2
        
        # Check for threshold params, if evaluated to a list of tuple, then 
        # give the list or tuple as thresholds.
        thresholds = rospy.get_param(self.node_name + '/planning_thresholds')
        try:
            if eval(thresholds).__class__ in [list, tuple]:
                get_planning_thresholds = eval(thresholds)
            else:
                get_planning_thresholds = thresholds
        except:
            get_planning_thresholds = thresholds
        
        # dummy variables
        plan_length = 0
        num_tests = 0
        plans = [] 

        self.online_planner = OnlinePlanning(id_discdds,
                 diffeo_structure_threshold,
                 id_algo,
                 plan_length,
                 num_tests,
                 get_planning_thresholds,
                 plans)
        
        self.state = WAIT_GOAL
        

    def info(self, string):
        '''
        :param string: will be sent to logger and ros channel
        '''
        print(string)
        self.info_pub.publish(String(string))
    

def ros_main():
    dpServer = DiffeoplanRosServer()
    dpServer.set_default_params()
    dpServer.load_diffeoplan()
    pdb.set_trace()
