#!/usr/bin/env python
import roslib; roslib.load_manifest('diffeo_planner')

import rospy #@UnresolvedImport
import sys
import numpy as np
from optparse import OptionParser
import yaml
import array_msgs.msg #@UnresolvedImport

class CommandTester():
    def __init__(self):
        opts = OptionParser()
        opts.add_option('-s', '--system', default='orbit')
        options, _ = opts.parse_args()
        
        config_file = (roslib.packages.get_pkg_dir('diffeo_planner') + 
                       '/configs/' + options.system + '.yaml')
        system = yaml.load(open(config_file))
        self.system = system
        node_name = 'diffeo_planner_command_tester'
        self.node_name = node_name
        rospy.init_node(node_name)
        print('Node started')
        
#        cam_name = '/' + system['name']
        self.cmd_publisher = rospy.Publisher(system['ROS_msg_topic'], array_msgs.msg.FloatArray)

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
            
        
        
        
    def run(self):
        while not rospy.is_shutdown():
            try:
                sys.stdout.write('\033[45m' + self.node_name + '$\033[0m ')
                argc = sys.stdin.readline().split(' ')
                
                try:
                    outcommand = int(argc[0])
                    if outcommand < 0:
                        cmd = self.stop_command
                    else:
                        cmd = self.command_list[outcommand]
                        
                    self.cmd_publisher.publish(array_msgs.msg.FloatArray(cmd))
                except:
                    pass
            except:
                print('\033[91mError: Unexpected error occurred \033[0m')
        
if __name__ == '__main__':
    CommandTester().run()
    