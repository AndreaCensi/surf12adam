#!/usr/bin/env python
import roslib; roslib.load_manifest('diffeo_planner')

import yaml
from optparse import OptionParser

import rospy #@UnresolvedImport
import random
import array_msgs.msg #@UnresolvedImport
import pdb

def main():
    opts = OptionParser()
    opts.add_option('-s', '--system', default='orbit')
    opts.add_option('-l', '--length', default=5, type=int)
    opts.add_option('-f', '--frequence', default=1., type=float)
    options, args = opts.parse_args()
    
    config_file = (roslib.packages.get_pkg_dir('diffeo_planner') + 
                       '/configs/' + options.system + '.yaml')
    
    if not os.path.exists(config_file):
        msg = 'File %r does not exist; use -s switch.' % config_file
        raise Exception(msg)

    system = yaml.load(open(config_file))
    
    node_name = 'learn_command_generator'
    node_name = node_name
    rospy.init_node(node_name)
    print('Node started')
    pdb.set_trace()
    posp = rospy.Publisher(system['position_topic'], array_msgs.msg.FloatArray)
    hm = array_msgs.msg.FloatArray()
    hm.data = system['home_position']
    pdb.set_trace()
    posp.publish(hm)
    pdb.set_trace()
#    rospy.sleep(5)
    
    cmd_publisher = rospy.Publisher(system['ROS_msg_topic'], array_msgs.msg.FloatArray)
    
    stop_command = system['stop_command']
    
    if not system.has_key('command_list'):
        print('Incremental control requires a command_list in config file')
    
    command_list = system['command_list']
    nc = len(command_list)
    r = rospy.Rate(options.frequence)
    while not rospy.is_shutdown():
        cmd_index = random.randint(0,nc-1)
        num_cmd = random.randint(0,options.length)
        for _ in range(num_cmd):
            cmd = command_list[cmd_index]
            print('Command: %s' % cmd)
            cmd_publisher.publish(array_msgs.msg.FloatArray(cmd))
            r.sleep()
            
    cmd_publisher.publish(array_msgs.msg.FloatArray(stop_command))
    
    
if __name__ == '__main__':
    main()