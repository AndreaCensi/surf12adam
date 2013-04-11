#!/usr/bin/env python
import roslib; roslib.load_manifest('diffeo_planner')

import os
import yaml
from optparse import OptionParser

import rospy #@UnresolvedImport
import random
import numpy as np

import array_msgs.msg #@UnresolvedImport
from array_msgs.msg import FloatArray
from geometry_msgs.msg import Twist
from numpy.testing import assert_almost_equal


class Babbler:

    def main(self):
        opts = OptionParser()
        opts.add_option('--system')
        opts.add_option('--min_length', default=3, type=int)
        opts.add_option('--max_length', default=10, type=int)
        opts.add_option('--frequency', default=1, type=float)
        opts.add_option('--levy', default=1, type=int)
        options, args = opts.parse_args()

        if options.system is None:
            msg = 'Please specify system with --system.' 
            raise Exception(msg)

        config_file = (roslib.packages.get_pkg_dir('diffeo_planner') + 
                           '/configs/' + options.system + '.yaml')
        
        if not os.path.exists(config_file):
            msg = 'File %r does not exist; use -s switch.' % config_file
            raise Exception(msg)

        system = yaml.load(open(config_file))
        
        rospy.init_node('babbler')
        
        position_topic = system.get('position_topic', None)
        if position_topic:
            posp = rospy.Publisher(system['position_topic'], FloatArray)
            hm = array_msgs.msg.FloatArray()
            hm.data = system['home_position']
            pdb.set_trace() # XXX
            posp.publish(hm)
            pdb.set_trace()
        else:
            rospy.loginfo('Skipping position command.')

        self.system = system
        
        if not 'command_list' in system:
            msg = 'Incremental control requires a command_list in config file'
            raise Exception(msg)
        
        self.command_list = system['command_list']
        self.min_length = options.min_length
        self.max_length = options.max_length
        self.levy = options.levy
        self.frequency = options.frequency
        self.cmd_publisher = rospy.Publisher(system['ROS_msg_topic'], FloatArray)

        self.queue = []
        rospy.Subscriber('/youbot_safety/unsafe_cmd_vel', Twist, self.skip_callback)

        self.schedule_call()
        
        
        self.last_command = None

        rospy.spin()
        # shut down (XXX)
        stop_command = system['stop_command']
        self.cmd_publisher.publish(FloatArray(stop_command))
        

    def schedule_call(self):
        rospy.Timer(rospy.Duration(1.0/self.frequency), self.periodic, oneshot=True)

    def skip_callback(self, msg):
        rospy.loginfo('Got a skip notification')
        self.queue = []

    def get_next_command_probabilities(self):
        nc = len(self.command_list)

        # let's define the probabilities
        def prob_cmd(i):
            if not self.levy:
                return 1.0
            if self.last_command is None:
                return 1.0
            if i == self.last_command:
                return 0.0
            else:
                v1 = np.array(self.command_list[self.last_command])
                v2 = np.array(self.command_list[i])
                # Don't go for the opposite
                if np.allclose(v1, -v2):
                    return 0.0
                else:
                    return 1.0

        probabilities = np.array(map(prob_cmd, range(nc)))
        probabilities /= np.sum(probabilities)

        assert_almost_equal(len(probabilities), nc)
        assert_almost_equal(np.sum(probabilities), 1.0)

        return probabilities

    def choose_next_command(self):
        probabilities = self.get_next_command_probabilities()
        rospy.loginfo('Prob: %s' % probabilities)
        nc = len(self.command_list)
        cmd_index = numpy_choice(range(nc), p=probabilities)
        self.last_command = cmd_index
        num_cmd = random.randint(self.min_length, self.max_length) # XXX: float

        cmd = self.command_list[cmd_index]
        rospy.loginfo('Sampled %s x %s' % (num_cmd, cmd))
        for _ in range(num_cmd):
            self.queue.append(cmd)

    def periodic(self, msg):
        if not self.queue:
            self.choose_next_command()
    
        cmd = self.queue.pop(0)    
        self.cmd_publisher.publish(FloatArray(cmd))
                
        rospy.loginfo('Sending %s' % cmd)

        self.schedule_call()
 
def numpy_choice(a, size=1, replace=True, p=None):
   # Backported to 1.3

    # Format and Verify input
    if isinstance(a, int):
        if a > 0:
            pop_size = a #population size
        else:
            raise ValueError("a must be greater than 0")
    else:
        a = np.asarray(a)
        if len(a.shape) != 1:
            raise ValueError("a must be 1-dimensional")
        pop_size = a.size
        if pop_size is 0:
            raise ValueError("a must be non-empty")

    if None != p:
        p = np.asarray(p)
        if len(p.shape) != 1:
            raise ValueError("p must be 1-dimensional")
        if p.size != pop_size:
            raise ValueError("a and p must have same size")
        if any(p < 0):
            raise ValueError("probabilities are not non-negative")
        if not np.allclose(p.sum(), 1):
            raise ValueError("probabilities do not sum to 1")

    # Actual sampling
    if replace:
        if None != p:
            cdf = p.cumsum()
            uniform_samples = np.random.random(size)
            idx = cdf.searchsorted(uniform_samples, side='right')
        else:
            idx = self.randint(0, pop_size, size=size)
    else:
        if size > pop_size:
            raise ValueError(''.join(["Cannot take a larger sample than ",
                                      "population when 'replace=False'"]))
        
        if None != p:
            if np.sum(p>0) < size:
                raise ValueError("Fewer non-zero entries in p than size")
            n_uniq = 0
            p = p.copy()
            found = np.zeros(size, dtype=np.int)
            while n_uniq < size:
                x = self.rand(size-n_uniq)
                if n_uniq > 0:
                    p[found[0:n_uniq]] = 0
                p = p/p.sum()
                cdf = np.cumsum(p)
                new = cdf.searchsorted(x, side='right')
                new = np.unique(new)
                found[n_uniq:n_uniq+new.size] = new
                n_uniq += new.size
            idx = found
        else:
            idx = self.permutation(pop_size)[:size]

    #Use samples as indices for a if a is array-like
    if isinstance(a, int):
        return idx
    else:
        return a.take(idx)

    
if __name__ == '__main__':
    Babbler().main()