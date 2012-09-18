'''
Created on Sep 18, 2012

@author: adam
'''



from scipy.signal import convolve2d

import rosbag
from PIL import Image #@UnresolvedImport
import sys
from diffeoplan.library.logs.rosbag.ros_conversions import imgmsg_to_pil

print(sys.argv)

bagfile = open(sys.argv[1])
bag = rosbag.Bag(bagfile)
topics = ['Y0']
for topic, msg, t in bag.read_messages(topics=topics):
    img = imgmsg_to_pil(msg)
    print(t)
