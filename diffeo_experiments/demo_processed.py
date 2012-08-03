#!/usr/bin/env python
import roslib
roslib.load_manifest('logitech_cam')
import sys
import rosbag
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
#import cv
import ParseMessages
#from cv_bridge import CvBridge, CvBridgeError
from PIL import Image
import pdb

from logitech_cam.msg import IntArray

def add_noise(Y, noise):
    """
    Add a noise of max size <noise> to the image Y
    """
    if noise == 0:
        return Y
    dtype0 = Y.dtype
    N = np.random.randint(-noise,noise,Y.shape).astype(np.int16)
    Y = N + Y
    if dtype0 == np.uint8:
        Y[Y<0]=0
        Y[Y>255]=255
    return Y.astype(dtype0)

class logitech_cam_simulation:
    def __init__(self,size,world,scale_factor):
        self.zoom = 1.0
        self.zoom_max = 2.0
        self.zoom_min = 0.5
        self.scale_factor = np.array(scale_factor)
        self.world = world
        self.nois_level = 0
        #self.command_list 
        
    def get_Y_pair(self,(x0,y0),(w,h),(dx,dy),zoom=[1.0, 1.0]):
        
#        pdb.set_trace()
        im = self.world
        dw2 = w*(0.5-0.5*zoom[0])
        dh2 = h*(0.5-0.5*zoom[0])
        imc = im.crop(np.array([x0-dw2,y0-dh2,x0+w+dw2,y0+h+dh2]).astype(np.int)).resize((w,h))
        Y0 = np.array(imc.getdata(),np.uint8).reshape((h,w,3))
#        pdb.set_trace()
        dw2 = w*(0.5-0.5*zoom[1])
        dh2 = h*(0.5-0.5*zoom[1])
        imc2 = im.crop(np.array([x0+dx-dw2,y0+dy-dh2,x0+dx+w+dw2,y0+dy+h+dh2]).astype(np.int)).resize((w,h))
        Y1 = np.array(imc2.getdata(),np.uint8).reshape((h,w,3))
        return (Y0,Y1)
    
    def simulate_Y_pair(self,size,command):
#        pdb.set_trace()
        print('simulate')
        if (self.zoom+float(command[2])*self.scale_factor[2]<self.zoom_max and self.zoom+float(command[2])*self.scale_factor[2]>self.zoom_min):
            delta = np.array(command[:2])*self.scale_factor[:2]
            im = self.world
    #        self.zoom = self.zoom + command[2]*self.scale_factor[2]
            zoom = [self.zoom, self.zoom+float(command[2])*self.scale_factor[2]]
            
            P0 = (np.random.randint(abs(delta[0]),im.size[0]-abs(delta[0])-size[0]*2),np.random.randint(abs(delta[1]),im.size[1]-abs(delta[1])-size[1]*2))
    #        P0 = (np.random.randint(abs(delta[0]),im.size[0]*max(zoom)-abs(delta[0])),np.random.randint(abs(delta[1]),im.size[1]*max(zoom)-abs(delta[1])))
    #        self.get_Y_pair(P0,delta,im)
            
            self.zoom = zoom[1]
    #        pdb.set_trace()
            Y0,Y1 = self.get_Y_pair(P0,size,delta,zoom)
            Y0 = add_noise(Y0,self.nois_level)
            Y1 = add_noise(Y1,self.nois_level)
            return (Y0,Y1,command)
        else: 
            return (0,0,0)

# Initiate
print len(sys.argv)
for arg in sys.argv:
    print arg

try:
    outfile = sys.argv[sys.argv.index('-o')+1]
except ValueError:
    print 'No output file specified'
    outfile = 'outbag.bag'
    
try:
    image_str = sys.argv[sys.argv.index('-i')+1]
except ValueError:
    print 'No input file specified'
    image_str = 'random'
    
try:
    size = eval(sys.argv[sys.argv.index('-size')+1])
except ValueError:
    print 'No output file specified'
    size = (64,48)
    
try:
    duration = int(sys.argv[sys.argv.index('-t')+1])
except ValueError:
    print 'No output file specified'
    duration = 10
        
try:
    command_list = eval(sys.argv[sys.argv.index('-c')+1])
except ValueError:
    print 'No output file specified'
    command_list = [[200,0,0],[-200,0,0],[0,150,0],[0,-150,0],[0,0,5],[0,0,-5],[0,0,0]]


scale_factor = [1.0/50,2.0/50,1.0/50]
#command_list = [[200,0,0],[-200,0,0],[0,150,0],[0,-150,0],[0,0,5],[0,0,-5],[0,0,0]]
#command_list = [[0,0,5],[0,0,-5]]


# Keep track of the actual zoom for the last command and the current zoom
zoom_last = 1.0
zoom = 1.0

out_bag = rosbag.Bag(outfile, 'w')

# Load image to crop subimages from
if image_str == 'random':
    M = np.random.randint(0,255,(748,1024,3)).astype(np.uint8)
    im = Image.fromarray(M)
else:
    im = Image.open(image_str+'.png')

cam_sim = logitech_cam_simulation(size,im,scale_factor)
#
Y0,Y1,cmd = cam_sim.simulate_Y_pair(size,[0,0,50])
print('simulated')
def show(Y0,Y1):
    Image.fromarray(Y0).show()
    Image.fromarray(Y1).show()
#pdb.set_trace()
for i in range(duration):
    print(i)
    command = command_list[np.random.randint(0,len(command_list))]
    Y0,Y1,cmd = cam_sim.simulate_Y_pair(size,command)
#    Image.fromarray(Y0).save('test'+str(i)+'y0cmd'+str(cmd)+'.png')
#    Image.fromarray(Y1).save('test'+str(i)+'y1cmd'+str(cmd)+'.png')
#    pdb.set_trace()
    if Y0.__class__ == int:
        print('Ignoring message')
    else:
        Y0ros = ParseMessages.numpy_to_imgmsg(Y0)
        Y1ros = ParseMessages.numpy_to_imgmsg(Y1)
        out_bag.write('Y0',Y0ros,rospy.Time(i+1))
        out_bag.write('Y1',Y1ros,rospy.Time(i+1))
        cmd_msg = IntArray()
        cmd_msg.data = cmd
        out_bag.write('U0',cmd_msg,rospy.Time(i+1))

out_bag.close()
