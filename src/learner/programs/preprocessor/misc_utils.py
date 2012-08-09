#import roslib 
#import rosbag
#import sys
#roslib.load_manifest('logitech_cam')
#import rospy
#from std_msgs.msg import String
#import sensor_msgs
#from sensor_msgs.msg import Image
#import ParseMessages
#import numpy
#from PIL import Image
#import ImageChops, ImageOps
#import boot_agents
#from boot_agents import *
#import pdb
#import pickle
#import yaml
#from diffeoplan.library.discdds import diffeo_action
#from diffeoplan.library.discdds import diffeo_system
#from optparse import OptionParser

#
#def get_angle(D):
#    Y, X = np.meshgrid(range(y0.shape[1]), range(y0.shape[0]))
#    Dx = D.d[:, :, 0] - X
#    Dy = D.d[:, :, 1] - Y
#    #print Dx
#    #print Dy
#    Dx_f = Dx.reshape(Dx.size)
#    Dy_f = Dy.reshape(Dy.size)
#    var_f = D.variance.reshape(Dx.size)
#    var_mean = mean(var_f)
#    at = []
#    for i in range(Dx.size):
#        if abs(var_f[i]) > abs(var_mean):
#            at.append(np.arctan2(Dy_f[i], Dx_f[i]))
#    #at = np.arctan(D.d[:,:,1]-Y,D.d[:,:,0]-X)
#    return (np.mean(at), np.std(at))
