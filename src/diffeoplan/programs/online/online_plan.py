'''
Created on Oct 4, 2012

@author: adam
'''
import camera_actuator.srv #@UnresolvedImport

from .. import set_current_config

from bootstrapping_olympics.utils import substitute
from conf_tools import ConfToolsException
from diffeoplan.configuration import DiffeoplanConfigMaster
import contracts

from . import logger
#import roslib 
import rospy
#import sensor_msgs.msg
import pdb
from diffeoplan.library.logs.rosbag.bag_reader import get_image_array
from PIL import Image #@UnresolvedImport
import sys
from diffeoplan.utils.lenient_option_parser import MyOptionParser

def main():
    config = DiffeoplanConfigMaster()
    config.load('default:.')
    
    algos = config.algos.expand_names('begc3')
    
    logger.info('Using %d algorithms: %s' % (len(algos), algos))
    
    outdir = 'out'
    
    pdb.set_trace()
    

def get_problem():
    logger.info('Starting online module')
    rospy.init_node('diffeoplan', anonymous=True)
    
    takeImage = rospy.ServiceProxy('/logitech_cam/take_image', camera_actuator.srv.imageService)
    
    logger.info('Place camera at goal configuration and press any key to take picture')
    sys.stdin.readline()
    image_goal = get_image_array(takeImage(0).image)
    logger.info('Goal image taken')
    
    logger.info('Place camera at start configuration and press any key')
    sys.stdin.readline()
    image_start = get_image_array(takeImage(0).image)
    
    Image.fromarray(image_goal).save('goal.png')
    Image.fromarray(image_start).save('start.png')
    
    return image_start, image_goal
