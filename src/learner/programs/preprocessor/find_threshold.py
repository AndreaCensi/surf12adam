from . import logger
import numpy as np
import rosbag
from diffeoplan.library.logs.rosbag.bag_reader import get_image_array

def find_dt_threshold(bagfile, stream, distance, ignore_first=100, max_images=300):
    '''
    Finds the value of dy/dt for when the robot is probably moving
    :param infile:    File to read data from
    '''
    bag = rosbag.Bag(bagfile)
    logger.info('Calibrating diff_threshold for camera motion...')
    ic = 0

    Y_last = None
    diffs = []
    for _, msg, _ in  bag.read_messages(stream):
        Y = get_image_array(msg)
        ic = ic + 1
               
        if ic > max_images:
            print('break')
            break
        if ic < ignore_first:
            continue
        
        if Y_last is not None:
            diff = distance(Y_last, Y)
            diffs.append(diff)
        
        Y_last = Y
        
    bag.close()
    diff_threshold = 2 * np.mean(diffs) 
    logger.info('diff_threshold = %g ' % diff_threshold)
    return diff_threshold
