from . import logger
import rosbag
import numpy as np

from .ros_conversions import imgmsg_to_pil

def get_image_array(image):
    im, _, _ = imgmsg_to_pil(image)
    pix = np.asarray(im).astype(np.uint8)
    return pix

        
def read_bag(bagfile):
    """ 
        Reads the data in the bag file and converts it as well.
    
        Use as follows:
        
            for Y0, u, Y1 in learn_bag(bagfile):
                ....
    """
    logger.info('Learning from %r' % bagfile)
    bag = rosbag.Bag(bagfile)
    
    i = 0
    r = 0
    for topic, msg, t in bag.read_messages(topics=['Y0', 'Y1', 'U0']):
        logger.info('Reading cmd')
        if topic == 'Y0':
            print 'Y0 read @', t
            Y0_ros = msg
            if r == 0:
                r = 1
                t0 = t
                
        if topic == 'U0':
            print 'U0 read @', t
            U0_ros = msg
            if r != 0:
                if t0 == t:
                    r += 1
                else:
                    logger.error('Error interpreting bag, unexpected timestamp, ignoring message.')
                    r = 0
                
        if topic == 'Y1':
            print 'Y1 read @', t
            Y1_ros = msg
            if r != 0:
                if t0 == t:
                    r += 1
                else:
                    logger.error('Error interpreting bag, unexpected timestamp, ignoring message.')
                    r = 0
                    
        # If r == 3, then a complete image pair and command is read
        if r == 3:
            logger.info('Updating estimators %d' % i)
            i += 1
            Y0 = get_image_array(Y0_ros)
            Y1 = get_image_array(Y1_ros)
            U0 = U0_ros.data
            yield Y0, U0, Y1
            r = 0
