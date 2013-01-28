from . import logger, np, rosbag
from . import imgmsg_to_pil
from .. import LogItem
import pdb


def get_image_array(image):
    im, _, _ = imgmsg_to_pil(image)
    pix = np.asarray(im).astype(np.uint8)
    return pix


    
def read_bag(bagfile):
    """ 
        Reads the data in the bag file and converts it as well.
    
        Use as follows:
        
            for Y0, u, Y1 in read_bag(bagfile):
                ....
    """
    logger.info('Reading bag file %r' % bagfile)
    bag = rosbag.Bag(bagfile)
    
    i = 0
    r = 0
    for topic, msg, t in bag.read_messages(topics=['Y0', 'Y1', 'U0']):
                
        #logger.info('Reading cmd')
        if topic == 'Y0':
            #print 'Y0 read @', t
            Y0_ros = msg
            if r == 0:
                r = 1
                t0 = t
                
        if topic == 'U0':
            #print 'U0 read @', t
            U0_ros = msg
            if r != 0:
                if t0 == t:
                    r += 1
                else:
                    logger.error('Error interpreting bag, unexpected timestamp,'
                                 ' ignoring message.')
                    r = 0
                
        if topic == 'Y1':
            #print 'Y1 read @', t
            Y1_ros = msg
            if r != 0:
                if t0 == t:
                    r += 1
                else:
                    logger.error('Error interpreting bag, unexpected timestamp,'
                                 ' ignoring message.')
                    r = 0
     
        if r == 3: # then a complete image pair and command is read
            
            # When done with the loop, yeild the last tuple
            #logger.info('Updating estimators %d' % i)
            i += 1
            Y0 = get_image_array(Y0_ros)
            Y1 = get_image_array(Y1_ros)
            U0 = U0_ros.data
            r = 0
            yield LogItem(y0=Y0, u=U0, y1=Y1, x0=None)

    
def read_bag_state(bagfile):
    print('Read bag state')
    logger.info('Reading bag file %r' % bagfile)
    bag = rosbag.Bag(bagfile)
    # Check if the bag has states recorded
    has_X0 = False
    for topic, msg, t in bag.read_messages(topics=['X0']):
        # If one state is read, then proceed with reading with states
        has_X0 = True
    logger.info('bagfile has state records: ' + str(has_X0))
    if not has_X0:
        state = None
    else:
        state = None
        
    i = 0
    r = 0
    for topic, msg, t in bag.read_messages(topics=['Y0', 'Y1', 'U0', 'X0']):
        #logger.info('Reading cmd')
        if topic == 'Y0':
            #print 'Y0 read @', t
            Y0_ros = msg
            if r == 0:
                r = 1
                t0 = t
                
        if topic == 'U0':
            #print 'U0 read @', t
            U0_ros = msg
            if r != 0:
                if t0 == t:
                    r += 1
                else:
                    logger.error('Error interpreting bag, unexpected timestamp,'
                                 ' ignoring message.')
                    r = 0
                
        if topic == 'Y1':
            #print 'Y1 read @', t
            Y1_ros = msg
            if r != 0:
                if t0 == t:
                    r += 1
                else:
                    logger.error('Error interpreting bag, unexpected timestamp,'
                                 ' ignoring message.')
                    r = 0

        if topic == 'X0':
            state = msg.data
            if r != 0:
                if t0 == t:
                    r += 1
                else:
                    pdb.set_trace()
                    logger.error('Error interpreting bag, unexpected timestamp,'
                                 ' ignoring message.')
                    r = 0
             
        if r == 4 or (not has_X0 and r == 3): # then a complete image pair and command is read
            #logger.info('Updating estimators %d' % i)
            i += 1
            Y0 = get_image_array(Y0_ros)
            Y1 = get_image_array(Y1_ros)
            U0 = U0_ros.data
            r = 0
            yield LogItem(y0=Y0, u=U0, y1=Y1, x0=state)
