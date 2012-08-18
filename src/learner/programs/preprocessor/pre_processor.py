from . import logger
from learner.programs.preprocessor import topic_image_raw
from learner.programs.preprocessor.bag_processing import (read_processed_data,
    read_Y0UY1_tuples)
from learner.programs.preprocessor.find_threshold import find_dt_threshold
from learner.programs.preprocessor.utils import image_distance_L1
from learner.programs.preprocessor.zoomer import Zoomer
import rosbag



def preprocess(infile, outfile, output_size,
                use_zoom=True,
                min_zoom=100, max_zoom=500):
    logger.info('Preprocessing file %r' % infile)
    
    distance = image_distance_L1
    
    diff_treshold = find_dt_threshold(infile,
                                           topic_image_raw, distance=distance,
                                           ignore_first=100, max_images=300)
    
    zoomer = Zoomer(use_zoom=use_zoom,
                    min_zoom=min_zoom,
                    max_zoom=max_zoom,
                    output_size=output_size)
    
    data_stream = read_processed_data(infile, zoomer)
    tuples_stream = read_Y0UY1_tuples(data_stream,
                                      image_distance=distance,
                                      threshold=diff_treshold)
    
    out_bag = rosbag.Bag(outfile, 'w')
    for Y0, U, Y1 in tuples_stream:
        write_stuff(out_bag, Y0, U, Y1)
    out_bag.close()

      
def write_stuff(bag, Y0, U, Y1):
    # write to the bag
    logger.info('Writing tuple')
    pass
        
                
#          
#    def validate_bag(self, outfile): 
#        bag = rosbag.Bag(outfile)
#        ver_bag = rosbag.Bag('verify.bag', 'w')
#        topics = ['Y0', 'U0', 'Y1']
#        last_topic = 'Y1'
#        seq_error = False
#        for topic, msg, t in bag.read_messages(topics=topics):
#            if topic == 'Y0':
#                if last_topic == 'Y1':
#                    pass
#                else:
#                    logger.info('Wrong sequence')
#                    seq_error = True
#
#            if topic == 'U0':
#                if last_topic == 'Y0':
#                    pass
#                else:
#                    logger.info('Wrong sequence')
#                    seq_error = True
#            
#            if seq_error:
#                ver_bag.write(topic, msg, t)
#                        
#            if topic == 'Y1':
#                seq_error = False
#
#            last_topic = topic
#            
#        ver_bag.close()
                
#                
#    
#def find_Y0(img_container, diff_threshold):
#    '''
#    Return the image before the camera started to move
#    :param img_container:
#    :param diff_threshold:
#    '''
#    n = len(img_container)
#    state = 0
#    for i in range(n - 1):
#        diff = compare_images(img_container[i], img_container[i + 1])
#        if (diff > diff_threshold) and (i < 5):
#            # Cam moving, search for stop
#            state = 1
#        
#        if (i >= 5) and (state == 0):
#            return img_container[0]
#        
#        if state == 1:
#            # Search for stop in motion (backwards in time)
#            if diff < diff_threshold:
#                logger.info('stopped at index %g ' % i)
#                if compare_images(img_container[i + 1], img_container[i + 2]) < diff_threshold:
#                    # camera was stopped as well
#                    return img_container[i + 2]
#                else:
#                    # Camera was stopped now but not on next one, return this image
#                    return img_container[i + 1]
#    # nothing found, return most recent image
#    return img_container[0]
