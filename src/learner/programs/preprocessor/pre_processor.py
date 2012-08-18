from . import logger
from learner.programs.preprocessor import topic_image_raw
from learner.programs.preprocessor.bag_processing import (read_processed_data,
    read_Y0UY1_tuples)
from learner.programs.preprocessor.find_threshold import find_dt_threshold
from learner.programs.preprocessor.utils import image_distance_L1
from learner.programs.preprocessor.zoomer import Zoomer
import rosbag
import pdb
from learner.programs.diffeo_learner.ros_conversions import numpy_to_imgmsg


def preprocess(infile, outfile, output_size,
                use_zoom=True,
                min_zoom=100, max_zoom=500):
    logger.info('Preprocessing file %r' % infile)
    
    distance = image_distance_L1
    
    diff_threshold = find_dt_threshold(infile,
                                           topic_image_raw, distance=distance,
                                           ignore_first=100, max_images=300)
    
    
    zoomer = Zoomer(use_zoom=use_zoom,
                    min_zoom=min_zoom,
                    max_zoom=max_zoom,
                    output_size=output_size)

    data_stream = read_processed_data(infile, zoomer)
    tuples_stream = read_Y0UY1_tuples(data_stream,
                                      image_distance=distance,
                                      threshold=diff_threshold)
    
    out_bag = rosbag.Bag(outfile, 'w')
    for Y0, U, Y1 in tuples_stream:
        write_stuff(out_bag, Y0, U, Y1)
    out_bag.close()

      
def write_stuff(bag, Y0, U, Y1):
    # write to the bag
    y0msg = numpy_to_imgmsg(Y0[1])
    y1msg = numpy_to_imgmsg(Y1[1])
    bag.write('Y0', y0msg, U[0])
    bag.write('U0', U[1], U[0])
    bag.write('Y1', y1msg, U[0])

