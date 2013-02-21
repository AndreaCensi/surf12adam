from . import logger
from learner.programs.preprocessor import topic_image_raw
from learner.programs.preprocessor.bag_processing import (read_processed_data,
    read_Y0UY1_tuples)
from learner.programs.preprocessor.find_threshold import find_dt_threshold
from learner.programs.preprocessor.utils import image_distance_L1
from learner.programs.preprocessor.zoomer import Zoomer
import rosbag
from PIL import Image #@UnresolvedImport
from diffeoplan.library.logs.rosbag.ros_conversions import pil_to_imgmsg
import std_msgs.msg
import pdb


def preprocess(infile, outfile, output_size,
                use_zoom=True,
                min_zoom=100, max_zoom=200, zoom_factor=5):
    logger.info('Preprocessing file %r' % infile)
    
    distance = image_distance_L1
    
    diff_threshold = find_dt_threshold(infile,
                                           topic_image_raw, distance=distance,
                                           ignore_first=100, max_images=300)
    
    
    zoomer = Zoomer(use_zoom=use_zoom,
                    min_zoom=min_zoom,
                    max_zoom=max_zoom,
                    zoom_factor=zoom_factor,
                    output_size=output_size)

    data_stream = read_processed_data(infile, zoomer)
    tuples_stream = read_Y0UY1_tuples(data_stream,
                                      image_distance=distance,
                                      threshold=diff_threshold)
    
    out_bag = rosbag.Bag(outfile, 'w')
#    i = 0
    for Y0, U, Y1 in tuples_stream:
        write_tuple_to_bag(out_bag, Y0, U, Y1, x=zoomer.current_zoom)
#        i += 1
#        if i > 20:
#            break
    out_bag.close()

      
def write_tuple_to_bag(bag, Y0, U, Y1, x=0):
    # write to the bag
    y0msg = pil_to_imgmsg(Image.fromarray(Y0[1]))
    y0msg.header.stamp = U[0]
    y1msg = pil_to_imgmsg(Image.fromarray(Y1[1]))
    y1msg.header.stamp = U[0]
#    pdb.set_trace()
    bag.write('Y0', y0msg, U[0])
    bag.write('U0', U[1], U[0])
    bag.write('Y1', y1msg, U[0])
    try:
        bag.write('X0', std_msgs.msg.Int64(int(x)), U[0])
    except:
        pdb.set_trace()

