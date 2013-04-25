from . import logger
from optparse import OptionParser
from diffeoplan.utils.script_utils import UserError
import os
import rosbag
import yaml
from learner.programs.preprocessor import topic_image_raw
import sys
from diffeoplan.library.logs.rosbag.bag_reader import get_image_array
from PIL import Image  # @UnresolvedImport
from diffeoplan.library.logs.rosbag.ros_conversions import pil_to_imgmsg


def youbot_preprocessor():
    usage = "usage: %prog -i inputname [-o outputdir] -s [W,H] -z bool"
    parser = OptionParser(usage=usage, version="%prog 1.0")
    parser.add_option("-i", "--input",
                      help="Path and name to input file (must be *.raw.bag)")
    parser.add_option("-o", "--output",
                      help="Path to output directory")
    parser.add_option("-s", "--size", default='[160,120]', help="Image size WxH")


    options, _ = parser.parse_args(sys.argv)

    bag = options.input 
    
    if bag is None:
        msg = 'Must specify input file with -i.'
        raise UserError(msg)

    if not os.path.exists(bag):
        msg = 'Filename %r does not exist.' % bag
        raise UserError(msg)
    
    input_suffix = '.raw.bag'
    
    if not input_suffix in bag:
        msg = 'I expect a %s as input, got %r.' % (input_suffix, bag)
        raise UserError(msg)
    
    if options.output is None:
        outdir = os.path.dirname(bag)
        msg = 'No output dir specified; I will use %s' % outdir
        logger.info(msg)
    else:
        outdir = options.output
        
    basename = os.path.basename(bag)    
    basename = basename[:-len(input_suffix)]
    
    output_size = tuple(eval(options.size))
    
    
    out_name = basename + '%sx%s' % output_size
    
    output = os.path.join(outdir, out_name + '.processed.bag')

    logger.info('Reading from: %s' % bag)
    logger.info('Writing to:   %s' % output)
    
    preprocess(bag, output, output_size)
    
def preprocess(bagfile, output, output_size):

    """ yields topic, data, t but data already a numpy array """
    bag = rosbag.Bag(bagfile)
    info = yaml.load(bag._get_yaml_info())
    for topic_dict in info['topics']:
        if 'FloatArray' in topic_dict['type']:
            command_topic = topic_dict['topic']
            
    topics = [topic_image_raw, command_topic]
    
    logger.info('bag_processing, using topics: %s' % topics)
    
    
    def read_tuples(raw_stream):
        i = 0
        Y0 = None
        U0 = None
        Y_last = None
        t0 = 0
        
        for topic, msg, t in raw_stream:
#            logger.debug('read msg type %s and topic %s' % (msg._type, topic))
            i += 1
            
            if topic == command_topic:
#                logger.debug('command read')
                Y1 = Y_last
                if Y0 is not None and U0 is not None:
                    # yeild tuple
                    yield (Y0, U0, Y1, t0)
                
                Y0 = Y_last
                U0 = msg
                t0 = t
                
            if topic == topic_image_raw:
#                logger.debug('image read')
                Y_last = msg
    
    tuples_stream = read_tuples(bag.read_messages(topics=topics))
    
    out_bag = rosbag.Bag(output, 'w')
    for Y0, U, Y1, t0 in tuples_stream:
        y0_pil = Image.fromarray(get_image_array(Y0)).resize(output_size)
        y1_pil = Image.fromarray(get_image_array(Y1)).resize(output_size)
#        pdb.set_trace()
        logger.info('Writing data (Y0, U0=%s, Y1)' % U)
        out_bag.write('Y0', pil_to_imgmsg(y0_pil), t0)
        out_bag.write('U0', U, t0)
        out_bag.write('Y1', pil_to_imgmsg(y1_pil), t0)

    out_bag.close()
