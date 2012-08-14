from . import logger, read_bag
from ... import DiffeoLearner
from optparse import OptionParser
import os


def diffeo_learner_main():
    usage = "usage: %prog -i inputbag -p outputpath -n ddsname -s [W,H] -a [rx,ry]"
    parser = OptionParser(usage=usage, version="%prog 1.0")
    parser.add_option("-i", "--input", default=None,
                      help="Input processed.bag file")
    parser.add_option("-p", "--path", default=None,
                      help="Path to output files")
    parser.add_option("-n", "--name", default='camdds',
                      help="Output dds system name")
    parser.add_option("-s", "--size", default='[160,120]',
                      help="Image size WxH")
    parser.add_option("-a", "--area", default='[6,6]',
                      help="Size of search area")
    options, args = parser.parse_args()
    if args: 
        raise ValueError('spurious arguments %s' % args)
    
    if options.input is None:
        raise ValueError('Required options -i')
    
    if options.path is None:
        dirname = os.path.dirname(options.input)
        logger.info('No path given; using %s' % dirname)
    else:
        dirname = options.path
        
    bagfile = options.input
    name = options.name
    size = eval(options.size)
    area = eval(options.area)

    learn = DiffeoLearner(size, area)
    for y0, u, y1 in read_bag(bagfile):
        learn.update(y0, u, y1)
        
        
    logger.info('Commands: %s' % learn.command_list)
    learn.summarize()
    
    learn.diffeo_dump(dirname, name)
    learn.show_diffeomorphisms()

