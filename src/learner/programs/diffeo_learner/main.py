from . import logger, read_bag
from ... import DiffeoLearner
from optparse import OptionParser
import os
import pickle
import pdb


def diffeo_learner_main():
    usage = "usage: %prog   -p outputpath -n ddsname -s [W,H] -a [rx,ry] bag1 bag2 .."
    parser = OptionParser(usage=usage, version="%prog 1.0") 
    parser.add_option("-p", "--path", default=None,
                      help="Path to output files")
    parser.add_option("-n", "--name", default='camdds',
                      help="Output dds system name")
    parser.add_option("-s", "--size", default='[160,120]',
                      help="Image size WxH")
    parser.add_option("-a", "--area", default='[6,6]',
                      help="Size of search area")
    parser.add_option("-l", "--learner", default='No',
                      help="Reuse learner")
    parser.add_option("-o", "--outlearner", default='No',
                      help="Save the learner")
    options, args = parser.parse_args()
     
    if not args:
        raise ValueError('Required file names/.')
    
    if options.path is None:
        dirname = os.path.dirname(options.input)
        logger.info('No path given; using %s' % dirname)
    else:
        dirname = options.path
         
    name = options.name
    size = eval(options.size)
    area = eval(options.area)

    learner = options.learner
    outlearner = options.outlearner

    if learner == 'No':
        learn = DiffeoLearner(size, area)
    else:
        learn = pickle.load(open(learner))
    pdb.set_trace()
    
    for bagfile in args:
        i = 0
        for y0, u, y1 in read_bag(bagfile):    
            logger.info('Iteration number %d' % i)
            i = i + 1
            learn.update(y0, u, y1)
        
        
    logger.info('Commands: %s' % learn.command_list)
    learn.summarize()
    
    learn.diffeo_dump(dirname, name)
    learn.show_diffeomorphisms()
    
    
    if outlearner == 'No':
        pass
    else:
        pickle.dump(learn, open(outlearner, 'wb'))

