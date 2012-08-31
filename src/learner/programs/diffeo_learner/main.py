from . import logger, read_bag
from optparse import OptionParser
import os
import pickle
import urllib
from learner.diffeo_learner import DiffeoLearner


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
     
#    if not args:
#        raise ValueError('Required file names/.')
    
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
        logger.info('Loading diffeomorphism estimators')
        if learner[:4] == 'http':
            logger.info('from online source: %s' % learner)
            ofile = urllib.urlopen(learner)
        else:
            logger.info('from local source: %s' % learner)
            ofile = open(learner)
        learn = pickle.load(ofile)
        logger.debug('loaded')
    
    for bagfile in args:
        i = 0
        for y0, u, y1 in read_bag(bagfile):    
            logger.info('Iteration number %d' % i)
            i = i + 1
            learn.update(y0, u, y1)
#            if i > 15:
#                break
            
    if outlearner == 'No':
        pass
    else:
        logger.info('Saving learning agent to %s' % outlearner)
        pickle.dump(learn, open(outlearner, 'wb'))
    
        
    logger.info('Commands: %s' % learn.command_list)
    learn.summarize(prefix=name)
    learn.analyze(prefix=name, folder='out/diffeo-analysis/')
    
    learn.diffeo_dump(dirname, name)
#    learn.show_diffeomorphisms()
    
