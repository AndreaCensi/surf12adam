from . import logger
from bootstrapping_olympics.utils.in_a_while import InAWhile
from diffeoplan.library.images.uncertain_image import UncertainImage
from diffeoplan.utils.script_utils import UserError
from learner.diffeo_learner import DiffeoLearner
from optparse import OptionParser
from procgraph_pil.imread import imread
from reprep import Report
import os
import pickle
import urllib
from diffeoplan.library.logs.rosbag.bag_reader import read_bag


def diffeo_learner_main():
    usage = "usage: %prog   -p outputpath -n ddsname -s [W,H] -a [rx,ry] bag1 bag2 .."
    parser = OptionParser(usage=usage, version="%prog 1.0") 
    parser.add_option("-p", "--path", default=None,
                      help="Path to output files")
    parser.add_option("-n", "--name", default='camdds',
                      help="Output dds system name")
    
    parser.add_option("-s", "--size", default=None, # default='[160,120]',
                      help="Image size WxH")
    parser.add_option("-a", "--area", default=None, #default='[6,6]',
                      help="Size of search area")
    
    parser.add_option("-r", "--ratio", default=0.1, type='float',
                      help="Search area ratio.")
    
    parser.add_option("-l", "--learner", default='No',
                      help="Reuse learner")
    parser.add_option("-o", "--outlearner", default='No',
                      help="Save the learner")
    options, args = parser.parse_args()
    
    if options.path is None:
        dirname = os.path.dirname(options.input)
        logger.info('No path given; using %s' % dirname)
    else:
        dirname = options.path
         
    name = options.name 

    if options.size or options.area:
        msg = 'Using deprecated options. Start using --ratio.'
        raise UserError(msg)
        


    learner = options.learner
    outlearner = options.outlearner

    if learner == 'No':
        ratios = (options.ratio, options.ratio)
        
        learn = DiffeoLearner(True, {'max_displ': ratios, 'inference_method': 'sim'})
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
    
    in_a_while = InAWhile(30)
    
    for bagfile in args:
        i = 0
        for y0, u, y1 in read_bag(bagfile):    
            logger.info('Iteration number %d' % i)
            learn.update(y0, u, y1)
            i += 1
            
            if in_a_while.its_time():
                display_current_results(learn, name, dirname, i)
                in_a_while.reset()
            
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
    
def display_current_results(learner, name, dirname, iteration):
    dds = learner.summarize(prefix=name)
    r = Report('%s-it%s' % (name, iteration))
    r.text('summary', 'Iteration: %s' % iteration)
    base = '%s-current.html' % (name)
    filename = os.path.join(dirname, 'iterations', base)
    # TODO: add file
    f = '/opt/EPD/7.3/lib/python2.7/site-packages/PIL/Images/lena.jpg'
    lena = imread(f)
    image = UncertainImage(lena)
    dds.display(r, image)
    logger.info('Writing to %r.' % filename) 
    r.to_html(filename)
    
    
