from . import logger
from .. import declare_command
from collections import deque
from contracts import contract
from diffeoplan.programs.maketestcases.maketest import discdds_make_test_cases
from diffeoplan.utils.script_utils import UserError
from learner.programs.diffeo_learner.bag_reader import read_bag, LogItem
import numpy as np
import os
from contracts.interface import describe_value
from diffeoplan.library.images.test_case import TestCase
from diffeoplan.library.images.uncertain_image import UncertainImage



@declare_command('logcases',
                 'logcases --log <logname (bag)> -n <number> -d <delay>')
def logcases_main(config, parser): #@UnusedVariable
    """ 
        Creates test cases from log files. 
    
        Example uses:
        
            logcases --log mycam-0.processed.bag -n 10 --delay 1 --dds mycam
                      
    
    """
    
    parser.add_option('-n', help="Number of test cases.", default=1, type='int')
    parser.add_option('-d', '--delay', help="Delay between data (>=1)", default=1, type='int')
    parser.add_option('-l', '--log', help='Log filename')
    parser.add_option('-o', '--output', default='out/log-testcases',
                      help='Output directory')
#    parser.add_option('-p', '--pattern', default=default_tc_pattern,
#                       help='Pattern for the name of test cases. Should contain '
#                            'a "%d" that will be substituted with the ID.')
    parser.add_option('--dds',
                       help='Name of DDS that this test case will be associated to.')
    
    options = parser.parse_options()

    if options.log is None:
        msg = 'Please specify the filename using the --log option.'
        raise UserError(msg)

    if options.dds is None:
        msg = 'Please specify the DDS name using the --dds option.'
        raise UserError(msg)
        # check it exists?
        
    id_discdds = options.dds
    discdds = config.discdds.instance(id_discdds)
    
    
    id_tc_pattern = 'tc_%s_d%d_' % (id_discdds, options.delay) + '%03d'    
    logger.info('Creating test cases with pattern %r' % id_tc_pattern)
    
    
    bagfile = options.log 
    if not os.path.exists(bagfile):
        msg = 'Log %r does not exist.' % bagfile
        raise UserError(msg)

    if not '.processed.bag' in bagfile:
        msg = 'I would expect you use this program on a .processed.bag file.'
        logger.warning(msg)
    

    cases = logcases(bagfile, n=options.n, delta=options.delay,
                     id_discdds=id_discdds, id_tc_pattern=id_tc_pattern,
                     discdds=discdds)

    outdir = os.path.join(options.output, id_discdds)
    
    if not os.path.exists(outdir):
        os.makedirs(outdir)
        
        
    for tc in cases:
        tc.save(outdir)    
    
    

    
@contract(delta='int,>=1', n='int,>=1')
def logcases(bagfile, n, delta, id_tc_pattern, id_discdds, discdds):
    
    blocks = read_sequences_delta(bagfile, delta)
    miniplans = (make_plan(x) for x in blocks)
    sampled = reservoir_sample(miniplans, N=n)
    
    for i, m in enumerate(sampled):
        id_tc = id_tc_pattern % i
        
        I0 = UncertainImage(m.y0)
        I1 = UncertainImage(m.y1)
        plan = discdds.commands_to_indices(m.u)
        tc = TestCase(id_tc=id_tc, id_discdds=id_discdds,
                      y0=I0, y1=I1, true_plan=plan)
        yield tc

import random

def reservoir_sample(it, N):
    """ Reservoir sample. 
        See http://en.wikipedia.org/wiki/Reservoir_sampling 
        
        print reservoir_sample(range(1000), N=10)
    """
    reservoir = []
    for i, x in enumerate(it):
        if i < N:
            reservoir.append(x)
        else:
    
            # Randomly replace elements in the reservoir
            # with a decreasing probability.              
            # Choose an integer between 0 and index (inclusive)                
            r = random.randrange(i + 1)                
            if r < N:                        
                reservoir[r] = x
    return reservoir
    
    
@contract(returns='tuple( (array,shape(x)), list(array[K]), (array,shape(x)) )')
def make_plan(sequence):
    """ Takes a seuqences of lists of LogItems,
        converts them into a plan. """
    y0 = sequence[0].y0
    # last image
    y1 = sequence[-1].y1
    # intermediate commands (ground truth plan)
    u = [np.array(m_i.u) for m_i in sequence[:-1]]
    return LogItem(y0=y0, u=u, y1=y1)

@contract(delta='int,>=1')
def read_sequences_delta(bagfile, delta):
    """ Yields a sequence of lists of delta+ 1 log records """
    maxlen = delta + 1
    q = deque(maxlen=maxlen)
    for x in read_bag(bagfile):
        q.append(x)
        if len(q) == maxlen:
            yield list(q)
        
#    
#def log_length(bagfile):
#    """ Returns the length of the stream """
#    # Very inefficient
#    count = 0
#    for _ in read_bag(bagfile):
#        count += 1 
#    return count

    
