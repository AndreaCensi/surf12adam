from collections import deque
from contracts import contract
from diffeoplan.library import TestCase, UncertainImage
from learner.programs.diffeo_learner.bag_reader import read_bag, LogItem
import numpy as np
import random
    
    
@contract(delta='int,>=1', n='int,>=1')
def make_logcases(bagfile, n, delta, id_tc_pattern, id_discdds, discdds):
    
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
