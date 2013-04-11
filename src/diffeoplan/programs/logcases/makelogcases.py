from . import np, contract
from collections import deque
from diffeoplan.library import LogItem, TestCase, UncertainImage
from itertools import ifilter
    
    
@contract(delta='int,>=1', n='int,>=1')
def make_logcases(config, id_stream, n, delta, id_tc_pattern,
                  id_discdds, discdds,
                  seed):
    
    stream = config.streams.instance(id_stream)
    stream_data = stream.read_all()
    miniplans = iterate_testcases(stream_data, delta)
    sampled = reservoir_sample(miniplans, N=n, seed=seed)
    
    for i, m in enumerate(sampled):
        id_tc = id_tc_pattern % i
        I0 = UncertainImage(m.y0)
        I1 = UncertainImage(m.y1)
        assert len(m.u) == delta
        plan = discdds.commands_to_indices(m.u)
        tc = TestCase(id_tc=id_tc, id_discdds=id_discdds,
                      y0=I0, y1=I1, true_plan=plan)
        # TODO: add field desc
        # desc = 'Sampled from stream %s, i=%s for seed %s.' % (i, seed)
        yield tc


def iterate_testcases(sequence, delta):
    blocks = read_sequences_delta(sequence, delta)
    miniplans = (make_plan(x, simplify=True) for x in blocks)
    
    def accept_right_delta(c):
        return len(c.u) == delta
    
    filtered = ifilter(accept_right_delta, miniplans)
    return filtered
    

def reservoir_sample(sequence, N, seed):
    """ 
        Reservoir sample. 
        See http://en.wikipedia.org/wiki/Reservoir_sampling . 
        
        Try using: ::
            
            print reservoir_sample(range(1000), N=10)
    """
    rng = np.random.RandomState(seed)
    reservoir = []
    for i, x in enumerate(sequence):
        if i < N:
            reservoir.append(x)
        else:
            # Randomly replace elements in the reservoir
            # with a decreasing probability.              
            # Choose an integer between 0 and index (inclusive)                
            r = rng.randint(0, i + 1)                
            if r < N:                        
                reservoir[r] = x
    return reservoir
    
    
@contract(returns='tuple( (array,shape(x)), list(array[K]), (array,shape(x)) )')
def make_plan(sequence, simplify=True):
    """ Takes a sequence of lists of LogItems,
        converts them into a plan. """
    y0 = sequence[0].y0
    # last image
    y1 = sequence[-1].y1
    # intermediate commands (ground truth plan)
    u = [np.array(m_i.u) for m_i in sequence]
    # logger.info('Sequence of %d images' % len(sequence))
    # logger.info('Plan: %s' % str(u))
    if simplify:        
        u = simplify_plan(u)
        # logger.info('Plan simplified: %s' % str(u))

    return LogItem(y0=y0, u=u, y1=y1, x0=None)


def simplify_plan(plan0):
    """ 
        Simplifies the plan assuming that the dynamics is linear.
    """
    # First make it into tuples
    plan = map(tuple, plan0)
    different = set(plan)
    for d in different:
        dm = tuple(-np.array(d))
        if d in plan and dm in plan:
            plan.remove(d)
            plan.remove(dm)
    
    # logger.info('u: %s -> %s' % (plan0, plan))
    plan1 = map(np.array, plan)
    return plan1
    

@contract(delta='int,>=1')
def read_sequences_delta(sequence, delta):
    """ Yields a sequence of lists of delta log records """
    maxlen = delta 
    q = deque(maxlen=maxlen)
    for x in sequence:
        q.append(x)
        if len(q) == maxlen:
            yield list(q)
        
