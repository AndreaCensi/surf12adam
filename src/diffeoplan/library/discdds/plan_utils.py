import itertools
from contracts import contract

@contract(ncmd='M,>=1', maxsteps='K',
          returns='seq(seq[<=K](int,>=0,<M))')
def plans_of_max_length(ncmd, maxsteps):
    """ Returns all plans of the given maximum length. """
    plans = []
    for M in range(maxsteps + 1):
        plans_M = itertools.product(range(ncmd), repeat=M) 
        plans.extend(plans_M)
    return plans
