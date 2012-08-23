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


@contract(plan='seq[N]')
def plan_friendly(plan):
    """ Returns a friendly string for the plan. """
    if len(plan) == 0:
        return "<>"
    
    names = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'l']
    
    def cmd(u, n):
        s = names[u]
        if n > 1:
            s += '%s' % n
        return s 
            
    s = "".join([cmd(u, n) for (u, n) in plan_group(plan)])
    return "%s" % s

@contract(plan='seq[N]')
def plan_group(plan):
    """ 
        Count repeated actions 
        01103222 ->  (0,1) (1,2) (0,1) ...
    """
    plan = list(plan)
    while plan:
        first = plan.pop(0)
        n = 1
        while plan:
            if plan[0] == first:
                n += 1
                plan.pop(0)
            else:
                break
        yield (first, n)
