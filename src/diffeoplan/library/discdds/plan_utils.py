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
def plan_friendly(plan, names=None, empty='<>', use_exponent=False):
    """ Returns a friendly string for the plan. """
    if len(plan) == 0:
        return empty
    
    if names is None:
        names = [chr(ord('a') + x) for x in range(26)]
        #names = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'l']
    
    def cmd(u, n):
        if use_exponent:
            if n > 1:
                return '(%s)^%s' % (names[u], n)
            else:
                return names[u]
        else:
            return names[u] * n
        return s 
            
    s = "".join([cmd(u, n) for (u, n) in plan_group(plan)])
    return  s

@contract(plan='seq[N]')
def plan_friendly_tex(plan):
    """ Returns a TeX string for the plan. """
    if len(plan) == 0:
        return "\emptyset"
    
    names = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'l']
    
    def cmd(u, n):
        s = names[u]
        if n > 1:
            s += '^%s' % n
        return s 
            
    s = "".join([cmd(u, n) for (u, n) in plan_group(plan)])
    return s

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


@contract(plan='seq[N]', returns='seq[N+1](seq[<=N])')
def plan_steps(plan):
    """
        From a given plan (a,b,c,...), returns the 
        sequence of sub-plans [(),(a,),(a,b),...],
        including the empty plan at the beginning.
    """
    sub_plan = lambda i: plan[:i]
    return map(sub_plan, range(len(plan) + 1))
    
    

