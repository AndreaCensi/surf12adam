from . import np, logger
from collections import namedtuple
from diffeoplan.library import DistanceNormWeighted, DistanceNorm, plan_friendly
from reprep.report_utils import frozendict
from scipy.stats.stats import nanstd, nanmean
import itertools

Statistic = namedtuple('Statistic', 'name function symbol desc')

class Stats:
    statistics = {}
    reductions = {}
    tables = {}
    statstables = {}
    

Stats.reductions = {}
Stats.reductions['min'] = np.nanmin
Stats.reductions['max'] = np.nanmax
Stats.reductions['mean'] = nanmean
Stats.reductions['stddev'] = nanstd 
    

visualization_images = {
    'y0': ('Start image', 'y_0'),
    'y1': ('Goal image', 'y_1'),
    'py0': ('Predicted start using plan', 'p\cdot y_0'),
    'ipy1': ('Predicted goal using plan inverse', 'p^{-1}\cdot y_1'),
    'ty0': ('Predicted start using true plan', 't\cdot y_0'),
    'ity1': ('Predicted goal using true plan inverse', 't^{-1}\cdot y_0'),
}
        

def get_visualization_distances():
    distances = {}
    distances['L2'] = dict(distance=DistanceNorm(2), symbol='L_2')
    distances['L1'] = dict(distance=DistanceNorm(1), symbol='L_1')
    distances['L1w'] = dict(distance=DistanceNormWeighted(1), symbol='L_1^w')
    distances['L2w'] = dict(distance=DistanceNormWeighted(2), symbol='L_2^w')
    return distances



def get_combination_desc(i1, i2, d):
    alld = get_visualization_distances()
    sd = alld[d]['symbol']
    desc1 = visualization_images[i1][0]
    desc2 = visualization_images[i2][0]
    s1 = visualization_images[i1][1]
    s2 = visualization_images[i2][1]
    symbol = 'd_{%s}(%s,%s)' % (sd, s1, s2)
    desc = 'Distance %s between %s and %s' % (d, desc1, desc2)
    return dict(symbol=symbol, desc=desc)


def add_statistics(f):
    doc = f.__doc__
    if doc is None:
        logger.warning('No description for %s' % f)
        doc = "(missing description)"
    doc = doc.strip()
    if ':=' in doc:
        tokens = doc.split(':=')
        symbol = tokens[0]
        desc = "".join(tokens[1:])
    else:
        symbol = '\\text{%s}' % f.__name__  
        desc = doc
        
    Stats.statistics[f.__name__] = Statistic(name=f.__name__, function=f,
                                             symbol=symbol, desc=desc)
    return f
 
@add_statistics
def plan_time(stats):
    """ T := planning time """
    return stats['plan_time']

@add_statistics
def plan_found(stats):
    """ \\text{succ} := Success """
    return stats['result'].plan is not None

@add_statistics
def plan_string(stats):
    """ p := Plan """
    plan = stats['result'].plan 
    if plan is None:
        return None
    else:
        return plan_friendly(plan)
    
@add_statistics
def plan_length(stats):
    """ |p| := Plan length """
    if plan_found(stats):
        return len(stats['result'].plan)
    else:
        return np.nan

@add_statistics
def goal_threshold(stats): 
    """ d_{\\text{goal}} := Threshold to goal. """  
    return stats['algo'].metric_goal_threshold
    
@add_statistics
def num_start_closed(stats): 
    """ N^{\\text{S}}_{\\text{closed}} := Number of closed nodes in start tree. """  
    return stats['algo'].start_tree.get_num_closed()

@add_statistics
def num_start_open(stats): 
    """ N^{\\text{S}}_{\\text{open}} := Number of open nodes in start tree. """
    return stats['algo'].start_tree.get_num_open()

@add_statistics
def num_start_created(stats): 
    """ N^{\\text{S}}_{\\text{created}} := Number of created nodes in start tree. """
    return stats['algo'].start_tree.get_num_created()

@add_statistics
def num_start_redundant(stats): 
    """ N^{\\text{S}}_{\\text{redun}} := Number of nodes created but already found"""
    return stats['algo'].start_tree.get_num_created_but_redundant()

@add_statistics
def num_start_collapsed(stats): 
    """ N^{\\text{S}}_{\\text{collap}} := Number of nodes created but found equivalent"""
    return stats['algo'].start_tree.get_num_created_but_collapsed()

@add_statistics
def num_goal_closed(stats): 
    """ N^{\\text{G}}_{\\text{closed}} := Number of closed nodes in start tree. """  
    return stats['algo'].goal_tree.get_num_closed()

@add_statistics
def num_goal_open(stats): 
    """ N^{\\text{G}}_{\\text{open}} := Number of open nodes in start tree. """
    return stats['algo'].goal_tree.get_num_open()

@add_statistics
def num_goal_created(stats): 
    """ N^{\\text{G}}_{\\text{created}} := Number of created nodes in start tree. """
    return stats['algo'].goal_tree.get_num_created()

@add_statistics
def num_goal_redundant(stats):
    """ N^{\\text{G}}_{\\text{redun}} := Number of nodes created but already found""" 
    return stats['algo'].goal_tree.get_num_created_but_redundant()

@add_statistics
def num_goal_collapsed(stats): 
    """ N^{\\text{G}}_{\\text{collap}} := Number of nodes created but found equivalent"""
    return stats['algo'].goal_tree.get_num_created_but_collapsed()

@add_statistics
def num_closed(stats): 
    """ N_{\\text{closed}} := Number of closed nodes. """  
    return num_start_closed(stats) + num_goal_closed(stats)

@add_statistics
def num_open(stats): 
    """ N_{\\text{open}} := Number of open nodes. """
    return num_start_open(stats) + num_goal_open(stats)

@add_statistics
def num_created(stats): 
    """ N_{\\text{created}} := Number of created nodes. """
    return num_start_created(stats) + num_goal_created(stats)

@add_statistics
def num_redundant(stats):
    """ N_{\\text{redun}} := Number of nodes created but already found""" 
    return num_start_redundant(stats) + num_goal_redundant(stats)

@add_statistics
def num_collapsed(stats): 
    """ N_{\\text{collap}} := Number of nodes created but found equivalent"""
    return num_start_collapsed(stats) + num_goal_collapsed(stats)

@add_statistics
def num_states_evaluated(stats):
    """ N_{\\text{eval}} := Number of nodes fully expanded. """
    return  num_created(stats) - num_redundant(stats)


distances = get_visualization_distances()

def makestat(i1, i2, d):
    desc = get_combination_desc(i1, i2, d)
    
    def f2(stats):
        distances = stats['distances']
        key = frozendict(i1=i1, i2=i2, d=d)
        if not key in distances:
            return None
        else:
            return distances[key]
    f2.__name__ = 'd_%s_%s_%s' % (d, i1, i2)
    f2.__doc__ = '%s := %s' % (desc['symbol'], desc['desc'])
    return f2

for i1, i2 in itertools.combinations(visualization_images, 2):
    for d in get_visualization_distances():        
        add_statistics(makestat(i1, i2, d))

def f(stats, s, r):
    reduce_ = Stats.reductions[r]
    map_ = Stats.statistics[s].function
    return reduce_([map_(x) for x in stats])

for a, b in itertools.product(Stats.statistics, Stats.reductions):
    def funcC(a, b):
        def func(stats):
            return f(stats, a, b)
        return func
    Stats.tables['%s-%s' % (a, b)] = funcC(a, b)


Stats.statstables = {
    'all': list(Stats.statistics.keys()),
    'graph': [
        'plan_found',
        'plan_string',
        'd_L2_py0_y1',
        'plan_time',
        'num_states_evaluated',
        'num_closed',
        'num_open',
        'num_created',
        'num_redundant',
        'num_collapsed',
    ],
    'distances': [
        'plan_found',
        'plan_string',
        'd_L2_py0_y1',
        'goal_threshold',
        'plan_time',
        'num_states_evaluated',
        'num_closed',
        'num_open',
        'num_created',
        'num_redundant',
        'num_collapsed',
    ],
    'graph_details': [
        'plan_found',
        'num_states_evaluated',
        'num_closed',
        'num_open',
        'num_created',
        'num_redundant',
        'num_collapsed',
        'num_start_closed',
        'num_start_open',
        'num_start_created',
        'num_start_redundant',
        'num_start_collapsed',
        'num_goal_closed',
        'num_goal_open',
        'num_goal_created',
        'num_goal_redundant',
        'num_goal_collapsed',
    ]
}
