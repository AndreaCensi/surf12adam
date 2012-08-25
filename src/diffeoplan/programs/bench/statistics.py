from . import  np, logger
from collections import namedtuple
from diffeoplan.programs.bench.bench import get_visualization_distances, \
    visualization_images, get_combination_desc
import itertools
from reprep.report_utils.store_results import frozendict

Statistic = namedtuple('Statistic', 'name function symbol desc')

class Stats:
    statistics = {}

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
    return stats['plan_time']

@add_statistics
def plan_found(stats):
    return stats['result'].plan is not None

@add_statistics
def plan_length(stats):
    if plan_found(stats):
        return len(stats['result'].plan)
    else:
        return np.nan
    
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

    
