from . import  np, logger
from collections import namedtuple

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
        symbol = f.__name__  
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
    """ $N^{\\text{S}}_{\\text{closed}}$ := Number of closed nodes in start tree. """  
    return stats['algo'].start_tree.get_num_closed()

@add_statistics
def num_start_open(stats): 
    """ $N^{\\text{S}}_{\\text{open}}$ := Number of open nodes in start tree. """
    return stats['algo'].start_tree.get_num_open()

@add_statistics
def num_start_created(stats): 
    """ $N^{\\text{S}}_{\\text{created}}$ := Number of created nodes in start tree. """
    return stats['algo'].start_tree.get_num_created()

@add_statistics
def num_start_redundant(stats): 
    """ $N^{\\text{S}}_{\\text{redun}}$ := Number of nodes created but already found"""
    return stats['algo'].start_tree.get_num_created_but_redundant()

@add_statistics
def num_start_collapsed(stats): 
    """ $N^{\\text{S}}_{\\text{collap}}$ := Number of nodes created but found equivalent"""
    return stats['algo'].start_tree.get_num_created_but_collapsed()

@add_statistics
def num_goal_closed(stats): 
    """ $N^{\\text{G}}_{\\text{closed}}$ := Number of closed nodes in start tree. """  
    return stats['algo'].goal_tree.get_num_closed()

@add_statistics
def num_goal_open(stats): 
    """ $N^{\\text{G}}_{\\text{open}}$ := Number of open nodes in start tree. """
    return stats['algo'].goal_tree.get_num_open()

@add_statistics
def num_goal_created(stats): 
    """ $N^{\\text{G}}_{\\text{created}}$ := Number of created nodes in start tree. """
    return stats['algo'].goal_tree.get_num_created()

@add_statistics
def num_goal_redundant(stats):
    """ $N^{\\text{G}}_{\\text{redun}}$ := Number of nodes created but already found""" 
    return stats['algo'].goal_tree.get_num_created_but_redundant()

@add_statistics
def num_goal_collapsed(stats): 
    """ $N^{\\text{G}}_{\\text{collap}}$ := Number of nodes created but found equivalent"""
    return stats['algo'].goal_tree.get_num_created_but_collapsed()

# TODO: finish statistics
@add_statistics
def dist_values_L1_old(s):
    if not 'dist_y0_y1p' in s: 
        return np.nan
    return s['dist_y0_y1p']['values_L1']

@add_statistics
def dist_values_L2_old(s):
    if not 'dist_y0_y1p' in s: 
        return np.nan
    return s['dist_y0_y1p']['values_L2']

@add_statistics
def dist_values_L1(s):
    """ $d_{L_1}(y_0, \hat{p} \cdot y_1)$ := prediction """
    if not 'dist_y1_y1p' in s: 
        return np.nan
    return s['dist_y1_y1p']['values_L1']

@add_statistics
def dist_values_L2(s):
    """ $d_{L_2}(y_0, \hat{p} \cdot y_1)$ := prediction """
    if not 'dist_y1_y1p' in s: 
        return np.nan
    return s['dist_y1_y1p']['values_L2']
