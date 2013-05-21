from . import logger, contract
from diffeoplan.configuration import get_current_config
from diffeoplan.library import (DistanceNormWeighted, DistanceNorm, plan_friendly)
from itertools import chain
from reprep.report_utils import (FunctionWithDescription, WithDescription,
    symbol_desc_from_docstring, symbol_desc_from_string)
from reprep.utils import frozendict2
import itertools
from diffeo2dds.model.plan_utils import plan_friendly_tex

class Statistic(FunctionWithDescription):
    pass

class Stats:
    statistics = {}
    
    tables_for_single_sample = {}
    tables_for_multi_sample = {}
    
    images = {}
    distances = {}
    misc_descriptions = {}
    
    @staticmethod
    @contract(returns='dict(str:WithDescription)')
    def all_descriptions():
        """ Returns descriptions for all objects defined here. """
        s = dict()
        items = chain(Stats.statistics.items())
        
        for k, v in items:
            s[k] = WithDescription(name=v.get_name(),
                                   desc=v.get_desc(),
                                   symbol=v.get_symbol())
            
        
        config = get_current_config()
        
        for objspec in config.specs.values():
            if not objspec:
                logger.warning('No %s defined? Just wondering...' % 
                               objspec.name)
            for k, v in objspec.items():
                symbol, desc = symbol_desc_from_string(v['desc'])
                if desc is None:
                    logger.warning('No description entry %r.' % k)
                s[k] = WithDescription(name=k, desc=desc, symbol=symbol)
                # print k, s[k]
                
        s.update(Stats.misc_descriptions)            
        return s

    @staticmethod
    def describe(name, desc, symbol):
        s = WithDescription(name=name, desc=desc, symbol=symbol)
        Stats.misc_descriptions[name] = s
        
    @staticmethod
    def describe_image(name, desc, symbol):
        s = WithDescription(name=name, desc=desc, symbol=symbol)
        Stats.images[name] = s
        
    @staticmethod
    def get_images():
        return Stats.images
     
    
Stats.describe_image('y0', 'Start image', 'y_0')
Stats.describe_image('y1', 'Goal image', 'y_1')
Stats.describe_image('py0', 'Predicted start using plan', 'p\cdot y_0')
Stats.describe_image('ipy1', 'Predicted goal using plan inverse', 'p^{-1}\cdot y_1')
Stats.describe_image('ty0', 'Predicted start using true plan', 'p_{\circ}\cdot y_0')
Stats.describe_image('ty0', 'Predicted start using true plan', 'p_{\circ}\cdot y_0')
Stats.describe_image('ity1', 'Predicted goal using true plan inverse', 'p_{\circ}^{-1}\cdot y_0')

        

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
    visualization_images = Stats.get_images()
    desc1 = visualization_images[i1].get_desc()
    desc2 = visualization_images[i2].get_desc()
    s1 = visualization_images[i1].get_symbol()
    s2 = visualization_images[i2].get_symbol()
    symbol = 'd_{%s}(%s,%s)' % (sd, s1, s2)
    desc = 'Distance %s between %s and %s' % (d, desc1, desc2)
    return dict(symbol=symbol, desc=desc)


def add_statistics(f):
    symbol, desc = symbol_desc_from_docstring(f)    
    Stats.statistics[f.__name__] = Statistic(name=f.__name__,
                                             symbol=symbol, desc=desc,
                                             function=f)
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
def plan_string_tex(stats):
    """ p := Plan """
    plan = stats['result'].plan 
    if plan is None:
        return None
    else:
        return '$%s$' % plan_friendly_tex(plan)


@add_statistics
def true_plan_string(stats):
    """ p_{\circ} := True plan """
    true_plan = stats['tc'].true_plan 
    if true_plan is None:
        return None
    else:
        return plan_friendly(true_plan)

@add_statistics
def true_plan_string_tex(stats):
    """ p_{\circ} := True plan """
    true_plan = stats['tc'].true_plan 
    if true_plan is None:
        return None
    else:
        return '$%s$' % plan_friendly_tex(true_plan)
    
    
@add_statistics
def plan_length(stats):
    """ |p| := Plan length """
    if plan_found(stats):
        return len(stats['result'].plan)
    else:
        return None
    
# TODO: add same plan
# @add_statistics
# def plan_same(stats):
#    """ |p| := Plan length """
#    if plan_found(stats):
#        return len(stats['result'].plan)
#    else:
#        return np.nan

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
        key = frozendict2(i1=i1, i2=i2, d=d)
        if not key in distances:
            return None
        else:
            return distances[key]
    f2.__name__ = 'd_%s_%s_%s' % (d, i1, i2)
    f2.__doc__ = '%s := %s' % (desc['symbol'], desc['desc'])
    return f2

for i1, i2 in itertools.combinations(Stats.get_images(), 2):
    for d in get_visualization_distances():        
        add_statistics(makestat(i1, i2, d))
 

Stats.tables_for_single_sample = {
    'all': list(Stats.statistics.keys()),
    'graph': [
        'plan_found',
        'plan_string_tex',
        # 'd_L2_py0_y1//f3',
        'd_L2w_py0_y1//f3',
        'plan_time//f2',
        'num_states_evaluated//d',
        'num_closed//d',
        'num_open//d',
        'num_created//d',
        'num_redundant//d',
        'num_collapsed//d',
    ],
    'distances': [
        'plan_found',
        'plan_string_tex',
        'true_plan_string_tex',
        # 'd_L2_py0_y1//f3',
        'd_L2w_py0_y1//f3',
        'goal_threshold//f3',
        'plan_time//f2',
        'num_states_evaluated//d',
        'num_closed//d',
        'num_open//d',
        'num_created//d',
        'num_redundant//d',
        'num_collapsed//d',
    ],
    'graph_details': [
        'plan_found',
        'num_states_evaluated//d',
        'num_closed//d',
        'num_open//d',
        'num_created//d',
        'num_redundant//d',
        'num_collapsed//d',
        'num_start_closed//d',
        'num_start_open//d',
        'num_start_created//d',
        'num_start_redundant//d',
        'num_start_collapsed//d',
        'num_goal_closed//d',
        'num_goal_open//d',
        'num_goal_created//d',
        'num_goal_redundant//d',
        'num_goal_collapsed//d',
    ]
}

Stats.tables_for_multi_sample = {                   
     'graph_details_many': [
        'plan_found/min_mean_max/min_mean_max_s',
        'num_states_evaluated/min_mean_max/min_mean_max_s',
        'num_closed/min_mean_max/min_mean_max_s',
        'num_open/min_mean_max/min_mean_max_s',
        'num_created/min_mean_max/min_mean_max_s',
        'num_redundant/min_mean_max/min_mean_max_s',
        'num_collapsed/min_mean_max/min_mean_max_s',
        'num_start_closed/min_mean_max/min_mean_max_s',
        'num_start_open/min_mean_max/min_mean_max_s',
        'num_start_created/min_mean_max/min_mean_max_s',
        'num_start_redundant/min_mean_max/min_mean_max_s',
        'num_start_collapsed/min_mean_max/min_mean_max_s',
        'num_goal_closed/min_mean_max/min_mean_max_s',
        'num_goal_open/min_mean_max/min_mean_max_s',
        'num_goal_created/min_mean_max/min_mean_max_s',
        'num_goal_redundant/min_mean_max/min_mean_max_s',
        'num_goal_collapsed/min_mean_max/min_mean_max_s',
    ],
    'graph_details_mean': [
        'plan_found/mean',
        'num_states_evaluated/mean',
        'num_closed/mean',
        'num_open/mean',
        'num_created/mean',
        'num_redundant/mean',
        'num_collapsed/mean',
        'num_start_closed/mean',
        'num_start_open/mean',
        'num_start_created/mean',
        'num_start_redundant/mean',
        'num_start_collapsed/mean',
        'num_goal_closed/mean',
        'num_goal_open/mean',
        'num_goal_created/mean',
        'num_goal_redundant/mean',
        'num_goal_collapsed/mean',
    ],
    'distances_mean': [
        'plan_found/num',
        'plan_found/mean/perc',
        'plan_length/mean/f1',
        'plan_time/mean/f1',
        'num_states_evaluated/mean/f1',
        'num_closed/mean/f1',
        'num_open/mean/f1',
        'num_created/mean/f1',
        'num_redundant/mean/f1',
        'num_collapsed/mean/f1',
    ],
    'tr1_resources': [
#        'plan_found/num',
#        'plan_found/mean/perc',
#        'plan_length/mean/f1',
        'plan_time/mean/f1',
        'num_states_evaluated/mean/f1',
#        'num_closed/mean/d',
#        'num_open/mean/d',
#        'num_created/mean/d',
#        'num_redundant/mean/d',
#        'num_collapsed/mean/d',
    ],
                                 
                                 

}
