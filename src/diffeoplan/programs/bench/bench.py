from . import Stats, contract, get_visualization_distances, logger
from diffeoplan.library import DiffeoSystem, TestCase
from reprep.report_utils import StoreResults
import itertools
import time
import warnings

__all__ = ['run_planning', 'run_planning_stats']


def run_planning(id_algo, id_tc, testcase, algo):
    '''
    
    :param config:
    :param id_algo:
    :param id_discdds:
    :param id_tc:
    :param algo: Already init()ed instance of DiffeoPlanningAlgo
    ''' 
    # run the planning
    y0 = testcase.y0
    y1 = testcase.y1
    
    # compute achievable precision using the model
    dds = algo.get_dds()
    metric = algo.metric_goal
    y1p = dds.predict(y0, testcase.true_plan)
    d = metric.distance(y1, y1p)
    precision = d * 1.01
    
    # compute a bound that we want on the visibility
    warnings.warn('Remember to make a parameter of the test case whether we want'
                  ' to preserve a minimum visibility ratio.')
    should_preserve = 'dcl' in id_tc
    if should_preserve:
        action = dds.plan2action(testcase.true_plan)
        min_visibility = action.get_diffeo2d_forward().get_visibility() / 1.01
    else:
        min_visibility = 0
    
    logger.info('Using model and algo metric, the reachable distance is %s. '
                'Treshold: %s' % (d, precision))
    
    
    # TODO: add computation time
    t0 = time.clock()
    # Run the planning
#    try:
    planning_result = algo.plan(y0, y1, precision=precision,
                                min_visibility=min_visibility)
#    except:
#        try:
#            logger.info(algo.start_tree.memoize_cache.summary())
#            logger.info(algo.goal_tree.memoize_cache.summary())
#            logger.info(algo.connector.memoize_cache.summary())
#        except:
#            pass
#        raise
    
    cache_stats = {
     'start_tree': algo.start_tree.get_cache_stats(),
     'goal_tree': algo.goal_tree.get_cache_stats(),
     'connector': algo.connector.get_cache_stats(),
    }
    
    algo.start_tree.clear_cache()
    algo.goal_tree.clear_cache()
    algo.connector.clear_cache()
    
    plan_time = time.clock() - t0
    
    results = {}
    results['id_tc'] = id_tc
    results['tc'] = testcase
    results['id_discdds'] = testcase.id_discdds
    results['id_algo'] = id_algo
    results['algo'] = algo
    results['result'] = planning_result
    results['plan_time'] = plan_time
    results['cache_stats'] = cache_stats
    
    return results


@contract(results=dict, discdds=DiffeoSystem, testcase=TestCase)
def run_planning_stats(results, discdds, testcase):
    '''
        Compute statistics for the result of planning.
    
        :param results: output of run_planning.
    '''
    result = results['result']
#    id_discdds = results['id_discdds']
#    id_tc = results['id_tc'] 
#    tc = config.testcases.instance(id_tc)
    # this is the planned plan
    plan = result.plan
    # reinstance system and test case
#    discdds = config.discdds.instance(id_discdds)
    idiscdds = discdds.inverse()
    # predict result according to plan
    y0 = testcase.y0
    y1 = testcase.y1

    def compute_all_distances(y0, y1):
        x = [(k, d.distance(y0, y1)) for k, d in alld.items()]
        return dict(x)

    images = results['images'] = {}
    images['y0'] = y0
    images['y1'] = y1         
        
    if plan is not None:
        images['py0'] = discdds.predict(y0, plan)
        iplan = tuple(reversed(plan))
        images['ipy1'] = idiscdds.predict(y1, iplan)
    
    if testcase.true_plan is not None:
        images['ty0'] = discdds.predict(y0, testcase.true_plan)
        itrue_plan = tuple(reversed(testcase.true_plan))
        images['ity1'] = idiscdds.predict(y1, itrue_plan)
    
    distances = results['distances'] = StoreResults()
    for i1, i2 in itertools.combinations(Stats.get_images(), 2):
        if i1 == i2:
            continue
        
        if not i1 in images or not i2 in images:
            continue
        
        alld = get_visualization_distances()
        for d in alld:
            dvalue = alld[d]['distance'].distance(images[i1], images[i2])
            distances[dict(i1=i1, i2=i2, d=d)] = dvalue 
        
    return results
