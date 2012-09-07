from . import get_visualization_distances, logger
from reprep.report_utils import StoreResults
import itertools
import time
from diffeoplan.programs.bench.statistics import Stats
from contracts import contract
from diffeoplan.library.discdds.diffeo_system import DiffeoSystem
from ctypes.test.test_delattr import TestCase

__all__ = ['run_planning', 'run_planning_stats']


def run_planning(id_algo, id_tc, testcase, algo):
    '''
    
    :param config:
    :param id_algo:
    :param id_discdds:
    :param id_tc:
    :param algo: Already init()ed instance of DiffeoPlanningAlgo
    '''
    # load the test case 
#    testcase = config.testcases.instance(id_tc)
    
    # run the planning
    y0 = testcase.y0
    y1 = testcase.y1
    
    # compute achievable precision using the model
    dds = algo.get_dds()
    metric = algo.metric_goal
    y1p = dds.predict(y0, testcase.true_plan)
    d = metric.distance(y1, y1p)
    precision = d * 1.01
    logger.info('Using model and algo metric, the reachable distance is %s. '
                'Treshold: %s' % (d, precision))
    
    
    # TODO: add computation time
    t0 = time.clock()
    # Run the planning
    planning_result = algo.plan(y0, y1, precision)
    
    plan_time = time.clock() - t0
    
    results = {}
    results['id_tc'] = id_tc
    results['tc'] = testcase
    results['id_discdds'] = testcase.id_discdds
    results['id_algo'] = id_algo
    results['algo'] = algo
    results['result'] = planning_result
    results['plan_time'] = plan_time
    
    
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
