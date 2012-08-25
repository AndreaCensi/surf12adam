from diffeoplan.library import UncertainImage
import time
from diffeoplan.library.images.distance.distance_norm_weighted import DistanceNormWeighted
from diffeoplan.library.images.distance.distance_norm import DistanceNorm
import itertools
from reprep.report_utils.store_results import StoreResults
 

def run_planning(config, id_algo, id_tc, algo):
    '''
    
    :param config:
    :param id_algo:
    :param id_discdds:
    :param id_tc:
    :param algo: Already init()ed instance of DiffeoPlanningAlgo
    '''
    # load the test case 
    testcase = config.testcases.instance(id_tc)
    
    # run the planning
    y0 = testcase.y0
    y1 = testcase.y1
    
    # TODO: add computation time
    t0 = time.clock()
    # Run the planning
    planning_result = algo.plan(y0, y1)
    plan_time = time.clock() - t0
    
    results = {}
    results['id_tc'] = id_tc
    results['tc'] = testcase
    results['id_discdds'] = testcase.id_discdds
    results['id_algo'] = id_algo
    results['algo'] = algo
    results['result'] = planning_result
    #results['init_time'] = init_time
    results['plan_time'] = plan_time
    return results

def run_planning_stats(config, results):
    '''
        Compute statistics for the result of planning.
    
        :param results: output of run_planning.
    '''
    result = results['result']
    id_discdds = results['id_discdds']
    id_tc = results['id_tc'] 
#    tc = results['tc']
    tc = config.testcases.instance(id_tc)
    # this is the planned plan
    plan = result.plan
    # reinstance system and test case
    testcase = config.testcases.instance(id_tc)
    discdds = config.discdds.instance(id_discdds)
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
    images['py0'] = discdds.predict(y0, plan)
    iplan = tuple(reversed(plan))
    images['ipy1'] = idiscdds.predict(y1, iplan)
    
    if tc.true_plan is not None:
        images['ty0'] = discdds.predict(y0, tc.true_plan)
        itrue_plan = tuple(reversed(tc.true_plan))
        images['ity1'] = idiscdds.predict(y1, itrue_plan)
    
    distances = results['distances'] = StoreResults()
    for i1, i2 in itertools.combinations(visualization_images, 2):
        if i1 == i2:
            continue
        
        if not i1 in images or not i2 in images:
            continue
        
        alld = get_visualization_distances()
        for d in alld:
            dvalue = alld[d]['distance'].distance(images[i1], images[i2])
            distances[dict(i1=i1, i2=i2, d=d)] = dvalue 
        
    return results

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

visualization_images = {
    'y0': ('Start image', 'y_0'),
    'y1': ('Goal image', 'y_1'),
    'py0': ('Predicted start using plan', 'p\cdot y_1'),
    'ipy1': ('Predicted goal using plan inverse', 'p^{-1}\cdot y_1'),
    'ty0': ('Predicted start using true plan', 'p\cdot y_1'),
    'ity1': ('Predicted goal using true plan inverse', 'p^{-1}\cdot y_1'),
}
        

def get_visualization_distances():
    distances = {}
    distances['L2'] = dict(distance=DistanceNorm(2), symbol='L_2')
    distances['L1'] = dict(distance=DistanceNorm(1), symbol='L_1')
    distances['L1w'] = dict(distance=DistanceNormWeighted(1), symbol='L_1^w')
    distances['L2w'] = dict(distance=DistanceNormWeighted(2), symbol='L_2^w')
    return distances
