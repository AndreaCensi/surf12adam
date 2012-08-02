from diffeoplan.configuration import DiffeoplanConfig
from diffeoplan.library import UncertainImage

def run_planning(id_algo, id_tc):
    config = DiffeoplanConfig
    # load the test case 
    testcase = config.testcases.instance(id_tc)
    # load the dynamics, as specified in the test case
    id_discdds = testcase.id_discdds
    discdds = config.discdds.instance(id_discdds)
    # instance the algorithm
    algo = config.algos.instance(id_algo)
    # initialize with the dynamics
    # TODO: add computation time
    algo.init(discdds)
    
    # run the planning
    y0 = testcase.y0
    y1 = testcase.y1
    # TODO: add computation time
    planning_result = algo.plan(y0, y1)
    
    results = {}
    results['id_tc'] = id_tc
    results['id_discdds'] = id_discdds
    results['id_algo'] = id_algo
    results['result'] = planning_result
    return results

def run_planning_stats(results):
    '''
        Compute statistics for the result of planning.
    
        :param results: output of run_planning.
    '''
    result = results['result']
    id_discdds = results['id_discdds']
    id_tc = results['id_tc'] 
    
    # this is the planned plan
    plan = result.plan
    # reinstance system and test case
    config = DiffeoplanConfig
    testcase = config.testcases.instance(id_tc)
    discdds = config.discdds.instance(id_discdds)
    # predict result according to plan
    y0 = testcase.y0
    y1plan = discdds.predict(y0, plan)
    # compute the 
    results['dist_y0_y1p'] = UncertainImage.compute_all_distances(y0, y1plan) 
    return results

