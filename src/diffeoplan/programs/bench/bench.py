from diffeoplan.library import UncertainImage
import time
 

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
    
    # this is the planned plan
    plan = result.plan
    # reinstance system and test case
    testcase = config.testcases.instance(id_tc)
    discdds = config.discdds.instance(id_discdds)
    # predict result according to plan
    y0 = testcase.y0
    y1 = testcase.y1
    
    if plan is not None:
        y1plan = discdds.predict(y0, plan)
        # computes all distances as specified in this functions
        # This compares y0 with plan * y0 (wrong)
        # TODO: use the distances that we know
        results['dist_y0_y1p'] = UncertainImage.compute_all_distances(y0, y1plan) 
        results['dist_y1_y1p'] = UncertainImage.compute_all_distances(y1, y1plan)
    
    return results


