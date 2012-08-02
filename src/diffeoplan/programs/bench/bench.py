from diffeoplan.configuration.master import DiffeoplanConfig

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
    results['planning_results'] = planning_result
    return results

def compute_stats(id_algo, results):
    return {}

