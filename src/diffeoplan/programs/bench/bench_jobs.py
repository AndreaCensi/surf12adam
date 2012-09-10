from . import (contract, run_planning_stats, run_planning, results2stats_dict,
    jobs_tables, visualize_result)
from collections import defaultdict
from compmake import comp, comp_stage_job_id
from diffeoplan.configuration import set_current_config
from diffeoplan.library import UncertainImage
from reprep import Report
from reprep.report_utils import ReportManager, StoreResults
from compmake.ui.user_utils import comp_store


def create_bench_jobs(config, algos, testcases, outdir):
    # dict(id_algo, id_tc, id_discdds, plan_length) => PlanningResults
    allplanning = StoreResults()
    # dict(id_algo, id_tc, id_discdds, plan_length) => resultstats
    allruns = StoreResults() 
    # dict(id_algo, id_tc) => DiffeoPlanningAlgorithm
    algoinit = StoreResults()
    
    rm = ReportManager(outdir)
    
    comp_store(config, job_id='config') 
    
    # Let's instantiate all test cases and sort them by discdds
    # so that we do only one initialization per algorithms
    id_discdds2testcases = defaultdict(lambda: {}) 
    alltc = {} # id -> Promise TestCase
    for id_tc in testcases:
        alltc[id_tc] = comp(instantiate_testcase, comp_store(config), id_tc)
        # Do it once, now, to get its dds
        tc1 = config.testcases.instance(id_tc) 
        id_discdds2testcases[tc1.id_discdds][id_tc] = tc1
    
    # Load discdds before, they might be automatically generated
    # as well so we want the generation to happen only once.
    discdds = {} # id -> Promise DiffeoSystem
    for id_discdds in id_discdds2testcases: 
        discdds[id_discdds] = comp(instantiate_discdds, comp_store(config), id_discdds)
        
    # for each algorithm
    for id_algo in algos:
        config.algos[id_algo] # check it is in the configuration
            
        # for each dynamics
        for id_discdds, dds in discdds.items():
            job_id = 'init-%s-%s' % (id_algo, id_discdds)
            
            # initialize the algorithm for that dynamics
            algo = comp(init_algorithm, comp_store(config),
                        id_algo, id_discdds, discdds[id_discdds], job_id=job_id)
            algoinit[dict(id_algo=id_algo, id_discdds=id_discdds)] = algo
            
            # for each test case in that dynamics
            for id_tc, tc in id_discdds2testcases[id_discdds].items():
                
                # run the planning
                job_id = 'plan-%s-%s' % (id_algo, id_tc)
                result = comp(run_planning, id_algo,
                              id_tc, alltc[id_tc], algo, job_id=job_id)
                
                # compute statistics
                result_stats = comp(run_planning_stats, result,
                                    dds, alltc[id_tc],
                                    job_id=job_id + '-stats') 
    
                attrs = dict(id_algo=id_algo, id_tc=id_tc,
                             id_discdds=tc.id_discdds,
                             true_plan_length=len(tc.true_plan))
                allruns[attrs] = result_stats
                allplanning[attrs] = result
    
    jobs_report_algo_init(config, rm, algoinit)
    jobs_report_tc(config, rm, testcases, alltc)
    jobs_report_dds(config, rm, discdds)

    allstats = StoreResults()
    for key, run in allruns.items():
        allstats[key] = comp(results2stats_dict, run,
                             job_id=comp_stage_job_id(run, 'statsdict'))

    jobs_tables(allstats, rm)
    jobs_visualization(config, allruns, rm)
    
    rm.create_index_job()
    

def instantiate_discdds(config, id_discdds):
    set_current_config(config)
    return config.discdds.instance(id_discdds)

def instantiate_testcase(config, id_tc):
    set_current_config(config)
    return config.testcases.instance(id_tc)

def jobs_report_tc(config, rm, testcases, alltc):
    for id_tc in testcases:
        tc = config.testcases.instance(id_tc)
        report = comp(report_tc, comp_store(config), id_tc, alltc[id_tc],
                      job_id='report_tc-%s' % id_tc)
        report_attrs = dict(true_plan_length=len(tc.true_plan),
                            id_tc=id_tc, id_discdds=tc.id_discdds)
        rm.add(report, 'tc', **report_attrs)

def jobs_report_dds(config, rm, discdds):
    for id_discdds, dds in discdds.items():
        report = comp(report_dds, comp_store(config), id_discdds, dds,
                      job_id='report_dds-%s' % id_discdds)
        rm.add(report, 'dds', id_discdds=id_discdds)
        
def jobs_report_algo_init(config, rm, algoinit): #@UnusedVariable
    """ add the initialization report for each algorithm """
    for k, algo in algoinit.items():
        id_algo = k['id_algo'] 
        id_discdds = k['id_discdds']
        job_id = 'init-%s-%s-report' % (id_algo, id_discdds)
        report = comp(report_init_algorithm, id_algo, id_discdds, algo,
                      job_id=job_id)
        report_attrs = dict(id_algo=id_algo, id_discdds=id_discdds)
        rm.add(report, 'init', **report_attrs)
        

def jobs_visualization(config, allruns, rm):
    
    for run in allruns:
        id_tc = run['id_tc']
        id_algo = run['id_algo']
        result = allruns[run]
        job_id = 'plan-%s-%s-visualize' % (id_algo, id_tc)
        report = comp(visualize_result, comp_store(config), id_tc, id_algo,
                      result, job_id=job_id)
        report_attrs = run
        rm.add(report, 'visualization', **report_attrs)


def init_algorithm(config, id_algo, id_discdds, discdds):
    """ Returns the instanced DiffeoPlanninAlgorithm """
    # instance the algorithm
    set_current_config(config)
    algo = config.algos.instance(id_algo)    
    # initialize the algorithm with the dynamics
    # TODO: add computation time
    #t0 = time.clock()
    algo.set_name_for_log(id_algo)
    algo.init(id_discdds, discdds) 
    #init_time = time.clock() - t0
    return algo


@contract(returns=Report)
def report_init_algorithm(id_algo, id_discdds, algo):
    """ Creates a report for the initialization phase of the algorithm """
    r = Report('init-%s-%s' % (id_algo, id_discdds)) 
    algo.init_report(r)
    return r

    
@contract(returns=Report)
def report_tc(config, id_tc, tc):
    set_current_config(config)
    r = Report('tc-%s' % (id_tc))
    tc.display(r)
    return r


@contract(returns=Report)
def report_dds(config, id_discdds, discdds, image='lena'):
    set_current_config(config)
    r = Report('dds-%s' % (id_discdds))
    image = config.images.instance(image)
    y0 = UncertainImage(image)
    discdds.display(r, y0)
    return r


