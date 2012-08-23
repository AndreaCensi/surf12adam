from . import (create_tables, report_for_stats, write_report, run_planning_stats,
    run_planning)
from collections import defaultdict
from compmake import comp
from contracts import contract
from diffeoplan.programs.bench.visualization import create_visualization_jobs
from reprep import Report
from reprep.report_utils import StoreResults
import os


def create_bench_jobs(config, algos, testcases, outdir):
    # dict(id_algo, id_tc, id_discdds, plan_length) => PlanningResults
    allplanning = StoreResults()
    # dict(id_algo, id_tc, id_discdds, plan_length) => resultstats
    allruns = StoreResults() 
    # dict(id_algo, id_tc) => DiffeoPlanningAlgorithm
    algoinit = StoreResults()
    
    
    # Let's instantiate all test cases and sort them by discdds
    # so that we do only one initialization per algorithms
    id_discdds2testcases = defaultdict(lambda: {}) 
    for id_tc in testcases:
        tc = config.testcases.instance(id_tc)
        id_discdds2testcases[tc.id_discdds][id_tc] = tc
        
    # for each algorithm
    for id_algo in algos:
        config.algos[id_algo] # check it is in the configuration
            
        # for each dynamics
        for id_discdds in id_discdds2testcases:
            job_id = 'init-%s-%s' % (id_algo, id_discdds)
            
            # initialize the algorithm for that dynamics
            algo = comp(init_algorithm, config,
                        id_algo, id_discdds, job_id=job_id)
            algoinit[dict(id_algo=id_algo, id_discdds=id_discdds)] = algo
            
            # for each test case in that dynamics
            for id_tc in id_discdds2testcases[id_discdds]:
                
                # run the planning
                job_id = 'plan-%s-%s' % (id_algo, id_tc)
                result = comp(run_planning, config, id_algo,
                              id_tc, algo, job_id=job_id)
                
                # compute statistics
                result_stats = comp(run_planning_stats, config, result,
                                    job_id=job_id + '-stats') 
    
                attrs = dict(id_algo=id_algo, id_tc=id_tc,
                             id_discdds=tc.id_discdds,
                             plan_length=len(tc.true_plan))
                allruns[attrs] = result_stats
                allplanning[attrs] = result
    
    create_algo_init_jobs(outdir, algoinit)
    
    def add_report(short, stats, desc):
        job_id = 'report-%s' % short
        report = comp(report_for_stats, short, stats, desc, job_id=job_id)
        report_basename = os.path.join(outdir, 'reports/%s' % short)
        comp(write_report, report, report_basename, job_id=job_id + '-write')
    
    for id_algo in algos:
        this_algo = allruns.select(id_algo=id_algo)
        
        stats = list(this_algo.values())
        add_report('%s' % id_algo, stats,
                   'All runs of algorithm %s' % id_algo)
    
        alldiscdds = set(this_algo.field('id_discdds'))
        
        for id_discdds in alldiscdds:
            stats = list(this_algo.select(id_discdds=id_discdds).values())
            add_report('%s-%s' % (id_algo, id_discdds), stats,
                       'All runs of algorithm %s on %s' % (id_algo, id_discdds))
        
    create_tables(outdir, allruns)
    
    create_visualization_jobs(config, outdir, allruns)
    
def create_algo_init_jobs(outdir, algoinit):
    """ add the initialization report for each algorithm """
    for k, algo in algoinit.items():
        id_algo = k['id_algo'] 
        id_discdds = k['id_discdds']
        job_id = 'init-%s-%s-report' % (id_algo, id_discdds)
        report = comp(init_algorithm_report, id_algo, id_discdds, algo, job_id=job_id)
        basename = 'init_%s_%s' % (id_algo, id_discdds)
        report_basename = os.path.join(outdir, 'init_reports', basename)
        comp(write_report, report, report_basename, job_id=job_id + '-write')
    
def init_algorithm(config, id_algo, id_discdds):
    """ Returns the instanced DiffeoPlanninAlgorithm """
    # instance the algorithm
    algo = config.algos.instance(id_algo)    
    # initialize the algorithm with the dynamics
    discdds = config.discdds.instance(id_discdds)
    # TODO: add computation time
    #t0 = time.clock()
    algo.init(id_discdds, discdds) 
    #init_time = time.clock() - t0
    return algo

@contract(returns=Report)
def init_algorithm_report(id_algo, id_discdds, algo):
    """ Creates a report for the initialization phase of the algorithm """
    r = Report('init-%s-%s' % (id_algo, id_discdds)) 
    algo.init_report(r)
    return r
    
