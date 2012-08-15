from . import (create_tables, report_for_stats, write_report, run_planning_stats,
    run_planning)
from compmake import comp
from reprep.report_utils import StoreResults
import os
from diffeoplan.programs.bench.visualization import create_visualization_jobs

def create_bench_jobs(config, algos, testcases, outdir):
    allruns = StoreResults()
    
    for id_algo in algos:
        for id_tc in testcases:
            tc = config.testcases.instance(id_tc)
            job_id = 'plan-%s-%s' % (id_algo, id_tc)
            # If you didn't use compmake
            #  result = run_planning(config, id_algo, id_tc, job_id=job_id)
            #  result_stats = run_planning_stats(config, result)
            # To use compamke, 
            # change  result = f(parma, param)
            # into:   result = comp(f, param, param)
            result = comp(run_planning, config, id_algo, id_tc, job_id=job_id)
            
            result_stats = comp(run_planning_stats, config, result,
                                job_id=job_id + '-stats') 

            attrs = dict(id_algo=id_algo,
                               id_tc=id_tc,
                               id_discdds=tc.id_discdds,
                               #id_image=xxx
                               plan_length=len(tc.true_plan))
            allruns[attrs] = result_stats
    
    
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
    