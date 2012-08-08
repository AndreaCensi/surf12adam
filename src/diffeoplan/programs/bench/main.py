from . import (create_tables, report_for_stats, write_report, run_planning_stats,
    run_planning)
from .. import declare_command, logger
from bootstrapping_olympics.utils import UserError, expand_string
from compmake import batch_command, compmake_console, comp, use_filesystem
from compmake.scripts.master import read_rc_files
from reprep.report_utils import StoreResults
import os

@declare_command('bench',
                 'bench -a <algorithms> -t <testcases>')
def bench_main(config, parser): #@UnusedVariable
    parser.add_option("-a", "--algorithms", default='*',
                      help="Comma-separated list of algorithms. Can use *.")
    
    parser.add_option("-t", "--testcases", default='*',
                       help="Comma-separated list of algorithms. Can use *.")

    parser.add_option("-o", "--output", default='out/planning-results',
                      help="Output directory")

    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    
    options = parser.parse_options()
    
    def expand(what, spec, options):
        try:
            expanded = expand_string(spec, options)
        except ValueError:
            expanded = []
        if not expanded:
            msg = 'Specified set %r of %s not found.' % (spec, what)
            msg += ' Available %s: %s' % (spec, options)
            raise UserError(msg)
        return expanded
    
    algos = expand('algorithms', options.algorithms, config.algos.keys())
    if not list(config.testcases.keys()):
        raise UserError('No test cases defined (try `dp gentests`)')
    testcases = expand('testcases', options.testcases, config.testcases.keys())

    logger.info('Using %d algorithms: %s' % (len(algos), algos))
    logger.info('Using %d testcases.' % (len(testcases)))
    
    outdir = options.output
    
    # Compmake storage for results
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
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
        print alldiscdds
        for id_discdds in alldiscdds:
            stats = list(this_algo.select(id_discdds=id_discdds).values())
            add_report('%s-%s' % (id_algo, id_discdds), stats,
                       'All runs of algorithm %s on %s' % (id_algo, id_discdds))
    
    
    create_tables(outdir, allruns)
    
    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0




