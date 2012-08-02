from .. import declare_command, logger
from . import compute_stats, run_planning
from bootstrapping_olympics.utils import UserError, expand_string
from compmake import batch_command, compmake_console, comp
from diffeoplan import DiffeoplanConfig
import os
from compmake.storage import use_filesystem

@declare_command('bench',
                 'bench -a <algorithms> -t <testcases>')
def bench_main(global_config, parser): #@UnusedVariable
    parser.add_option("-a", "--algorithms", default='*',
                      help="Comma-separated list of algorithms. Can use *.")
    
    parser.add_option("-t", "--testcases", default='*',
                       help="Comma-separated list of algorithms. Can use *.")

    parser.add_option("-o", "--output", default='out/planning-results',
                      help="Output directory")

    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    
    options = parser.parse_options()
    
    config = DiffeoplanConfig

    def expand(what, options):
        expanded = expand_string(what, options)
        if not expanded:
            msg = 'Specified sets %r not found.' % what
            msg += ' Available: %s' % options
            raise UserError(msg)
        return expanded
    
    algos = expand(options.algorithms, config.algos.keys())
    testcases = expand(options.testcases, config.testcases.keys())

    logger.info('Using %d algorithms: %s' % (len(algos), algos))
    logger.info('Using %d testcases.' % (len(testcases)))
    
    
    storage = os.path.join(options.output, 'compmake')
    use_filesystem(storage)
    
    for id_algo in algos:
        algo_results = []
        for id_tc in testcases:
            job_id = 'plan-%s-%s' % (id_algo, id_tc)
            result = comp(run_planning, id_algo, id_tc, job_id=job_id) 
            algo_results.append(result)
        job_id = 'stats-%s' % id_algo
        stats = comp(compute_stats, id_algo, algo_results, job_id=job_id)
        #comp(create_report, id_algo, stats, out)
            
    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0







