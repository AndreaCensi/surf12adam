from . import create_bench_jobs
from .. import declare_command, logger
from compmake import (batch_command, compmake_console, use_filesystem,
    read_rc_files)
import os

@declare_command('bench',
                 'bench -a <algorithms> -t <testcases>')
def dp_bench_main(config, parser): #@UnusedVariable
    parser.add_option("-a", "--algorithms", default='*',
                      help="Comma-separated list of algorithms. Can use *.")
    
    parser.add_option("-t", "--testcases", default='*',
                       help="Comma-separated list of algorithms. Can use *.")

    parser.add_option("-o", "--output", default='out/dp-bench',
                      help="Output directory")

    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    
    options = parser.parse_options()
    
    algos = config.algos.expand_names(options.algorithms)
    testcases = config.testcases.expand_names(options.testcases)

    logger.info('Using %d algorithms: %s' % (len(algos), algos))
    logger.info('Using %d testcases.' % (len(testcases)))
    
    outdir = options.output
    
    # Compmake storage for results
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
    create_bench_jobs(config=config, algos=algos,
                      testcases=testcases, outdir=outdir)
    
    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0
    



