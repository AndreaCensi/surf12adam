from . import create_bench_jobs
from .. import declare_command, logger
from compmake import (batch_command, compmake_console, use_filesystem,
    comp_prefix, read_rc_files)
from pprint import pformat
import os

@declare_command('batch',
                 'batch <sets> -c command')
def batch_main(config, parser):

    parser.add_option("-o", "--output", default='out/dp-batch',
                      help="Output directory")

    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    
    options, which = parser.parse()
    
    if not which:
        todo = config.sets.keys()
        id_comb = 'all'  
    else:
        todo = config.sets.expand_names(which)
        id_comb = "+".join(sorted(todo))
        
    logger.info('Batch sets to do: %s' % todo)
    
    outdir = os.path.join(options.output, 'set-%s' % id_comb)
    
    # Compmake storage for results
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
    for id_set in todo:
        logger.info('Instantiating batch set  %s' % id_set)
        spec = config.sets[id_set]
        
        try:            
            algos = config.algos.expand_names(spec['algorithms']) 
            testcases = config.testcases.expand_names(spec['testcases']) 
            comp_prefix('%s' % id_set)
            b_outdir = os.path.join(outdir, id_set)
            create_bench_jobs(config=config, algos=algos,
                              testcases=testcases, outdir=b_outdir)
        except:
            logger.error('Error while instantiating batch\n%s' % pformat(spec))
            raise
        
    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0
    
