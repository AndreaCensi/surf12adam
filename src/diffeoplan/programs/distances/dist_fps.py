from .. import declare_command, logger
from bootstrapping_olympics.utils.in_a_while import InAWhile

@declare_command('dist-fps',
                 'dist-fps -d <distances> -t <testcases>')
def distances_main(config, parser): #@UnusedVariable
    """ FPS statistics for distances """
    parser.add_option("-d", "--distances", default='*',
                      help="Comma-separated list of distances. Can use *.")
    
    parser.add_option("-t", "--testcases", default='*',
                       help="Comma-separated list of algorithms. Can use *.")

    parser.add_option("-r", "--repeat", default=1, type='int',
                       help="Repeat many times.")

#    parser.add_option("-c", "--command",
#                      help="Command to pass to compmake for batch mode")
#    
    options = parser.parse_options()
    
    distances = config.distances.expand_names(options.distances)
    testcases = config.testcases.expand_names(options.testcases)

    logger.info('Using distances: %s' % distances)
    logger.info('Using testcases: %s' % testcases)
    
    testcases = map(config.testcases.instance, testcases)
    
    # Compmake storage for results
    for id_distance in distances:    
        dist = config.distances.instance(id_distance)
        
        fps = InAWhile()
        for _ in range(options.repeat):
            for tc in testcases: 
                fps.its_time()
                result = dist.distance(tc.y0, tc.y1)
                print('%s / %s: %s (%s)' % (id_distance, tc.id_tc, result, tc.true_plan))
        print('frames per second: %s' % fps.fps())

#    store = StoreResults()
#    # Compmake storage for results
#    for id_distance, id_tc in itertools.product(distances, testcases):
#        
#        dist = config.distances.instance(id_distance) 
#        tc = config.testcases.instance(id_tc)
#        result = dist.distance(tc.y0, tc.y1)
#        keys = dict(id_distance=id_distance, id_tc=id_tc,
#                    nsteps=len(tc.true_plan))
#        store[keys] = result
#        
#def compute_distances(id_distance):
#        
#    storage = os.path.join(outdir, 'compmake')
#    use_filesystem(storage)
#    read_rc_files()
#    
#    create_bench_jobs(config=config, algos=algos,
#                      testcases=testcases, outdir=outdir)
#    
#    if options.command:
#        return batch_command(options.command)
#    else:
#        compmake_console()
#        return 0
#    




