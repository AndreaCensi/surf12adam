from . import declare_command


@declare_command('config', 'show-config [-v] [-t type]')
def show_config(config, parser): #@UnusedVariable
    """ Shows all objects described in the configuration files. """
    parser.add_option("-v", "--verbose", default=False, action='store_true',
                      help="Shows also test cases")
    parser.add_option('-t', "--type", default=True, help="Show only type t objects.")

    options = parser.parse_options()

    if options.type == 'streams' or options.type == True:
        print('Streams:')    
        print(config.streams.summary_string_id_desc_patterns())
    
    if options.type == 'images' or options.type == True:
        print('Images:')    
        print(config.images.summary_string_id_desc_patterns())
    
    if options.type == 'symdiffeos' or options.type == True:
        print('List of symbolic diffeos defined (*.symdiffeos.yaml):')
        print(config.symdiffeos.summary_string_id_desc_patterns())
        
    if options.type == 'symdds' or options.type == True:
        print('List of symbolic DDS:')
        print(config.symdds.summary_string_id_desc_patterns())
        
    if options.type == 'discdds' or options.type == True:
        print('List of discrete DDS:')
        print(config.discdds.summary_string_id_desc_patterns())
            
    if options.type == 'distances' or options.type == True:
        print('Distances:')
        print(config.distances.summary_string_id_desc_patterns())
             
    if options.type == 'algos' or options.type == True:
        print('Algorithms:')
        print(config.algos.summary_string_id_desc_patterns())
    
    if options.type == 'sets' or options.type == True:
        print('Batch experiments:')
        print(config.sets.summary_string_id_desc_patterns())
        
    if options.type == 'testcases' or options.type == True:
        if options.verbose:
            print('Test cases:')
            print(config.testcases.summary_string_id_desc_patterns())
        else:
            n = len(config.testcases)
            print('There are %d testcases; use -v to show them' % n)
