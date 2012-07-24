from . import declare_command
from .. import DiffeoplanConfig


@declare_command('show-config', 'show-config [-v]')
def show_config(global_config, parser): #@UnusedVariable
    """ Shows all objects described in the configuration files. """
    parser.add_option("-v", "--verbose", default=False, action='store_true',
                      help="More verbose display.")
    options = parser.parse_options()
    
    verbose = options.verbose
    # TODO: add more details

    config = DiffeoplanConfig
    
    print('\nImages:')
    for id_image in config.images.keys():
        print('- %s ' % id_image)
        if verbose:
            image = config.images.instance(id_image)
            print('   %s ' % str(image.shape))
            
    print('\nDiffeos:')
    for id_diffeo in config.diffeos.keys():
        print('- %s ' % id_diffeo)
        if verbose:
            diffeo = config.diffeos.instance(id_diffeo)
    
    print('\nDDS:')
    for id_dds in config.dds.keys():
        print('- %s ' % id_dds)
        if verbose:
            dds = config.dds.instance(id_dds)
    
    
@declare_command('show-symdiffeos', 'show-symdiffeos [--instance] [id1 id2 ...]')
def show_symdiffeos(global_config, parser): #@UnusedVariable
    parser.add_option("--instance", default=False, action='store_true',
                      help="Actually instances the diffeomorphisms.")
    options, args = parser.parse()
    
    symdiffeos = DiffeoplanConfig.symdiffeos
    
    if args:
        which = args
    else:
        which = list(symdiffeos.keys()) 
    
    for id_symdiffeo in which: #@UndefinedVariable
        print('- %s ' % id_symdiffeo)
        if options.instance:
            symdiffeo = symdiffeos.instance(id_symdiffeo) 
            
            
            
        
