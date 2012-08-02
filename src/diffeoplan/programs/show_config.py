from . import declare_command
from .. import DiffeoplanConfig


@declare_command('show-config', 'show-config [-v]')
def show_config(global_config, parser): #@UnusedVariable
    """ Shows all objects described in the configuration files. """
#    parser.add_option("-v", "--verbose", default=False, action='store_true',
#                      help="More verbose display.")
    options = parser.parse_options() #@UnusedVariable
    
#    verbose = options.verbose
    # TODO: add more details

    config = DiffeoplanConfig
    
    print('Images:')    
    print(config.images.summary_string_id_desc())

    print('List of symbolic diffeos defined (*.symdiffeos.yaml):')
    print(config.symdiffeos.summary_string_id_desc())
    
    print('List of symbolic DDS:')
    print(config.symdds.summary_string_id_desc())
    
    print('List of discrete DDS:')
    print(config.discdds.summary_string_id_desc())
        
    print('Diffeos:')
    print(config.diffeos.summary_string_id_desc())
         
    print('Algorithms:')
    print(config.algos.summary_string_id_desc())

    print('Test cases:')
    print(config.testcases.summary_string_id_desc())
    
