from . import declare_command


@declare_command('config', 'show-config [-v]')
def show_config(config, parser): #@UnusedVariable
    """ Shows all objects described in the configuration files. """
    options = parser.parse_options() #@UnusedVariable

    print('Images:')    
    print(config.images.summary_string_id_desc())

    print('List of symbolic diffeos defined (*.symdiffeos.yaml):')
    print(config.symdiffeos.summary_string_id_desc())
    
    print('List of symbolic DDS:')
    print(config.symdds.summary_string_id_desc())
    
    print('List of discrete DDS:')
    print(config.discdds.summary_string_id_desc())
        
#    print('Diffeos:')
#    print(config.diffeos.summary_string_id_desc())
         
    print('Algorithms:')
    print(config.algos.summary_string_id_desc())

    print('Test cases:')
    print(config.testcases.summary_string_id_desc())
    
