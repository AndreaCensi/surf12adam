from .. import declare_command
from . import discdds_make_test_cases


@declare_command('gentests',
                 'gentests -n <number> -l <plan length> [-i <image>] [<discdds1> <discdds2> ...]')
def maketestcases_display_main(config, parser): #@UnusedVariable
    """ Creates synthetic test cases using the learned DDS. """
    parser.add_option("-i", "--id_image", help="ID image.", default='lena')
    parser.add_option("-n", help="Number of test cases.", default=1, type='int')
    parser.add_option("-l", '--length',
                      help='Length of randomly-generated plan', default=2,
                      type='int')
    parser.add_option("-o", "--output", default='out/dp-gentests',
                      help="Output directory")
    
    options, which = parser.parse()
        
    if not which:
        todo = config.discdds.keys()  
    else:
        todo = config.discdds.expand_names(which)

    for id_discdds in todo: 
        discdds_make_test_cases(config=config, id_discdds=id_discdds,
                                id_image=options.id_image,
                                outdir=options.output,
                                num_cases=options.n,
                                plan_length=options.length)
        
        


    
