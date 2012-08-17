from . import logger, make_logcases
from .. import  declare_command
import os
from diffeoplan.utils.script_utils import UserError


@declare_command('logcases',
                 'logcases --log <logname (bag)> -n <number> -d <delay>')
def logcases_main(config, parser):
    """ Creates test cases from log files. """
    parser.add_option('-n', help="Number of test cases.", default=1, type='int')
    parser.add_option('-d', '--delay', help="Delay between data (>=1)", default=1, type='int')
    parser.add_option('-l', '--log', help='Log filename')
    parser.add_option('-o', '--output', default='out/log-testcases',
                      help='Output directory')

    parser.add_option('--dds',
                       help='Name of DDS that this test case will be associated to.')
    
    options = parser.parse_options()

    if options.log is None:
        msg = 'Please specify the filename using the --log option.'
        raise UserError(msg)

    if options.dds is None:
        msg = 'Please specify the DDS name using the --dds option.'
        raise UserError(msg)
        # check it exists?
        
    id_discdds = options.dds
    discdds = config.discdds.instance(id_discdds)
    
    
    id_tc_pattern = 'tc_%s_d%d_' % (id_discdds, options.delay) + '%03d'    
    logger.info('Creating test cases with pattern %r' % id_tc_pattern)
    
    
    bagfile = options.log 
    if not os.path.exists(bagfile):
        msg = 'Log %r does not exist.' % bagfile
        raise UserError(msg)

    if not '.processed.bag' in bagfile:
        msg = 'I would expect you use this program on a .processed.bag file.'
        logger.warning(msg)
    

    cases = make_logcases(bagfile, n=options.n, delta=options.delay,
                     id_discdds=id_discdds, id_tc_pattern=id_tc_pattern,
                     discdds=discdds)

    outdir = os.path.join(options.output, id_discdds)
    
    if not os.path.exists(outdir):
        os.makedirs(outdir)
        
        
    for tc in cases:
        tc.save(outdir)    
    
    

    
