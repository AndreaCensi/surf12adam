from . import logger, make_logcases
from .. import declare_command
from diffeoplan.utils import UserError
import os


@declare_command('logcases',
                 'logcases -s <stream> --dds <id_discdds> -n <number> -d <delay>')
def dp_logcases_main(config, parser):
    """ Creates test cases from log files. """
    parser.add_option('-n', help="Number of test cases.", default=1, type='int')
    parser.add_option('--seed', help="Seed for randomly choosing the images.", type='int')
    parser.add_option('-d', '--delay', help="Delay between data (>=1)", default=1, type='int')
    parser.add_option('-s', '--stream', help='Stream ID')
    parser.add_option('-o', '--output', default='out/dp-logcases',
                      help='Output directory')

    parser.add_option('--dds',
                       help='Name of DDS that this test case will be associated to.')
    
    options = parser.parse_options()

    if options.seed is None:
        msg = 'Please specify the seed using the --seed option. '
        msg += '(trust me, this is the better way)'
        raise UserError(msg)

    if options.stream is None:
        msg = 'Please specify the stream using the --stream option.'
        raise UserError(msg)

    if options.dds is None:
        msg = 'Please specify the DDS name using the --dds option.'
        raise UserError(msg)
        # check it exists?
        
    id_stream = options.stream
    id_discdds = options.dds
    discdds = config.discdds.instance(id_discdds)
    
    
    id_tc_pattern = 'tc_%s_d%d_' % (id_discdds, options.delay) + '%03d'    
    logger.info('Creating test cases with pattern %r' % id_tc_pattern)
    
    outdir = os.path.join(options.output, id_discdds)
    
    if not os.path.exists(outdir):
        os.makedirs(outdir)
        
    cases = make_logcases(seed=options.seed, config=config, id_stream=id_stream,
                           n=options.n, delta=options.delay,
                           id_discdds=id_discdds, id_tc_pattern=id_tc_pattern,
                           discdds=discdds)
        
    for tc in cases:
        tc.save(outdir)    
    
    

    
