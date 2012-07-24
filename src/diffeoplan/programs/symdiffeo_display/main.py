from . import symdiffeo_display
from .. import declare_command, logger
from diffeoplan.configuration.master import DiffeoplanConfig
from procgraph_pil import resize


@declare_command('symdiffeo-display',
                 'symdiffeo-display  [-i <image>] [-r <resolution>] [<diffeo1> <diffeo2> ...]')
def symdiffeo_display_main(global_config, parser): #@UnusedVariable
    """ Shows all objects described in the configuration files. """
    parser.add_option("-i", "--id_image", help="ID image.", default='lena')
    parser.add_option("-d", "--id_diffeo", help="ID diffeo.")
    parser.add_option("-r", "--resolution", default=64,
                      help="Resolution (pixels)")
    options, which = parser.parse()
    
    resolution = int(options.resolution)
    
    config = DiffeoplanConfig
    if not which:
        which = config.symdiffeos.keys()
    
    id_image = options.id_image
    image = config.images.instance(id_image)
    
    image = resize(image, resolution, resolution) 
    
    for id_diffeo in which:
        diffeo = config.symdiffeos.instance(id_diffeo) 
        from reprep import Report
        report = Report(id_diffeo)
        
        symdiffeo_display(report, image, diffeo, resolution)
    
        filename = 'out/symdiffeo_display/%s.html' % id_diffeo
        logger.info('Writing to %r.' % filename)
        report.to_html(filename)

