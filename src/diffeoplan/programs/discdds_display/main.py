from . import discdds_display
from .. import declare_command, logger
from diffeoplan.configuration.master import DiffeoplanConfig
from conf_tools.utils.friendly_paths import friendly_path


@declare_command('discdds-display',
                 'discdds-display  [-i <image>] [<discdds1> <discdds2> ...]')
def discdds_display_main(global_config, parser): #@UnusedVariable
    parser.add_option("-i", "--id_image", help="ID image.", default='lena')
    options, which = parser.parse()
    
    config = DiffeoplanConfig
    if not which:
        which = config.discdds.keys()
    
    id_image = options.id_image
    image = config.images.instance(id_image)
    
    for id_diffeo in which:
        dds = config.discdds.instance(id_diffeo) 
        from reprep import Report
        report = Report(id_diffeo)
        
        discdds_display(report, dds, image)
    
        filename = 'out/discdds_display/%s.html' % id_diffeo
        logger.info('Writing to %r.' % friendly_path(filename))
        report.to_html(filename)


