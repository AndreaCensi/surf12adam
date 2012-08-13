from . import symdiffeo_display
from .. import declare_command, write_report_files
from procgraph_pil import resize
from reprep import Report
import os


@declare_command('show-symdiffeo',
                 'show-symdiffeo  [-i <image>] [-r <resolution>] [<diffeo1> <diffeo2> ...]')
def show_symdiffeo(config, parser): #@UnusedVariable
    """ Displays a discrete DDS (class DiffeoSystem) """
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/show_symdiffeos/')
    parser.add_option("-i", "--id_image", help="ID image.", default='lena')
    parser.add_option("-d", "--id_diffeo", help="ID diffeo.")
    parser.add_option("-r", "--resolution", default=64,
                      help="Resolution (pixels)")
    options, which = parser.parse()
    
    outdir = options.output
    resolution = int(options.resolution)
    
    if not which:
        which = config.symdiffeos.keys()
    # TODO: add expand
    
    id_image = options.id_image
    image = config.images.instance(id_image)
    
    image = resize(image, resolution, resolution) 
    
    for id_diffeo in which:
        diffeo = config.symdiffeos.instance(id_diffeo) 
        report = Report(id_diffeo)
        
        symdiffeo_display(report, image, diffeo, resolution)

        write_report_files(report, basename=os.path.join(outdir, id_diffeo))    

