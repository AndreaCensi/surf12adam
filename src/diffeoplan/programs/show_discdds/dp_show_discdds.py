from .. import declare_command, write_report_files
from reprep import Report
import os
from diffeoplan.library.images import UncertainImage


@declare_command('show-discdds',
                 'show-discdds  [-i <image>] [<discdds1> <discdds2> ...]')
def show_discdds(config, parser): #@UnusedVariable
    """ Creates a report for a DDS. """
    parser.add_option("-i", "--id_image", help="ID image.", default='lena')
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-show-discdds/')
    options, which = parser.parse()
    
    outdir = options.output 
  
    if not which:
        todo = config.discdds.keys()  
    else:
        todo = config.discdds.expand_names(which)

    id_image = options.id_image
    image = UncertainImage(config.images.instance(id_image))
    
    for id_dds in todo:
        dds = config.discdds.instance(id_dds) 
        report = Report(id_dds)
        
        dds.display(report, image=image)
    
        write_report_files(report, basename=os.path.join(outdir, id_dds))    


