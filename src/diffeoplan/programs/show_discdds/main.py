from .. import declare_command, write_report_files
from reprep import Report
import os


@declare_command('show-discdds',
                 'show-discdds  [-i <image>] [<discdds1> <discdds2> ...]')
def show_discdds(config, parser): #@UnusedVariable
    """ Displays the learned DDS """
    parser.add_option("-i", "--id_image", help="ID image.", default='lena')
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/show_dds/')
    options, which = parser.parse()
    
    outdir = options.output 
  
    if not which:
        todo = config.discdds.keys()  
    else:
        todo = config.discdds.expand_names(which)

    id_image = options.id_image
    image = config.images.instance(id_image)
    
    for id_dds in todo:
        dds = config.discdds.instance(id_dds) 
        report = Report(id_dds)
        
        dds.display(report, image=image)
    
        write_report_files(report, basename=os.path.join(outdir, id_dds))    


