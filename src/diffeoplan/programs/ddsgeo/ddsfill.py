from .. import declare_command
from reprep import Report
import os
from diffeoplan.programs.utils import write_report_files
from diffeoplan.programs.ddsgeo.diffeo_system_bounds import DiffeoSystemBounds


@declare_command('show-discdds-fill',
                 'show-discdds-fill [<discdds1> <discdds2> ...]')
def ddsfill(config, parser):
    """ Displays the intrinsic geometry of a learned DDS """
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/show_dds_fill/')
    options, which = parser.parse()
    
    outdir = options.output 
  
    if not which:
        todo = config.discdds.keys()  
    else:
        todo = config.discdds.expand_names(which)


    for id_dds in todo:
        dds = config.discdds.instance(id_dds) 
        report = Report(id_dds)
        
        show_ddsfill(id_dds, dds, report)    
        
        write_report_files(report, basename=os.path.join(outdir, id_dds))    


def show_ddsfill(id_dds, dds, report):
    ds = DiffeoSystemBounds(id_dds, dds, info_threshold=0.6)
    ds.display(report)
         
