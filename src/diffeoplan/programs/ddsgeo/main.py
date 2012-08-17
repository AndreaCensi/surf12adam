from .. import declare_command, write_report_files
from reprep import Report
import os
from diffeoplan.library.discdds import DiffeoStructure


@declare_command('show-discdds-geo',
                 'show-discdds-geo [<discdds1> <discdds2> ...]')
def ddsgeo(config, parser):
    """ Displays the intrinsic geometry of a learned DDS """
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/show_dds_geo/')
    options, which = parser.parse()
    
    outdir = options.output 
  
    if not which:
        todo = config.discdds.keys()  
    else:
        todo = config.discdds.expand_names(which)


    for id_dds in todo:
        dds = config.discdds.instance(id_dds) 
        report = Report(id_dds)
        
        
        show_diffeo_structure(dds, report)    
        
        write_report_files(report, basename=os.path.join(outdir, id_dds))    


def show_diffeo_structure(dds, report):
    ds = DiffeoStructure(dds)
    ds.display(report.section('distances'))
         
