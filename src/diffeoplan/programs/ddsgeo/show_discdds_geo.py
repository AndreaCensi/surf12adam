from .. import declare_command, write_report_files
from reprep import Report
import os
from diffeoplan.library.analysis.structure.diffeo_structure import DiffeoStructure


@declare_command('show-discdds-geo',
                 'show-discdds-geo [<discdds1> <discdds2> ...]')
def show_discdds_geo_main(config, parser):
    """ Displays the intrinsic geometry of a learned DDS """
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-show-discdds-geo/')
    parser.add_option("-t", "--tolerance", help="Normalized tolerance",
                      default=0.3, type='float')
    options, which = parser.parse()
    
    outdir = options.output 
  
    if not which:
        todo = config.discdds.keys()  
    else:
        todo = config.discdds.expand_names(which)


    for id_dds in todo:
        dds = config.discdds.instance(id_dds) 
        report = Report(id_dds)
        
        show_diffeo_structure(dds, report, tolerance=options.tolerance)    
        
        write_report_files(report, basename=os.path.join(outdir, id_dds))    


def show_diffeo_structure(dds, report, tolerance):
    ds = DiffeoStructure(dds, tolerance=tolerance)
    with report.subsection('display') as r:
        ds.display(r)
    
    with report.subsection('show_reduction_steps') as r:
        ds.show_reduction_steps(r, max_nsteps=5)
        
    with report.subsection('show_reduction') as r:
        ds.show_reduction(r)


