from .. import declare_command
from . import DiffeoSystemBounds, DiffeoSystemBounds2
from diffeoplan.programs.utils import write_report_files
from reprep import Report
import os


@declare_command('show-discdds-fill',
                 'show-discdds-fill [<discdds1> <discdds2> ...]')
def show_discdds_fill_main(config, parser):
    
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-show-discdds-fill/')
    parser.add_option("-t", "--tolerance", help="Normalized tolerance",
                      default=0.3, type='float')
    parser.add_option("--collapse_threshold", help="Collapse threshold",
                      default=0, type='float')
    parser.add_option("-n", "--num_iterations", help="Max number of iterations",
                      default=100000, type='int')
    parser.add_option("-v", "--min_visibility", help="Minimum visibility",
                      default=0.5, type='float')
    parser.add_option("--debug_it", help="Debug iterations",
                      default=100, type='float')

    options, which = parser.parse()
    
    outdir = options.output 
  
    if not which:
        todo = config.discdds.keys()  
    else:
        todo = config.discdds.expand_names(which)

    for id_dds in todo:
        dds = config.discdds.instance(id_dds) 
        report = Report(id_dds)
        
        show_ddsfill(id_dds, dds, report,
                     tolerance=options.tolerance,
                     collapse_threshold=options.collapse_threshold,
                     debug_it=options.debug_it,
                     max_it=options.num_iterations,
                     min_visibility=options.min_visibility)    
        
        write_report_files(report, basename=os.path.join(outdir, id_dds))    


def show_ddsfill(id_dds, dds, report, **args):
    ds = DiffeoSystemBounds(id_dds, dds, **args)
    ds.display(report)
         

@declare_command('show-discdds-fill2',
                 'show-discdds-fill2 [<discdds1> <discdds2> ...]')
def show_discdds_fill2_main(config, parser):
    """ Displays the intrinsic geometry of a learned DDS """
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-show-discdds-fill2/')
    parser.add_option("-t", "--tolerance", help="Normalized tolerance",
                      default=0.3, type='float')
    parser.add_option("--collapse_threshold", help="Collapse threshold",
                      default=0, type='float')
    parser.add_option("-n", "--num_iterations", help="Max number of iterations",
                      default=100000, type='int')
    parser.add_option("-v", "--min_visibility", help="Minimum visibility",
                      default=0.5, type='float')
    parser.add_option("--debug_it", help="Debug iterations",
                      default=100, type='float')

    options, which = parser.parse()
    
    outdir = options.output 
  
    if not which:
        todo = config.discdds.keys()  
    else:
        todo = config.discdds.expand_names(which)

    for id_dds in todo:
        dds = config.discdds.instance(id_dds) 
        report = Report(id_dds)
        
        show_ddsfill2(id_dds, dds, report,
                     tolerance=options.tolerance,
                     collapse_threshold=options.collapse_threshold,
                     debug_it=options.debug_it,
                     max_it=options.num_iterations,
                     min_visibility=options.min_visibility)    
        
        write_report_files(report, basename=os.path.join(outdir, id_dds))    


def show_ddsfill2(id_dds, dds, report, **args):
    ds = DiffeoSystemBounds2(id_dds, dds, **args)
    ds.display(report)
         

