from . import declare_command
from reprep import Report
import os
from diffeoplan.programs.utils import write_report_files

@declare_command('show-tc', 'show-tc [-o <dir>] [<tc1> <tc2> ...]')
def show_tc(config, parser): #@UnusedVariable
    """ Visualizes some test cases. """ 
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/show_tc/')
    
    options, which = parser.parse()
    
    outdir = options.output
    
    if not which:
        which = config.testcases.keys()
    
    for id_tc in which:
        tc = config.testcases.instance(id_tc) 
        report = Report(id_tc)
        tc.display(report)
        
        write_report_files(report, basename=os.path.join(outdir, id_tc))
