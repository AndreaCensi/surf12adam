from . import declare_command
from .. import write_report_files
from reprep import Report
import os

@declare_command('show-tc', 'show-tc [-o <dir>] [<tc1> <tc2> ...]')
def show_tc(config, parser): #@UnusedVariable
    """ Displays the test cases. """ 
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-show-tc/')
    
    options, which = parser.parse()
    
    outdir = options.output
    
    if not which:
        todo = config.testcases.keys()  
    else:
        todo = config.testcases.expand_names(which)

    for id_tc in todo:
        tc = config.testcases.instance(id_tc) 
        report = Report(id_tc)
        tc.display(report)
        
        write_report_files(report, basename=os.path.join(outdir, id_tc))
