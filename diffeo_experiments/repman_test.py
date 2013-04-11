from reprep.report_utils import ReportManager
from compmake import (comp, batch_command, compmake_console, use_filesystem,
    read_rc_files)
import os
from reprep import Report
import pdb



def main():
    outdir = 'test/repman'
    
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    
    rm = ReportManager(outdir)
    report = comp(make_rep1, 'TestReport3')
    report2 = comp(make_rep1, 'TestReport4')
    
    
    rm.add(report, 'rep3')
    rm.add(report2, 'rep4')
    
    rm.create_index_job()
    read_rc_files()
    
    compmake_console()
    
def make_rep1(title):
    report = Report(title)
    report.text('Summary', 'Test report 1')
    return report
    
    
    
if __name__ == '__main__':
    main()
