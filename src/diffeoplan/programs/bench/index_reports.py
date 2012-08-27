from . import logger, contract, np
from compmake import comp
from compmake.utils import duration_human
from conf_tools.utils import friendly_path
from reprep.report_utils.store_results import StoreResults
import os
import time
from reprep import Report


class ReportManager:
    
    def __init__(self, outdir):
        self.outdir = outdir
        self.allreports = StoreResults()
        
    def add(self, report, report_type, **kwargs):
        key = dict(report=report_type, **kwargs)
        dirname = os.path.join(self.outdir, report_type)
        basename = "_".join(kwargs.values())
        filename = os.path.join(dirname, basename)
        job_id = 'write-%s-%s' % (report_type, basename)
        filename = comp(write_report, report, filename, job_id=job_id)
        self.allreports[key] = filename    

    
@contract(report=Report, report_basename='str')
def write_report(report, report_basename): 
    html = report_basename + '.html'
    logger.info('Writing to %r.' % friendly_path(html))
    rd = os.path.join(os.path.dirname(report_basename), 'images')
    report.to_html(html, write_pickle=True, resources_dir=rd)
    # TODO: save hdf format
    return html

@contract(reports=StoreResults, index=str)
def index_reports(reports, index):
    """
        Writes an index for the reports to the file given. 
        The special key "report" gives the report type.
        
        reports[dict(report=...,param1=..., param2=...) ] => filename
    """
    
    logger.info('Writing on %s' % index)
    
    f = open(index, 'w')
    
    f.write("""
        <style type="text/css">
        span.when { float: right; }
        li { clear: both; }
        </style>
    """)
    
    mtime = lambda x: os.path.getmtime(x)
    
    # create order statistics
    alltimes = np.array(map(mtime, reports.values())) 
    
    def order(filename):
        """ returns between 0 and 1 the order statistics """
        histime = mtime(filename)
        compare = (alltimes < histime) 
        return np.mean(compare * 1.0)
        
    def style_order(order):
        if order > 0.95:
            return "color: green;"
        if order > 0.9:
            return "color: orange;"        
        if order < 0.5:
            return "color: gray;"
        return ""     
        
        
    def write_li(k, filename):
        href = os.path.relpath(filename, os.path.dirname(index))
        desc = ",  ".join('%s = %s' % (a, b) for a, b in k.items())
        # friendly time
        when = duration_human(time.time() - mtime(filename))
        style = style_order(order(filename))
        f.write('<li style="%s"><a href="%s">%s</a> <span class="when">%s ago</span></li>' 
                % (style, href, desc, when))

        
    # write the first 10
    last = list(reports.items())
    last.sort(key=lambda x: (-mtime(x[1])))
    nlast = min(len(last), 10)
    f.write('<h2 id="last">Last %d reports</h2>\n' % (nlast))

    f.write('<ul>')
    for i in range(nlast):
        write_li(*last[i])
    f.write('</ul>')
    report_types = sorted(list(set(reports.field('report'))))
    

    for report_type in report_types:
        f.write('<h2 id="%s">%s</h2>\n' % (report_type, report_type))
        f.write('<ul>')
        r = reports.select(report=report_type)
        items = list(r.items())    
        #items.sort(key=lambda x: mtime(x[1]))
        for k, filename in items:
            write_li(k, filename)
        f.write('</ul>')

    f.close()
    
def write_sections(allruns):
    fields = allruns.field_names()
    fields.sort() # TODO: sort
    if not fields:
        return
    print fields
    f0 = fields[0]
    
    
    
