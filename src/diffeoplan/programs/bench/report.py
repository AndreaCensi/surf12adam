from . import contract, logger
from reprep import Report
from conf_tools.utils import friendly_path


@contract(id_algo='str', stats='list(dict)')
def report_for_algo(id_algo, stats):
    r = Report(id_algo)
    r.data('num_samples', len(stats))
    
    return r 

def write_report(report, report_basename):
    html = report_basename + '.html'
    logger.info('Writing to %r.' % friendly_path(html))
    report.to_html(html)
