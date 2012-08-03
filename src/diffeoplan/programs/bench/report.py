from . import contract, logger
from reprep import Report
from conf_tools.utils import friendly_path
import os


def report_for_stats(short, stats, desc):
    r = Report(short, caption=desc)
    report_stats(r, stats)
    return r
    

def report_stats(r, stats):
    sub = r.section('init_time', caption='Initialization time')
    values = [s['init_time'] for s in stats]
    report_values(sub, values)

    sub = r.section('plan_time', caption='Planning time')
    values = [s['plan_time'] for s in stats]
    report_values(sub, values)

    sub = r.section('plan_length', caption='Length of plan found')
    values = [len(s['result'].plan) for s in stats]
    report_values(sub, values)

    distance_measures = stats[0]['dist_y0_y1p'].keys()
    for d in distance_measures:
        values = [s['dist_y0_y1p'][d] for s in stats]
        report_values(r.section(d), values)

    
import numpy as np

@contract(values='seq[N]')
def report_values(r, values):
    values = np.array(values)
    r.data('num', len(values))
    r.data('min', np.min(values))
    r.data('max', np.max(values))
    r.data('mean', np.mean(values))
    r.data('stddev', np.std(values))
    f = r.figure(cols=5)
    with f.plot('sequence') as pylab:
        pylab.plot(values, '.')
    
    with f.plot('histogram') as pylab:
        pylab.hist(values)
    

def write_report(report, report_basename): 
    html = report_basename + '.html'
    logger.info('Writing to %r.' % friendly_path(html))
    rd = os.path.join(os.path.dirname(report_basename), 'images')
    report.to_html(html, write_pickle=True, resources_dir=rd)
    # TODO: save hdf format

def report_for_all(stats):
    # TODO
    pass
