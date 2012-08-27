from . import contract, np
from reprep import Report

def report_for_stats(short, stats, desc):
    r = Report(short, caption=desc)
    report_stats(r, stats)
    return r
    

def report_stats(r, stats):
    if False: # need to update, like in tables.py
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



def report_for_all(stats):
    # TODO
    pass
