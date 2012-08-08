from . import write_report, np
from compmake import comp
from reprep import Report
import itertools

statistics = {}
statistics['init_time'] = lambda s: s['init_time']
statistics['plan_time'] = lambda s: s['plan_time']
statistics['plan_length'] = lambda s: len(s['result'].plan)
statistics['dist_values_L1_old'] = lambda s: s['dist_y0_y1p']['values_L1']
statistics['dist_values_L2_old'] = lambda s: s['dist_y0_y1p']['values_L2']
statistics['dist_values_L1'] = lambda s: s['dist_y1_y1p']['values_L1']
statistics['dist_values_L2'] = lambda s: s['dist_y1_y1p']['values_L2']

reductions = {}
reductions['min'] = np.min
reductions['max'] = np.max
reductions['mean'] = np.mean
reductions['stddev'] = np.std
    
def f(stats, s, r):
    reduce_ = reductions[r]
    map_ = statistics[s]
    #print('computing %s %s' % (s, r))
    return reduce_([map_(x) for x in stats])

tables = {}
for a, b in itertools.product(statistics, reductions):
    def funcC(a, b):
        def func(stats):
            return f(stats, a, b)
        return func
    tables['%s-%s' % (a, b)] = funcC(a, b)
        
def create_table_algos_discdds(allruns, id_table):
    algos = list(set(allruns.field('id_algo')))
    discdds = list(set(allruns.field('id_discdds')))
    
    data = np.zeros((len(algos), len(discdds)), dtype='S64')
    
    for i, id_algo in enumerate(algos):
        for j, id_discdds in enumerate(discdds):
            stats = allruns.select(id_algo=id_algo, id_discdds=id_discdds).values()
            value = tables[id_table](stats)
            data[i, j] = '%g' % value
             
    r = Report(id_table)
    r.table(id_table, data=data, cols=discdds, rows=algos, caption=id_table)
    return r
                                    
def create_tables(outdir, allruns):
    for id_table in tables: 
        job_id = 'table-%s' % id_table
        report = comp(create_table_algos_discdds, allruns, id_table, job_id=job_id)
        basename = '%s/reports/table-%s' % (outdir, id_table)
        comp(write_report, report, basename, job_id=job_id + '-write')
    
