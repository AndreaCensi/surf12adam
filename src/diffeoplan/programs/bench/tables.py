from . import write_report, np
from compmake import comp
from reprep import Report
import itertools
from scipy.stats.stats import nanmean, nanstd
from diffeoplan.programs.bench.statistics import Stats
from bootstrapping_olympics.utils.not_found import raise_x_not_found
from contracts import contract
from reprep.report_utils.store_results import StoreResults


reductions = {}
reductions['min'] = np.nanmin
reductions['max'] = np.nanmax
reductions['mean'] = nanmean
reductions['stddev'] = nanstd 
    
def f(stats, s, r):
    reduce_ = reductions[r]
    map_ = Stats.statistics[s].function
    #print('computing %s %s' % (s, r))
    return reduce_([map_(x) for x in stats])

tables = {}
for a, b in itertools.product(Stats.statistics, reductions):
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
    
def create_tables_by_sample(outdir, allruns):
    statstables = {'all': list(Stats.statistics.keys()),
                   'graph': ['plan_found',
                             'num_closed',
                             'num_open',
                             'num_created',
                             'num_redundant',
                             'num_collapsed'],
                   'graph_details': ['plan_found',
                             'num_closed',
                             'num_open',
                             'num_created',
                             'num_redundant',
                             'num_collapsed',
                             'num_start_closed',
                             'num_start_open',
                             'num_start_created',
                             'num_start_redundant',
                             'num_start_collapsed',
                             'num_goal_closed',
                             'num_goal_open',
                             'num_goal_created',
                             'num_goal_redundant',
                             'num_goal_collapsed',
                             ]}

    # make sure we know them
    for id_statstable in statstables:
        for id_stats in statstables[id_statstable]:
            if not id_stats in Stats.statistics:
                raise_x_not_found('statistic', id_stats, Stats.statistics)


    for id_statstable in statstables:
        job_id = 'bysample-%s' % id_statstable
        report = comp(create_tables_by_samples,
                      id_statstable, statstables[id_statstable],
                      allruns, job_id=job_id)
        basename = '%s/reports/bysample_%s' % (outdir, id_statstable)
        comp(write_report, report, basename, job_id=job_id + '-write')


@contract(id_stats='str', stats='list[>=1](str)', allruns=StoreResults)
def create_tables_by_samples(id_stats, stats, allruns):
    '''
    
    :param id_stats: name of this set
    :param stats: list of string corresponding to functions in statistics
    :param allruns: 
    '''
    r = Report(id_stats)
    testcases = sorted(list(set(allruns.field('id_tc'))))
    algos = sorted(list(set(allruns.field('id_algo'))))
    # for each sample
    for id_tc in testcases:
        rows = algos
        cols = ["$%s$" % s.symbol for s in map(Stats.statistics.__getitem__, stats)]
        data = []
        for id_algo in algos:
            row = []
            for id_stats in stats:
                s = list(allruns.select(id_algo=id_algo, id_tc=id_tc).values())
                assert len(s) == 1
                value = Stats.statistics[id_stats].function(s[0])
                row.append(value)
            data.append(row)
        r.table(id_tc, data=data, cols=cols, rows=rows, fmt='%g')
    
    return  r
    
    

        
        
