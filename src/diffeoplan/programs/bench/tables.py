from . import np
from bootstrapping_olympics.utils.not_found import raise_x_not_found
from compmake import comp
from contracts import contract
from diffeoplan.programs.bench.statistics import Stats
from reprep import Report
from reprep.report_utils.store_results import StoreResults
from compmake.ui.ui import comp_stage_job_id

def create_table_algos_discdds(allruns, id_table):
    algos = list(set(allruns.field('id_algo')))
    discdds = list(set(allruns.field('id_discdds')))
    
    data = np.zeros((len(algos), len(discdds)), dtype='S64')
    
    for i, id_algo in enumerate(algos):
        for j, id_discdds in enumerate(discdds):
            stats = allruns.select(id_algo=id_algo, id_discdds=id_discdds).values()
            value = Stats.tables[id_table](stats)
            data[i, j] = '%g' % value
             
    r = Report(id_table)
    r.table(id_table, data=data, cols=discdds, rows=algos, caption=id_table)
    return r

def results2stats_dict(results):
    """ Applies all possible statistics to the results dictionary. """
    res = {}
    for x in Stats.statistics:
        res[x] = Stats.statistics[x].function(results)
    return res
                                    
def create_tables(allstats, rm):
    for id_table in Stats.tables: 
        job_id = 'table-%s' % id_table
        report = comp(create_table_algos_discdds, allstats, id_table, job_id=job_id)
        rm.add(report, 'table', id_table=id_table)
    
def create_tables_by_sample(allstats, rm):
    for id_statstable in Stats.statstables:
        if len(Stats.statstables[id_statstable]) == 0:
            raise ValueError(Stats.statstables)
        for id_stats in Stats.statstables[id_statstable]:
            if not id_stats in Stats.statistics:
                raise_x_not_found('statistic', id_stats, Stats.statistics)

    for id_statstable in Stats.statstables:
        stats = Stats.statstables[id_statstable]
        job_id = 'bysample-all-%s' % id_statstable
        report = comp(create_tables_by_samples,
                      id_statstable, stats,
                      allstats, job_id=job_id)
        rm.add(report, 'bysample-all', id_statstable=id_statstable)
        
    
        testcases = list(set(allstats.field('id_tc')))
        for id_tc in testcases:
            tcruns = allstats.select(id_tc=id_tc)
            
            job_id = 'bysample-%s-%s' % (id_tc, id_statstable)
            r = comp(create_table_for_sample, id_tc, id_statstable, stats, tcruns,
                     job_id=job_id)

            rm.add(r, 'bysample-single', id_tc=id_tc, id_statstable=id_statstable)
            


@contract(id_stats='str', stats='list[>=1](str)', allruns=StoreResults)
def create_tables_by_samples(id_stats, stats, allruns):
    '''
    
    :param id_stats: name of this set
    :param stats: list of string corresponding to functions in statistics
    :param allruns: 
    '''
    r = Report(id_stats)
    testcases = sorted(list(set(allruns.field('id_tc'))))
    for id_tc in testcases:
        tcruns = allruns.select(id_tc=id_tc)
        rt = create_table_for_sample(id_tc, id_stats, stats, tcruns)
        r.add_child(rt)
    return  r
    
def create_table_for_sample(id_tc, id_stats, stats, tcruns):
    r = Report('%s-%s' % (id_stats, id_tc))
    algos = sorted(list(set(tcruns.field('id_algo'))))
    rows = algos
    cols = ["$%s$" % s.symbol for s in map(Stats.statistics.__getitem__, stats)]
    data = []
    for id_algo in algos:
        row = []
        for id_stats in stats:
            s = list(tcruns.select(id_algo=id_algo).values())
            assert len(s) == 1
#            value = Stats.statistics[id_stats].function(s[0])
            value = s[0][id_stats]
            row.append(value)
        data.append(row)
    r.table(id_tc, data=data, cols=cols, rows=rows, fmt='%g')
    return  r
        

    

        
        
