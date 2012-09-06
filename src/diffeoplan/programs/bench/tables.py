from . import Stats, contract
from compmake import comp
from itertools import product
from reprep.report_utils import ReportManager, StoreResults, table_by_rows


def results2stats_dict(results):
    """ Applies all possible statistics to the results (output of run_planning_stats). """
    res = {}
    for x in Stats.statistics:
        res[x] = Stats.statistics[x].function(results)
    return res

def make_samples_groups(allstats):
    """ Creates lists of samples. Also supports single samples. 
    
        Return a dictionary str -> StoreResults.
    """
    sample_groups = {}
    sample_groups['all'] = allstats
    
    # First we create samples by deltas
    for delta, samples in allstats.groups_by_field_value('true_plan_length'):
        sample_groups['true_plan_length=%s' % delta] = samples
    
    # TODO: group by discdds

    # TODO: then we make all single samples
    print sample_groups.keys()
    return sample_groups

def jobs_tables(allstats, rm):
    # Reports per-t
    jobs_tables_by_sample_rows_algo(allstats, rm, Stats.tables_for_single_sample)
    jobs_tables_by_algo_rows_samples(allstats, rm, Stats.tables_for_single_sample)
    
    samples_groups = make_samples_groups(allstats)
    jobs_tables_by_sample_groups(samples_groups, rm,
                                 Stats.tables_for_multi_sample)
    
    jobs_tables_by_algo_rows_sample_groups(samples_groups, rm, Stats.tables_for_multi_sample)
        
def jobs_tables_by_sample_groups(samples_groups, rm, tables):
    # Tables grouping by algorithm
    for g, s in product(samples_groups.items(), tables.items()):
        id_sample_group, samples = g
        id_statstable, stats = s
        
        table_key = dict(id_sample_group=id_sample_group,
                         id_stats_table=id_statstable,
                         id_reduction='mean_std')
        
        r = comp(table_by_rows,
                 samples=samples,
                 rows_field='id_algo', # group by algorithm
                 cols_fields=stats, # which statistics for each col
                 source_descs=Stats.all_descriptions())
        
        rm.add(r, 'bysamplegroups', **table_key)
        
@contract(allstats=StoreResults, rm=ReportManager,
          tables='dict(str:list(str))')
def jobs_tables_by_sample_rows_algo(allstats, rm, tables):
    for id_statstable, stats in tables.items():
        for id_tc, tcruns in allstats.groups_by_field_value('id_tc'):
            job_id = 'bysample-%s-%s' % (id_tc, id_statstable)

            r = comp(table_by_rows,
                     samples=tcruns,
                     rows_field='id_algo', # group by algorithm
                     cols_fields=stats, # which statistics for each col
                     source_descs=Stats.all_descriptions(),
                     job_id=job_id)

            rm.add(r, 'bysample', id_tc=id_tc, id_statstable=id_statstable)
            
@contract(allstats=StoreResults, rm=ReportManager,
          tables='dict(str:list(str))')
def jobs_tables_by_algo_rows_samples(allstats, rm, tables):
    """ One table for each algo, where rows are test cases. """
    for id_statstable, stats in tables.items():
        for id_algo, samples in allstats.groups_by_field_value('id_algo'):
            job_id = 'byalgo-%s-%s' % (id_algo, id_statstable)

            r = comp(table_by_rows,
                     samples=samples,
                     rows_field='id_tc', # rows = tc
                     cols_fields=stats, # which statistics for each col
                     source_descs=Stats.all_descriptions(),
                     job_id=job_id)

            rm.add(r, 'byalgo-rows-sample',
                   id_algo=id_algo, id_statstable=id_statstable)
 
@contract(samples_groups='dict(str:StoreResults)', rm=ReportManager,
          tables='dict(str:list(str))')
def jobs_tables_by_algo_rows_sample_groups(samples_groups, rm, tables):
    
    # Crate a new store, add the key "group"
    allstats = StoreResults()
    for id_group, samples in samples_groups.items():
        for key, value in samples.items():
            nkey = dict(id_group=id_group, **key)
            allstats[nkey] = value
    
    for id_statstable, stats in tables.items():
        for id_algo, samples in allstats.groups_by_field_value('id_algo'):
            job_id = 'byalgo-%s-%s' % (id_algo, id_statstable)

            r = comp(table_by_rows,
                     samples=samples,
                     rows_field='id_group', # rows = tc
                     cols_fields=stats, # which statistics for each col
                     source_descs=Stats.all_descriptions(),
                     job_id=job_id)

            rm.add(r, 'byalgo-rows-sample-groups',
                   id_algo=id_algo, id_statstable=id_statstable)
 
