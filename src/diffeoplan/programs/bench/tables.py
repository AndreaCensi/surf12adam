from . import Stats, contract
from compmake import comp
from itertools import product
from reprep.report_utils import ReportManager, StoreResults, table_by_rows
from compmake.ui.user_utils import comp_store


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
    
    # TODO: group also by discdds

    return sample_groups


def jobs_tables(allstats, rm):
    # Reports per-t
    tables_single = Stats.tables_for_single_sample
    tables_multi = Stats.tables_for_multi_sample
    jobs_tables_by_sample_rows_algo(allstats, rm, tables_single)
    jobs_tables_by_algo_rows_samples(allstats, rm, tables_single)
    
    samples_groups = make_samples_groups(allstats)
    jobs_tables_by_sample_groups(samples_groups, rm, tables_multi)
    jobs_tables_by_algo_rows_sample_groups(samples_groups, rm, tables_multi)
        
        
def jobs_tables_by_sample_groups(samples_groups, rm, tables):
    source_descs = comp_store(Stats.all_descriptions())
    # Tables grouping by algorithm
    for g, s in product(samples_groups.items(), tables.items()):
        id_sample_group, samples = g
        id_statstable, stats = s
        
        r = comp(table_by_rows,
                 "bysamplegroups-%s-%s" % (sanitize(id_sample_group), id_statstable),
                 samples=samples,
                 rows_field='id_algo', # group by algorithm
                 cols_fields=stats, # which statistics for each col
                 source_descs=source_descs)

        report_attrs = dict(id_sample_group=id_sample_group,
                            id_stats_table=id_statstable)
        
        report_attrs.update(samples.fields_with_unique_values())
        
        rm.add(r, 'bysamplegroups', **report_attrs)
        
def sanitize(s):
    s = s.replace('=', '-')
    return s

@contract(allstats=StoreResults, rm=ReportManager,
          tables='dict(str:list(str))')
def jobs_tables_by_sample_rows_algo(allstats, rm, tables):
    source_descs = comp_store(Stats.all_descriptions())
    
    for id_statstable, stats in tables.items():
        for id_tc, tcruns in allstats.groups_by_field_value('id_tc'):
            job_id = 'bysample-%s-%s' % (id_tc, id_statstable)

            r = comp(table_by_rows,
                     "bysample-%s-%s" % (id_tc, id_statstable),
                     samples=tcruns,
                     rows_field='id_algo', # group by algorithm
                     cols_fields=stats, # which statistics for each col
                     source_descs=source_descs,
                     job_id=job_id)
            
            report_attrs = dict(id_statstable=id_statstable) # id_tc=id_tc, 
            report_attrs.update(tcruns.fields_with_unique_values())

            rm.add(r, 'bysample', **report_attrs)
            

@contract(allstats=StoreResults, rm=ReportManager,
          tables='dict(str:list(str))')
def jobs_tables_by_algo_rows_samples(allstats, rm, tables):
    """ One table for each algo, where rows are test cases. """
    source_descs = comp_store(Stats.all_descriptions())
    for id_statstable, stats in tables.items():
        for id_algo, samples in allstats.groups_by_field_value('id_algo'):
            job_id = 'byalgo-%s-%s' % (id_algo, id_statstable)

            r = comp(table_by_rows,
                     "byalgo-rows-sample-%s-%s" % (id_algo, id_statstable),
                     samples=samples,
                     rows_field='id_tc', # rows = tc
                     cols_fields=stats, # which statistics for each col
                     source_descs=source_descs,
                     job_id=job_id)

            report_attrs = dict(id_statstable=id_statstable)
            report_attrs.update(samples.fields_with_unique_values())
            assert report_attrs['id_algo'] == id_algo
            
            rm.add(r, 'byalgo-rows-sample', **report_attrs)
 

@contract(samples_groups='dict(str:StoreResults)', rm=ReportManager,
          tables='dict(str:list(str))')
def jobs_tables_by_algo_rows_sample_groups(samples_groups, rm, tables):
    source_descs = comp_store(Stats.all_descriptions())
    
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
                     "byalgo-rows-sample-groups-%s-%s" % (id_algo, id_statstable),
                     samples=samples,
                     rows_field='id_group', # rows = tc
                     cols_fields=stats, # which statistics for each col
                     source_descs=source_descs,
                     job_id=job_id) 
            
            report_attrs = dict(id_statstable=id_statstable)
            report_attrs.update(samples.fields_with_unique_values())
            assert report_attrs['id_algo'] == id_algo
           
            rm.add(r, 'byalgo-rows-sample-groups', **report_attrs)
 
