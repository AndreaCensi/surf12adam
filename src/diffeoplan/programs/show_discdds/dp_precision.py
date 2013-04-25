'''
Created on Feb 27, 2013

@author: adam
'''
import os
from compmake import (batch_command, compmake_console, comp, read_rc_files,
    use_filesystem)
from diffeoplan.programs.distances.dp_dist_stats import fancy_error_display, \
    legend_put_below
from diffeoplan.programs.distances.dp_pred_stats import compute_predstats
from diffeoplan.programs.utils import declare_command
from itertools import chain, starmap, islice, cycle
from reprep import Report
from reprep.report_utils import StoreResults
from quickapp import ReportManager
import numpy as np
import itertools

@declare_command('precision', 'precision  ...') #TODO:
def uncert(config, parser):
    parser.add_option("-S", "--dds", help="DDS sytem .")
    parser.add_option("-c", "--command", \
                      help="Command to pass to compmake for batch mode")
    parser.add_option("-o", "--output", help="Output directory", \
                      default='out/dp-precision/')
    parser.add_option("-s", "--streams", help="Which streams to use.", \
                      default="*")
    parser.add_option("-d", "--distances", default='L2,L2w', \
                      help="Distances id to use for comparing the diffeo systems")
    parser.add_option("-l", "--length", default=9, type='int', \
                      help="Max length of test cases")
    options = parser.parse_options()
     
    dds = config.discdds.expand_names(options.dds) 
    streams = config.streams.expand_names(options.streams)
    
    id_comb = ",".join(dds)
    
    outdir = os.path.join(options.output, id_comb) 
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
    
    rm = ReportManager(os.path.join(outdir, 'reports'))
    max_delta = options.length
    distances = config.distances.expand_names(options.distances)
    
    store = create_stats_jobs(config, dds, streams, distances, max_delta, outdir)
    
    records = comp(make_records, store)
    
    report = comp(report_stats, records, dds, streams, distances)
    r0 = comp(empty_report)
    rm.add(report, 'main', id_dds='dds')
    rm.add(r0, 'empty')
    
    rm.create_index_job()
    
    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0

def empty_report():
    return Report('empty')

def create_stats_jobs(config, id_ddss, id_streams, id_distances, max_delta, outdir):
    store = StoreResults()
    
    for id_dds in id_ddss:
        for id_stream in id_streams:
            for delta in range(1, max_delta):
                key = dict(id_discdds=id_dds, id_stream=id_stream, delta=delta)
                store[key] = comp(compute_predstats, config, id_dds, id_stream, delta, id_distances)
    return store

            
def make_records(results):
    def make_array(key, distances):
        dtype = [('delta', 'int'),
                 ('id_stream', 'S64'),
                 ('id_discdds', 'S64')]
        dtype += list(distances[0].dtype.descr)
        dtype = np.dtype(dtype)
        for distance in distances:
            fields = [key['delta'], key['id_stream'], key['id_discdds']]
            fields += map(lambda x: float(distance[x]), distance.dtype.fields)
            
            yield np.array(tuple(fields), dtype=dtype)

    records = chain.from_iterable(starmap(make_array, results.items()))
    records = list(records)
    records = np.hstack(records)
#    pdb.set_trace()
    return records

dp_predstats_fig = dict(figsize=(6.6, 3))
def report_stats(records, id_ddss, id_streams, id_distances):
    r = Report('precision-stats')
    r.data('records', records)
    
    colors = list(islice(cycle(['r', 'g', 'b', 'k', 'y', 'm']), 50))
    perc = 10
    W = 0.2
    
    for i, id_dds in enumerate(id_ddss):
        r.text('dds%s' % i, id_dds)
    
    
    streams_sets = generate_stream_sets(id_streams)
    for stream_set, id_distance in itertools.product(streams_sets, id_distances):
        f = r.figure(cols=2)
        with f.plot('distance_legend', caption='Streams: %s, Distance: %s' 
                    % (stream_set['label'], id_distance),
                    **dp_predstats_fig) as pylab:
            ax = pylab.subplot(111)
            for i, id_dds in enumerate(id_ddss):
                which = (records['id_discdds'] == id_dds).astype('int')
                for id_stream in stream_set['id_streams']:
                    which += (records['id_stream'] == id_stream).astype('int')
                which = (which / (len(stream_set['id_streams']) + 1)).astype('bool')
                
                delta = records[which]['delta']
                
                distance = records[which][id_distance]
                
                step = float(i) / max(len(id_ddss) - 1, 1)
                xstep = W * 2 * (step - 0.5) 
                fancy_error_display(ax, delta + xstep, distance,
                                    colors[i], perc=perc, label='%s' % i)
            
        with f.plot('distance', caption='treams: %s, Distance: %s' 
                    % (stream_set['label'], id_distance),
                    **dp_predstats_fig) as pylab:
            ax = pylab.subplot(111)
            for i, id_dds in enumerate(id_ddss):
                which = (records['id_discdds'] == id_dds).astype('int')
                for id_stream in stream_set['id_streams']:
                    which += (records['id_stream'] == id_stream).astype('int')
                which = (which / (len(stream_set['id_streams']) + 1)).astype('bool')
                
                delta = records[which]['delta']
                
                distance = records[which][id_distance]
                
                step = float(i) / max(len(id_ddss) - 1, 1)
                xstep = W * 2 * (step - 0.5) 
                fancy_error_display(ax, delta + xstep, distance,
                                    colors[i], perc=perc, label='%s' % i)
            
            legend_put_below(ax)
            
        
        with f.plot('difference', caption='Difference from learner_0, SStreams: %s, Distance: %s' 
                    % (stream_set['label'], id_distance),
                    **dp_predstats_fig) as pylab:
            ax = pylab.subplot(111)
            
            which0 = (records['id_discdds'] == id_ddss[0])
            _ = records[which0]['delta'] # delta0
            distance0 = records[which0][id_distance]
            for i, id_dds in enumerate(id_ddss[1:]):
                which = (records['id_discdds'] == id_dds).astype('int')
                for id_stream in stream_set['id_streams']:
                    which += (records['id_stream'] == id_stream).astype('int')
                which = (which / (len(stream_set['id_streams']) + 1)).astype('bool')
                
                delta = records[which]['delta']
                
                distance = records[which][id_distance]
                difference = distance0 - distance
                
                step = float(i) / max(len(id_ddss) - 1, 1)
                xstep = W * 2 * (step - 0.5) 
                fancy_error_display(ax, delta + xstep, difference,
                                    colors[i + 1], perc=perc, label='%s' % i)
    return r

def generate_stream_sets(id_streams):
    sets = []
    sets.append(dict(label='all', id_streams=id_streams))
    for id_stream in id_streams:
        sets.append(dict(label=id_stream, id_streams=[id_stream]))
    return sets
