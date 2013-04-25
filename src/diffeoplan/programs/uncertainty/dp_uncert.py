'''
Created on Feb 14, 2013

@author: adam
'''
from . import logger
from compmake import (batch_command, compmake_console, comp, read_rc_files,
    use_filesystem)
from diffeoplan.library.images import UncertainImage
from diffeoplan.programs.distances.dp_dist_stats import (fancy_error_display,
    legend_put_below)
from diffeoplan.programs.distances.dp_pred_stats import compute_predstats
from diffeoplan.programs.utils import declare_command
from itertools import chain, starmap, islice, cycle
from quickapp import ReportManager
from reprep import Report
from reprep.report_utils import StoreResults
import numpy as np
import os
import pdb
import pylab

@declare_command('uncert', 'uncert  dds <stream1> [...]')
def uncert(config, parser):  # @UnusedVariable
    parser.add_option("-s", "--streams", help="Which streams to use.",
                      default="*")
    parser.add_option("-S", "--dds", help="DDS sytem .")
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-uncert/')
    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    options = parser.parse_options()
     
    dds = config.discdds.expand_names(options.dds) 
    streams = config.streams.expand_names(options.streams)
    
    id_comb = ",".join(dds) + "-" + ",".join(streams)
    
    outdir = os.path.join(options.output, id_comb) 
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
    
    rm = ReportManager(os.path.join(outdir, 'reports'))
    max_delta = 9
    
    store = create_uncert_stats_jobs(config, dds, streams, max_delta, outdir)
    
    records = comp(make_records, store)
    
    report = comp(report_uncert_stats, records, dds)
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
def create_uncert_stats_jobs(config, id_ddss, id_streams, max_delta, outdir):
    store = StoreResults()
    id_distances = ['L2', 'L2w']
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
    return records

dp_predstats_fig = dict(figsize=(6.6, 3))
def report_uncert_stats(records, id_ddss):

#    records = stats['records']

    r = Report('uncert-statsall')
    r.data('records', records)
    f = r.figure()
    
    id_distances = ['L2', 'L2w']
    
    colors = list(islice(cycle(['r', 'g', 'b', 'k', 'y', 'm']), 50))
    perc = 10
    W = 0.2

    with f.plot('distance', **dp_predstats_fig) as pylab:
        ax = pylab.subplot(111)
        for i, id_dds in enumerate(id_ddss):
            which = records['id_discdds'] == id_dds
            delta = records[which]['delta']
            
            distance = records[which]['L2w']
            
            if i == 0:
                distance0 = records[which]['L2']
                step = float(0) / max(len(id_distances) - 1, 1)
                xstep = W * 2 * (step - 0.5)
                fancy_error_display(ax, delta + xstep, distance0,
                                    colors[0], perc=perc,
                                    label='L2')

            step = float(i + 1) / max(len(id_distances) - 1, 1)
            xstep = W * 2 * (step - 0.5) 
            fancy_error_display(ax, delta + xstep, distance,
                                colors[i + 1], perc=perc, label='L2w' + id_dds)
    
        legend_put_below(ax)
    return r
# def uncert_stats(config, dds_id, streams, outdir):
#    
#    for stream_id in streams:
#        comp(calculate_uncert_stats, config, dds_id, stream_id, outdir)
#
# def calculate_uncert_stats(config, dds_id, stream_id, outdir):
#    dds = config.discdds.instance(dds_id)
#    stream = config.streams.instance(stream_id)
#    metric = config.distances.instance('L2')
#    
#    logitems = stream.read_all_state()
#    
#    for i, (y0, u, y1, x0) in enumerate(logitems):
# #        pdb.set_trace()
#        Y0 = UncertainImage(y0)
#        Y1 = UncertainImage(y1)
#        U0 = dds.command_to_index(u)
#        uimg = dds.predict(Y0, [U0])
#        
#        for j in [0, 1, 2, 3]:
#            pylab.figure()
#            pylab.imshow(np.sum(dds.predict(Y0, [j]).get_values() - Y0.get_values(), axis=2))
#            pylab.colorbar()
#            pylab.savefig(outdir + '/diff_' + str(j) + '.png')
#            pylab.clf()
#        
#        
#        
# #        diff0 = metric.distance(Y0, uimg)
# #        diff1 = metric.distance(Y1, uimg)
# #        
# #        logger.info(diff0, diff1)
#        
#        diff = uimg.get_values() - UncertainImage(y1).get_values()
#        diff_0 = uimg.get_values() - UncertainImage(y0).get_values()
#        variance = dds.actions[U0].diffeo.variance
#        diff = np.sum(diff, axis=2)
#        variance = variance / np.mean(variance)
#        diff = diff / np.mean(diff)
#        
#        res = variance - diff
#        logger.info('  mean(res) : ' + str(np.mean(res)))
#        logger.info('  std(res) : ' + str(np.std(res)))
#        
#        pylab.figure()
#        pylab.imshow(diff, vmin= -15, vmax=15)
#        pylab.savefig(outdir + 'diff' + str(i) + '.png')
#        pylab.clf()
#        pylab.imshow(diff, vmin= -15, vmax=15)
#        pylab.savefig(outdir + 'diff' + str(i) + '.png')
#        pylab.clf()
#        pylab.close()
#        pdb.set_trace()
