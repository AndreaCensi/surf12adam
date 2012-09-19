from .. import declare_command, logger
from boot_agents.utils import scale_score
from compmake import (batch_command, compmake_console, comp, read_rc_files,
    use_filesystem)
from diffeoplan.library import UncertainImage
from diffeoplan.programs.distances import (fancy_error_display, legend_put_below,
    create_subsets)
from diffeoplan.programs.logcases import iterate_testcases
from diffeoplan.utils import UserError
from geometry.utils import assert_allclose
from itertools import chain, starmap, islice, cycle
from reprep import Report
from reprep.plot_utils import ieee_spines
from reprep.report_utils import StoreResults, ReportManager
import numpy as np
import os
from diffeoplan.programs.distances.dp_dist_stats import dp_predstats_fig

@declare_command('pred-stats',
                 'pred-stats -d <distances> -s <streams> --dds discdds')
def dp_predstats_main(config, parser): 
    
    parser.add_option("-o", "--output", default='out/dp-pred-stats',
                      help="Output directory")

    parser.add_option("-d", "--distances", default='*',
                      help="Comma-separated list of distances. Can use *.")
    
    parser.add_option("-S", "--dds",
                       help="Comma-separated list of diffeosystems.")

    parser.add_option("-s", "--streams",
                       help="Comma-separated list of streams.")

    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    
    options = parser.parse_options()
    
    id_discdds = options.dds
    
    if not options.streams:
        msg = 'Please specify streams using -s.'
        raise UserError(msg)
    
    if not id_discdds:
        msg = 'Please specify which discdds to use.'
        raise UserError(msg)
    
    distances = config.distances.expand_names(options.distances)
    streams = config.streams.expand_names(options.streams)
    
    logger.info('Using distances: %s' % distances)
    logger.info('Using streams: %s' % streams)
    logger.info('Using discdds: %s' % id_discdds)
    
    outdir = '%s/%s' % (options.output, id_discdds)
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
    rm = ReportManager(os.path.join(outdir, "reports"))
    
    create_predstats_jobs(config=config, distances=distances,
                          id_discdds=id_discdds,
                          streams=streams, rm=rm, maxd=10)
    rm.create_index_job()

    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0
    
def create_predstats_jobs(config, distances, streams, id_discdds, rm, maxd):
    # Compmake storage for results
    store = StoreResults()
    
    for delta in range(1, maxd):
        for i, id_stream in enumerate(streams):
            key = dict(delta=delta, id_stream=id_stream)
            job_id = 'pred-log%s-delta%s' % (i, delta)
            
            store[key] = comp(compute_predstats,
                              config, id_discdds,
                              id_stream, delta, distances,
                              job_id=job_id)
     
    subsets = create_subsets(distances)
    
    job_report(subsets, id_discdds, store, rm)
    

def job_report(subsets, id_discdds, store, rm):
    records = comp(make_records, store)
    for id_subset, distances in subsets.items():
        job_id = 'report_predstats-%s' % id_subset    
        report = comp(report_predstats, id_discdds, id_subset, distances, records,
                      job_id=job_id)
        rm.add(report, 'main', subset=id_subset)
            
            
            
def make_records(results):
     
    def make_array(key, distances):
        dtype = [('delta', 'int'),
                 ('id_stream', 'S64')]
        dtype += list(distances[0].dtype.descr)
        dtype = np.dtype(dtype)
        for distance in distances:
            fields = [key['delta'], key['id_stream']]
            fields += map(lambda x: float(distance[x]), distance.dtype.fields)
            
            yield np.array(tuple(fields), dtype=dtype)

    records = chain.from_iterable(starmap(make_array, results.items()))
    records = list(records)
    records = np.hstack(records)
    return records


def report_predstats(id_discdds, id_subset, id_distances, records):
    r = Report('predistats-%s-%s' % (id_discdds, id_subset))
    
    r.data('records', records)
    f = r.figure()
    
    colors = list(islice(cycle(['r', 'g', 'b', 'k', 'y', 'm']), 50))
    delta = records['delta']
    W = 0.2

    # Save the raw values
    for i, id_d in enumerate(id_distances):
        r.data(id_d, records[id_d])
    
    with f.plot('values_order', **dp_predstats_fig) as pylab:
        ax = pylab.subplot(111)

        for i, id_d in enumerate(id_distances):
            distance = records[id_d]
            distance_order = scale_score(distance) / (float(distance.size) - 1)
            
            step = float(i) / max(len(id_distances) - 1, 1)
            xstep = W * 2 * (step - 0.5) 
            fancy_error_display(ax, delta + xstep, distance_order,
                                colors[i], perc=10, label=id_d)
            
        ieee_spines(pylab)    
        ticks = sorted(list(set(list(delta))))
        pylab.xlabel('interval length')
        pylab.ylabel('normalized distance')
        pylab.xticks(ticks, ticks)
        pylab.yticks((0, 1), (0, 1))
        pylab.axis((0.5, 0.5 + np.max(delta), -0.024, 1.2))
        legend_put_below(ax)

    with f.plot('values', **dp_predstats_fig) as pylab:
        ax = pylab.subplot(111)

        for i, id_d in enumerate(id_distances):
            distance = records[id_d]
            
            step = float(i) / max(len(id_distances) - 1, 1)
            xstep = W * 2 * (step - 0.5) 
            fancy_error_display(ax, delta + xstep, distance,
                                colors[i], perc=10, label=id_d)
            
        ieee_spines(pylab)    
        ticks = sorted(list(set(list(delta))))
        pylab.xlabel('interval length')
        pylab.ylabel('distance')
        pylab.xticks(ticks, ticks)
#        pylab.yticks((0, 1), (0, 1))
        a = pylab.axis()
        pylab.axis((0.5, 0.5 + np.max(delta), -0.024, a[3]))
        legend_put_below(ax)

    return r
     
def compute_predstats(config, id_discdds, id_stream, delta, id_distances):
    dds = config.discdds.instance(id_discdds)
    stream = config.streams.instance(id_stream)
    distances = dict(map(lambda x: (x, config.distances.instance(x)), id_distances))
    dtype = [(x, 'float32') for x in id_distances]
    
    results = []
    for logitem in iterate_testcases(stream.read_all(), delta):
        assert_allclose(len(logitem.u), delta)
        y0 = UncertainImage(logitem.y0)
        y1 = UncertainImage(logitem.y1)
        py0 = dds.predict(y0, dds.commands_to_indices(logitem.u))
        ds = []
        for name in id_distances:
            d = distances[name].distance(y1, py0)
#            d0 = distances[name].distance(y1, y0)
            ds.append(d)
        
        a = np.array(tuple(ds), dtype=dtype)
        results.append(a)
        
    return results


        
