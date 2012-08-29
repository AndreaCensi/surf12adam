from .. import declare_command, logger
from boot_agents.utils import scale_score
from compmake import (batch_command, compmake_console, comp, read_rc_files,
    use_filesystem)
from diffeoplan.library.images import UncertainImage
from diffeoplan.programs.logcases.makelogcases import iterate_testcases
from diffeoplan.utils import UserError
from geometry.utils import assert_allclose
from itertools import chain, starmap, islice, cycle
from reprep import Report
from reprep.plot_utils import ieee_spines
from reprep.report_utils import StoreResults, ReportManager
import numpy as np
import os

@declare_command('dist-stats',
                 'dist-stats -d <distances> [logs]')
def distances_main(config, parser): #@UnusedVariable
    """ FPS statistics for distances """
    parser.add_option("-d", "--distances", default='*',
                      help="Comma-separated list of distances. Can use *.")
    
    parser.add_option("-r", "--repeat", default=1, type='int',
                       help="Repeat many times.")

    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    
    options, args = parser.parse()
    
    logs = args
    
    if not logs:
        msg = 'Specify logs.'
        raise UserError(msg)
    
    distances = config.distances.expand_names(options.distances)

    logger.info('Using distances: %s' % distances)
    
    outdir = 'out/distances'
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
    rm = ReportManager(os.path.join(outdir, "reports"))
    
    create_diststats_jobs(config=config, distances=distances, logs=logs,
                          outdir=outdir, rm=rm)
    rm.create_index_job()

    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0
    
def create_diststats_jobs(config, distances, logs, outdir, rm, maxd=10):
    # Compmake storage for results
    store = StoreResults()

    for id_distance in distances:    
        for delta in range(1, maxd):
            for i, log in enumerate(logs):
                key = dict(id_distance=id_distance,
                           delta=delta,
                           log=log)
                job_id = '%s-log%s-delta%s' % (id_distance, i, delta)
                
                store[key] = comp(compute_dist_stats, config, id_distance,
                                  log, delta,
                                  job_id=job_id)
    
    
    for id_distance in distances:
        subset = store.select(id_distance=id_distance)
        stats = comp(compute_statistics, subset)
        report = comp(report_statistics, id_distance, stats)
        rm.add(report, 'bydistance', id_distance=id_distance)

    subsets = {}
    subsets['all'] = sorted(distances)
    
    initials = set(d[0] for d in distances)
    for initial in initials:
        which = filter(lambda x: x[0] == initial, distances)
        subsets[initial] = sorted(which)
        
    job_report(subsets, store, rm)
    
def job_report(subsets, store, rm):
    for id_subset, which in subsets.items():
        logger.info('%s = %s' % (id_subset, which))
        subset = store.select(lambda x: x['id_distance'] in which)
        logger.info('selected %s' % len(subset))
        substats = comp(compute_statistics, subset, job_id='%s-s' % id_subset)
        report = comp(report_statistics_all, id_subset, substats)
        rm.add(report, 'main', subset=id_subset)
            
    
        
def report_statistics_all(id_sub, stats, perc=10, W=0.2):
    records = stats['records']

    r = Report('statsall-%s' % id_sub)
    r.data('records', records)
    f = r.figure()
    
    id_distances = sorted(set(records['id_distance']))
        
    logger.info('%s: %s %s reo %s' % (id_sub, len(stats), id_distances,
                                      len(records)))

    with f.plot('with_stats') as pylab:
        colors = list(islice(cycle(['r', 'g', 'b', 'k', 'y', 'm']), 50))
        ax = pylab.subplot(111)

        for i, id_d in enumerate(id_distances):
            which = records['id_distance'] == id_d
            delta = records[which]['delta']
            distance = records[which]['distance']
            order = scale_score(distance)
            order = order / float(order.size)

            
            step = float(i) / (len(id_distances) - 1)
            xstep = W * 2 * (step - 0.5) 
            fancy_error_display(ax, delta + xstep, order,
                                colors[i], perc=perc, label=id_d)
            
        ieee_spines(pylab)    
        ticks = sorted(list(set(list(delta))))
        pylab.xlabel('plan length')
        pylab.ylabel('normalized distance')
        pylab.xticks(ticks, ticks)
        pylab.yticks((0, 1), (0, 1))
        pylab.axis((0.5, 0.5 + np.max(delta), -0.024, 1.2))
        legend_put_below(ax)

    return r

def report_statistics(id_sub, stats):
    records = stats['records']
    distance = records['distance']
    delta = records['delta']
    order = scale_score(distance)
    order = order / float(order.size)

    r = Report('stats-%s' % id_sub)
    r.data('records', records)
    f = r.figure()
    
    with f.plot('scatter') as pylab:
        pylab.scatter(delta, distance)
        pylab.xlabel('delta')
        pylab.ylabel('distance')
        pylab.axis((-1, np.max(delta) + 1, -0.05, np.max(distance)))
        
    with f.plot('with_stats') as pylab:
        fancy_error_display(pylab, delta, distance, 'g')

    with f.plot('distance_order') as pylab:
        fancy_error_display(pylab, delta, order, color='k')
        
    f = r.figure(cols=1)        
    bins = np.linspace(0, np.max(distance), 100)
    for i, d in enumerate(set(delta)):
        with f.plot('conditional%d' % i) as pylab:
            which = delta == d
            pylab.hist(distance[which], bins)

    return r

def fancy_error_display(pylab, xs, ys, color, perc=10, label=None):
#    gray = [0.5, 0.5, 0.5]
    
    pylab.scatter(xs, ys, s=20, c=color, edgecolors='none', alpha=0.01)

    for i, x in enumerate(set(xs)):
        which = xs == x
        
        # only assign the label to the first one
        if i == 0:
            kwargs = dict(label=label)
        else:
            kwargs = dict()
        p = fancy_errorbar(pylab, x, ys[which], perc, fmt='o', ecolor=color,
                       mfc=color, mec=color, **kwargs)
    return p

def fancy_errorbar(pylab, x, ys, p, *args, **kwargs):
    y_mean = np.mean(ys)
    above = np.percentile(ys, 100 - p) - y_mean
    below = y_mean - np.percentile(ys, p) 
    yerr = np.vstack((-above, -below))
    
    p = pylab.errorbar(x, y_mean, yerr=yerr, *args, **kwargs)
    return p

def legend_put_below(ax, frac=0.1):
#    ax = pylab.gca()
    # Shink current axis's height by 10% on the bottom
    box = ax.get_position()
    ax.set_position([box.x0, box.y0 + box.height * frac,
                 box.width, box.height * (1 - frac)])

    # Put a legend below current axis
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1),
              fancybox=True, shadow=True, ncol=5)

#    ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

def compute_statistics(results):
    
    def make_array(key, distances):
        for distance in distances:
            yield np.array(
                           (key['delta'], distance, key['id_distance']),
                           dtype=[('delta', 'int'),
                                  ('distance', 'float'),
                                  ('id_distance', 'S64')])
        

    records = chain.from_iterable(starmap(make_array, results.items()))
    records = np.hstack(list(records))
    stats = {}
    stats['records'] = records
    return stats

def compute_dist_stats(config, id_distance, log, delta):
    distance = config.distances.instance(id_distance)
    
    results = []
    for logitem in iterate_testcases(log, delta):
        assert_allclose(len(logitem.u), delta)
        y0 = UncertainImage(logitem.y0)
        y1 = UncertainImage(logitem.y1)
        d = distance.distance(y0, y1)
        results.append(d)
        
    logger.info('%s: found %d of %d steps in %s' % 
                (id_distance, len(results), delta, log))
    return results
        
        
        
