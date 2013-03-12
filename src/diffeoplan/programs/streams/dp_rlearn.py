'''
Created on Nov 7, 2012

@author: adam
'''
from . import logger
from boot_agents.diffeo import diffeomorphism2d_continuous
from compmake import (batch_command, compmake_console, comp, read_rc_files, use_filesystem)
from diffeoplan.library.discdds.writing import ds_dump
from diffeoplan.programs.streams.dp_plearn import filter_commands, report_dds, report_learner, summarize
from diffeoplan.programs.utils import declare_command
from reprep import Report
from reprep.report_utils import ReportManager, StoreResults
import numpy as np
import os
import sys
import time
import pdb
from compmake import CompmakeGlobalState
from compmake.jobs import get_job_cache
import pickle

@declare_command('rlearn',
                 'rlearn  [<stream1> ...]')
def rlearn(config, parser): #@UnusedVariable
    t0 = time.time()
    """ Displays the learned DDS """
    parser.add_option("-n", "--nthreads", help="Number of threads",
                      type='int', default='4')
    parser.add_option("-r", "--nrefine", help="Number of time to refine learning",
                      type='int', default='2')
    parser.add_option("-s", "--streams", help="Which streams to use.",
                      default="*")
    parser.add_option("-i", "--comb", default="default")
    parser.add_option("-l", "--learners", help="Learner config.", default="*")
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-rlearn/')
    parser.add_option("--sensels", default=None,
                      help="Sensel indices to use for debuging refining module")
    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    parser.add_option("--show", default=None, help="Name of learners to report")


    options = parser.parse_options()
    if options.show is not None:
        diffeomorphism2d_continuous.make_report(options.show.split(','))
        sys.exit()
    nthreads = options.nthreads
    nrefine = options.nrefine
    
    learners = config.learners.expand_names(options.learners) 
    streams = config.streams.expand_names(options.streams)
    
    if len(learners) > 1:
        logger.warn('Multiple learners are not supported for now')
    if len(streams) > 1:
        logger.warn('Multiple streams are not supported for now')
    
    id_comb = ",".join(streams) + "-" + ",".join(learners)
    outdir = os.path.join(options.output, id_comb) 
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
    rm = ReportManager(os.path.join(outdir, 'reports'))
    
    diffeo_system = jobs_rlearn(config, rm, learners, streams, outdir, nthreads, nrefine, options.sensels)
    
    # Time and report the learning
    learn_times = comp(learning_times_rlearn, outdir, learners, streams, nthreads, nrefine)
    
    
    rm.create_index_job()
    
    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0
    logger.info("Done after time: " + str(time.time() - t0) + ' seconds')
    
def learning_times_rlearn(outdir, learners, streams, nthreads, nrefines):
    
    job_ids = CompmakeGlobalState.jobs_defined_in_this_session
    id_learner, id_stream = learners[0], streams[0]

    cputime_index_level_learn = np.ones((nthreads, nrefines)) * np.NaN
    walltime_index_level_learn = np.ones((nthreads, nrefines)) * np.NaN
    for i in range(nthreads):
        for ref in range(nrefines):
            search_id = 'learn-%s-%s-%sof%s-refined%s' % (id_stream, id_learner, i + 1, nthreads, ref)
            for job_id in job_ids:
                if job_id == search_id:
                    job_cache = get_job_cache(job_id)
                    cputime_index_level_learn[i, ref] = job_cache.cputime_used
                    walltime_index_level_learn[i, ref] = job_cache.walltime_used
                    
    print cputime_index_level_learn
    print walltime_index_level_learn
    
    cputime_index_level_summarize = np.ones((nthreads, nrefines)) * np.NaN
    walltime_index_level_summarize = np.ones((nthreads, nrefines)) * np.NaN
    for i in range(nthreads):
        for ref in range(nrefines):
            search_id = 'learn-%s-%s-%s-refined%s-summarize' % (id_stream, id_learner, i , ref) # bug in computation naming, should be i+1
            print(search_id)
            for job_id in job_ids:
                if job_id == search_id:                    
                    job_cache = get_job_cache(job_id)
                    cputime_index_level_summarize[i, ref] = job_cache.cputime_used
                    walltime_index_level_summarize[i, ref] = job_cache.walltime_used
                    
    print cputime_index_level_summarize
    print walltime_index_level_summarize
    
    times = {'cputime_index_level_learn': cputime_index_level_learn,
             'walltime_index_level_learn': walltime_index_level_learn,
             'cputime_index_level_summarize': cputime_index_level_summarize,
             'walltime_index_level_summarize': walltime_index_level_summarize }
    
#    pdb.set_trace()
    pickle.dump(times, open(os.path.join(outdir, 'times.pickle'), 'wb'))
    
    
    
def jobs_rlearn(config, rm, learners, streams, outdir, nthreads, nrefine, sensels=None):
    # Loop over different commands/threads
    # This agent will be limited to learn <nthreads> different commands, 
    # additional commands will be ignored.
    diffeo_learners = []
    sub_learners = {}
    
#    store = StoreResults()
    store = dict()
    
    for i in range(nthreads):
        areas = None
        learner_i = None
        for ref in range(nrefine):
            logger.info('using area %s for initialization' % areas)
            learner_i, areas = jobs_rlearn_refine_level(config, rm, learners,
                                                        streams, outdir,
                                                        nthreads, i, ref, areas, learner_i)
#            key = dict(thread=i, ref=ref)
            key = (i, ref)
            store[key] = dict(learner=learner_i, areas=areas) 
#            this_result = comp(comp_append, this_result, (learner_i, areas))
##            this_result.append((learner_i, areas))
#            
#        if sensels is not None:
#            comp(report_area_propagation, this_result, sensels,
#                 'areas-learner-%s-%s-%s-of-%s' % (learners[0], streams[0], i, nthreads))
        
        # Finally append the last learner_i
        diffeo_learners.append(learner_i)

    # Only the first one because it only supports one stream and one learner for now 
    id_learner = learners[0]
    id_stream = streams[0]
    
    # Merge all learners on each level
    for ref in range(nrefine):
        level_learners = []
        for i in range(nthreads):
#            key = dict(thread=i, ref=ref)
            key = (i, ref)
#            pdb.set_trace()
            learner_ir = store[key]['learner']
            level_learners.append(learner_ir)
        level_learner = comp(merge_learners, level_learners)
        level_system = comp(summarize, level_learner,
                            job_id='learn-summarize-refined%s' % ref)
        level_dds_report = comp(report_dds, 'dds-refined%s' % ref, level_system,
                                job_id='learn-summarize-refined%s-report' % ref)
        comp(save_results, id_learner, id_stream, outdir, level_system, ref,
             job_id='learn-refined%s-%s-%s-summarize-save' % (ref, id_stream, id_learner))
        rm.add(level_dds_report, 'dds-refined%s' % ref,
               id_learner='refined%s' % ref, id_stream='all')
    
    # Merge all learners on the last level
    final_learner = comp(merge_learners, diffeo_learners)
    
    diffeo_system = comp(summarize, final_learner, job_id='learn-summarize')

    learner_report = comp(report_learner, 'learner', final_learner,
                          job_id='learn-report')
    
    diffeo_report = comp(report_dds, 'dds', diffeo_system,
                         job_id='learn-summarize-report')

    comp(save_results, id_learner, id_stream, outdir, diffeo_system,
         job_id='learn-%s-%s-summarize-save' % (id_stream, id_learner))
    
    rm.add(learner_report, 'learner', id_learner='final', id_stream='all')
    rm.add(diffeo_report, 'dds', id_learner='final', id_stream='all')
    
        
#def comp_append(original, new):
#    return original.append(new)
        
def jobs_rlearn_refine_level(config, rm, learners, streams, outdir, nthreads, i, ref, areas, parent):
#    learners_i = []
#    pdb.set_trace()
    id_learner, id_stream = learners[0], streams[0]
#    for id_learner, id_stream in itertools.product(learners, streams):
    # try instancing them
    config.streams.instance(id_stream)
    
    # Learn the first refining iteration
    job_id = 'learn-%s-%s-%sof%s-refined%s' % (id_stream, id_learner, i + 1, nthreads, ref)

    learner_i = comp(rlearn_partial, config, id_learner, id_stream,
                     i, nthreads, job_id=job_id, search_areas=areas, parent=parent)

    
    dds = comp(summarize, learner_i,
           job_id='learn-%s-%s-%s-refined%s-summarize' % (id_stream, id_learner, i, ref))

    learner_report = comp(report_learner,
                          'learner-%s-%s-%s-refined%s' % (id_stream, id_learner, i, ref),
                          learner_i,
                          job_id='learn-%s-%s-%s-refined%s-report' % (id_stream, id_learner, i, ref)) 
#        pdb.set_trace()
    diffeo_report = comp(report_dds,
                         'dds-%s-%s-%s-refined%s' % (id_stream, id_learner, i, ref),
                         dds,
                         job_id='learn-%s-%s-%s-refined%s-summarize-report' % (id_stream, id_learner, i, ref))
    
    rm.add(learner_report, 'learner-%s-refined%s' % (i, ref), id_learner=id_learner, id_stream=id_stream)
    rm.add(diffeo_report, 'dds-%s-refined%s' % (i, ref), id_learner=id_learner, id_stream=id_stream)

#    comp(save_results, id_learner, id_stream, outdir, dds,
#         job_id='learn-%s-%s-%s-refined%s-summarize-save' % (id_stream, id_learner, i, ref))
    
    return  learner_i, comp(calculate_areas, learner_i, dds, ref + 1)
        
def uncertainty_range(learner):
    pass
def merge_learners(learners):
    learner_0 = learners[0]
    for i in range(1, len(learners)):
        learner_0.merge(learners[i])
        
    return learner_0

#def summarize(learner, variance_limit=None):
#    dds = learner.summarize()
#    return dds

def calculate_areas(learner, dds, nrefine):
    return learner.calculate_areas(dds, nrefine)
    
#def jobs_rlearn_comb(config, rm, outdir, id_learner, id_stream, nthreads, nrefine,
#                     intermediate_reports=True):
#    partial = []
#    
#    for i in range(nthreads):
#        
#        
#        
#        job_id = 'learn-%s-%s-%sof%s' % (id_stream, id_learner, i + 1, nthreads)
#        learner_i = comp(rlearn_partial, config, id_learner, id_stream, i, nthreads, nrefine,
#                         job_id=job_id)
#        partial.append(learner_i)
#        
#        if intermediate_reports:
#            diffeo_i = comp(summarize, learner_i, job_id=job_id + '-summarize')
#            learner_i_report = comp(report_learner, 'learner-%s-%s-%sof%s' % (id_stream,
#                                                id_learner, i + 1, nthreads),
#                                    learner_i, job_id=job_id + '-report') 
#            diffeo_i_report = comp(report_dds, 'dds-%s-%s-%sof%s' % (id_stream,
#                                                          id_learner, i + 1, nthreads),
#                                    diffeo_i, job_id=job_id + '-summarize-report')
#            
#            rm.add(learner_i_report, 'learner-partial', id_learner=id_learner,
#                                                    i=i, id_stream=id_stream) 
#            rm.add(diffeo_i_report, 'dds-partial', id_learner=id_learner,
#                                                    i=i, id_stream=id_stream) 
#
#    current = partial[0]
#    for i in range(1, nthreads):
#        job_id = 'learn-%s-%s-join-%sof%s' % (id_stream, id_learner, i, nthreads - 1)
#        current = comp(rlearn_join, current, partial[i], job_id=job_id)
#    learner = current
#    dds = comp(summarize, current,
#               job_id='learn-%s-%s-summarize' % (id_stream, id_learner))
#
#    learner_report = comp(report_learner,
#                          'learner-%s-%s' % (id_stream, id_learner),
#                          learner,
#                          job_id='learn-%s-%s-report' % (id_stream, id_learner)) 
#    
#    diffeo_report = comp(report_dds,
#                         'dds-%s-%s' % (id_stream, id_learner),
#                         dds,
#                         job_id='learn-%s-%s-summarize-report' % (id_stream, id_learner))
#    
#    rm.add(learner_report, 'learner', id_learner=id_learner, id_stream=id_stream) 
#    rm.add(diffeo_report, 'dds', id_learner=id_learner, id_stream=id_stream) 
#
#    comp(save_results, id_learner, id_stream, outdir, dds,
#         job_id='learn-%s-%s-summarize-save' % (id_stream, id_learner)) 

def compute_estimator_startdata():
    
    pass

def report_area_propagation(data_sequence, sensels, id_report):
    
    report = Report(id_report)
    for index in sensels:
        f = report.figure()
        
        with f.plot('esim', caption='esim') as pylab:
            pylab.hold(True)
            for data in data_sequence:
                learner, next_area = data
                grid_shape = learner.grid_shape
                esim = learner.neig_esim_score[index, :].reshape(grid_shape)
                xl, yl = learner.area_positions_coarse[index]
                xu, yu = learner.area_positions_coarse[index] + learner.grid_shape
                
                extent_box = np.array((xl, xu, yl, yu)) * learner.area / learner.grid_shape
                
                pylab.imshow(esim, extent=extent_box)
            
    return report
            
    
def save_results(id_learner, id_stream, outdir, dds, level=''):
    id_dds = '%s-%s-r%s' % (id_stream, id_learner, level)
    resdir = os.path.join(outdir, 'results')
    desc = "Learned from stream %s and learner %s with %s refinings" % (id_stream, id_learner, level)
    ds_dump(dds, resdir, id_dds, desc)
    

def rlearn_join(learner1, learner2):
    # TODO:
    learner1.merge(learner2)
    return learner1

def rlearn_partial(config, id_learner, id_stream, i, n, search_areas, parent):
    '''
    Reads commands from a filtered stream and updates the corresponding learner.
    
    :param config:
    :param id_learner:    id of a learner in <config>
    :param id_stream:     id of a stream in <config> 
    :param i:             index of this learner
    :param n:             total number of threads/learners
    :param search_areas:  tuple of ararys (corner, size) of the search areas 
    '''
#    pdb.set_trace()
    stream = config.streams.instance(id_stream)
    learner = config.learners.instance(id_learner)

    logger.info('Using search area :' + str(search_areas))
    if search_areas is not None:
        learner.set_search_areas(search_areas[0])
    
    logitems = stream.read_all()
    filtered = filter_commands(logitems, i, n)
    nj = 0
    for _, u, _, _ in filtered:
        nj += 1
        
    logitems = stream.read_all()
    filtered = filter_commands(logitems, i, n)
    nrecords = 0
    j = 0
    t0 = time.time()
    for y0, u, y1, _ in filtered:
        logger.debug('Updating samp%d learner%d of %d' % (j, i, n))

        j += 1
        learner.update(y0, u, y1)
        nrecords += 1
        if nrecords % 10 == 0:
            logger.debug('currently %d records' % nrecords)
            
        # Progress bar
        steps = 50
        t = int((time.time() - t0) / j * nj)
        
        t_text = ''
        if t / (60 * 60 * 24) > 0:
            days = t / 60 / 60 / 24
            t_text += (', %s days, ' % days)
            t = t % (60 * 60 * 24)
        if t / (60 * 60) > 0:
            hours = t / (60 * 60)
            t_text += ('%s hours, ' % hours)
            t = t % (60 * 60)
        if t / 60 > 0:
            mins = t / 60
            t_text += ('%s minutes and ' % mins)
#        if t / 60 > 0:
        t_text += ('%s seconds remaining' % (t % 60))
        progress = steps * j / nj
        bar = '|' + '=' * progress + ' ' * (steps - progress) + '|'
        p_text = (' %s %% (%s of %s records)' % (100 * j / nj, j, nj))
        logger.info(bar + p_text + t_text)
    logger.info('Total of %d records' % (nrecords))
    
#    pdb.set_trace()
#    if parent is not None:
#        if hasattr(parent, 'plot_ranges'):
#            learner.plot_ranges = parent.plot_ranges
#        else:
#            learner.plot_ranges = 'requested'
#    else:
#        learner.plot_ranges = 'requested'
#    learner.summarize()
    return learner
