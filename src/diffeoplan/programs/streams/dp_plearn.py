from . import logger
from .. import declare_command
from compmake import (batch_command, compmake_console, comp, read_rc_files,
    use_filesystem)
from diffeoplan.library.discdds.writing import ds_dump
from reprep import Report
from reprep.report_utils import ReportManager
import itertools
import os
import warnings
import multiprocessing
import numpy as np
import pickle
from compmake import CompmakeGlobalState
from compmake.jobs import get_job_cache

@declare_command('plearn', 'plearn  [<stream1> ...]')
def plearn(config, parser): 
    """ Learn the diffeomorphisms in parallel. """
    # parser.add_option("-i", "--id_image", help="ID image.", default='lena')
    ncpus = multiprocessing.cpu_count()
    parser.add_option("-n", "--nthreads", help="Number of threads",
                      type='int', default=ncpus)
    parser.add_option("-s", "--streams", help="Which streams to use.",
                      default="*")
    parser.add_option("-i", "--comb", default="default")
    parser.add_option("-l", "--learners", help="Learner config.", default="*")
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-plearn/')
    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    options = parser.parse_options()
     
    nthreads = options.nthreads
    
    learners = config.learners.expand_names(options.learners) 
    streams = config.streams.expand_names(options.streams)
    id_comb = ",".join(streams) + "-" + ",".join(learners)
    outdir = os.path.join(options.output, id_comb) 
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
    rm = ReportManager(os.path.join(outdir, 'reports'))
    
    jobs_plearn(config, rm, learners, streams, outdir, nthreads)
        
    rm.create_index_job()
    
    # Time and report the learning
    comp(learning_times_plearn, outdir, learners, streams, nthreads)
    
    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0
    
def learning_times_plearn(outdir, learners, streams, nthreads):
    job_ids = CompmakeGlobalState.jobs_defined_in_this_session
    id_learner, id_stream = learners[0], streams[0]

    cputime_index_level_learn = np.ones((nthreads, 1)) * np.NaN
    walltime_index_level_learn = np.ones((nthreads, 1)) * np.NaN
    for i in range(nthreads):
        search_id = 'learn-%s-%s-%sof%s' % (id_stream, id_learner, i + 1, nthreads)
        for job_id in job_ids:
            if job_id == search_id:
                job_cache = get_job_cache(job_id)
                cputime_index_level_learn[i] = job_cache.cputime_used
                walltime_index_level_learn[i] = job_cache.walltime_used
                    
    print cputime_index_level_learn
    print walltime_index_level_learn
    
    cputime_index_level_summarize = np.ones((nthreads, 1)) * np.NaN
    walltime_index_level_summarize = np.ones((nthreads, 1)) * np.NaN
    for i in range(nthreads):
        search_id = 'learn-%s-%s-%sof%s-summarize' % (id_stream, id_learner, i + 1, nthreads)
        print(search_id)
        for job_id in job_ids:
            if job_id == search_id:                    
                job_cache = get_job_cache(job_id)
                cputime_index_level_summarize[i] = job_cache.cputime_used
                walltime_index_level_summarize[i] = job_cache.walltime_used
                    
    print cputime_index_level_summarize
    print walltime_index_level_summarize
    
    times = {'cputime_index_level_learn': cputime_index_level_learn,
             'walltime_index_level_learn': walltime_index_level_learn,
             'cputime_index_level_summarize': cputime_index_level_summarize,
             'walltime_index_level_summarize': walltime_index_level_summarize }
    
#    pdb.set_trace()
    pickle.dump(times, open(os.path.join(outdir, 'times.pickle'), 'wb'))

def jobs_plearn(config, rm, learners, streams, outdir, nthreads):
    for id_learner, id_stream in itertools.product(learners, streams):
        # try instancing them
        config.streams.instance(id_stream)
        # config.learners.instance(id_learner) # TODO: do in other way
        jobs_plearn_comb(config, rm, outdir, id_learner, id_stream, nthreads)
        
def jobs_plearn_comb(config, rm, outdir, id_learner, id_stream, nthreads,
                     intermediate_reports=True):
    partial = []
    for i in range(nthreads):
        job_id = 'learn-%s-%s-%sof%s' % (id_stream, id_learner, i + 1, nthreads)
        learner_i = comp(plearn_partial, config, id_learner, id_stream, i, nthreads,
                         job_id=job_id)
        partial.append(learner_i)
        
        if intermediate_reports:
            diffeo_i = comp(summarize, learner_i, job_id=job_id + '-summarize')
            learner_i_report = comp(report_learner, 'learner-%s-%s-%sof%s' % (id_stream,
                                                id_learner, i + 1, nthreads),
                                    learner_i, job_id=job_id + '-report') 
            diffeo_i_report = comp(report_dds, 'dds-%s-%s-%sof%s' % (id_stream,
                                                          id_learner, i + 1, nthreads),
                                    diffeo_i, job_id=job_id + '-summarize-report')
            
            rm.add(learner_i_report, 'learner-partial', id_learner=id_learner,
                                                    i=i, id_stream=id_stream) 
            rm.add(diffeo_i_report, 'dds-partial', id_learner=id_learner,
                                                    i=i, id_stream=id_stream) 

    current = partial[0]
    for i in range(1, nthreads):
        job_id = 'learn-%s-%s-join-%sof%s' % (id_stream, id_learner, i, nthreads - 1)
        current = comp(plearn_join, current, partial[i], job_id=job_id)
    learner = current
    dds = comp(summarize, current,
               job_id='learn-%s-%s-summarize' % (id_stream, id_learner))

    learner_report = comp(report_learner,
                          'learner-%s-%s' % (id_stream, id_learner),
                          learner,
                          job_id='learn-%s-%s-report' % (id_stream, id_learner)) 
    
    diffeo_report = comp(report_dds,
                         'dds-%s-%s' % (id_stream, id_learner),
                         dds,
                         job_id='learn-%s-%s-summarize-report' % (id_stream, id_learner))
    
    rm.add(learner_report, 'learner', id_learner=id_learner, id_stream=id_stream) 
    rm.add(diffeo_report, 'dds', id_learner=id_learner, id_stream=id_stream)

    comp(save_results, id_learner, id_stream, outdir, dds,
         job_id='learn-%s-%s-summarize-save' % (id_stream, id_learner))
    
    
def save_results(id_learner, id_stream, outdir, dds):
    id_dds = '%s-%s' % (id_stream, id_learner)
    resdir = os.path.join(outdir, 'results')
    desc = "Learned from stream %s and learner %s" % (id_stream, id_learner)
    ds_dump(dds, resdir, id_dds, desc)
    

def plearn_join(learner1, learner2):
#    if not learner1.initialized():
#        # not even one command
#        return learner2
#    elif not learner2.initialized():
#        return learner1
#    else:
    learner1.merge(learner2)
    return learner1

def report_learner(id_report, learner):
    r = Report(id_report)
    if learner is None:
        msg = 'Not display %r because not initialized' % id_report
        logger.info(msg)
        r.text('notice', 'Not initialized')
    else:
        learner.display(r)
    return r

def report_dds(id_report, dds):
    r = Report(id_report)
    dds.display(r)
    return r

def summarize(learner):
#    if not learner.initialized():
#        logger.info('Not summarizing because not initialized.')
#        return None
#    else:
    return learner.summarize()

def plearn_partial(config, id_learner, id_stream, i, n):
    '''
    Reads commands from a filtered stream and updates the corresponding learner.
    
    :param config:
    :param id_learner:    id of a learner in <config>
    :param id_stream:     id of a stream in <config> 
    :param i:             index of this learner
    :param n:             total number of threads/learners
    '''
    stream = config.streams.instance(id_stream)
    learner = config.learners.instance(id_learner)
    learner.nthreads = n
    learner.index = i

    logitems = stream.read_all_state()
    
    # filtered = filter_every(logitems, i, n)
    filtered = filter_commands(logitems, i, n)
    nrecords = 0
    # Switched order of images to get the correct output
    for y1, u, y0, x0 in filtered:
#    for y0, u, y1, x0 in logitems:  # use all items in log
        logger.info('x0 = ' + str(x0))
        
        learner.update(y0, u, y1, x0)
        nrecords += 1 
        if nrecords % 10 == 0:
            logger.info('currently %d records' % nrecords)

    logger.info('Total of %d records' % nrecords)
    return learner

def filter_every(it, i, n):
    count = 0
    for x in it:
        if count % n == i:
            yield x
        count += 1 
    
def filter_commands(it, i, n):
    '''
    Only gives the i-th % n discovered commands
    
    :param it:   generator with all items from the log.
    :param i:    index if the learner asking for commands.
    :param n:    total number of learners/learning threads
    read all (y0, u, y1) tuples from the generator it (from the specified log)
    and sort out the items for learner number i. 
    If number of commands and learners is the same, one learner will get one 
    command.
    If number of commands is less than number of learners, then the last 
    filtered_commands will yield nothing.  
    '''
    commands = []
      
    count = 0
    count_ours = 0
    for x in it:
        _, u, _, _ = x
        if not u in commands:
            commands.append(u)
        u_index = commands.index(u)
        if u_index % n == i:
            yield x
            count_ours += 1
            logger.debug('cmd %s %s/%s' % (u, count, count_ours))
        count += 1
        
        if False:
            if count_ours > 10:
                warnings.warn('Exiting very early')
                break
