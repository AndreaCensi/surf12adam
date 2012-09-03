from . import  logger
from .. import declare_command
from compmake import (batch_command, compmake_console, comp,
    read_rc_files, use_filesystem)
import itertools
import os
from reprep.report_utils.report_manager import ReportManager
from reprep import Report
from bootstrapping_olympics.utils.safe_pickle import safe_pickle_dump
from conf_tools.load_entries import write_entries
from diffeoplan.library.discdds.writing import ds_dump


@declare_command('plearn',
                 'plearn  [<stream1> ...]')
def plearn(config, parser): #@UnusedVariable
    """ Displays the learned DDS """
    #parser.add_option("-i", "--id_image", help="ID image.", default='lena')
    parser.add_option("-n", "--nthreads", help="Number of threads",
                      type='int', default='4')
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
    
    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0
    

def jobs_plearn(config, rm, learners, streams, outdir, nthreads):
    for id_learner, id_stream in itertools.product(learners, streams):
        # try instancing them
        config.streams.instance(id_stream)
        #config.learners.instance(id_learner) # TODO: do in other way
        jobs_plearn_comb(config, rm, outdir, id_learner, id_stream, nthreads)
        
def jobs_plearn_comb(config, rm, outdir, id_learner, id_stream, nthreads,
                     intermediate_reports=False):
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
    # TODO:
    learner1.merge(learner2)
    return learner1

def report_learner(id_report, learner):
    # TODO:
    r = Report(id_report)
    learner.display(r)
    return r

def report_dds(id_report, dds):
    # TODO:
    r = Report(id_report)
    dds.display(r)
    return r


def summarize(learner):
    return learner.summarize()

def plearn_partial(config, id_learner, id_stream, i, n):
    stream = config.streams.instance(id_stream)
    learner = config.learners.instance(id_learner)
    logitems = stream.read_all()
    
    filtered = filter_every(logitems, i, n)
    nrecords = 0
    for y0, u, y1 in filtered:
        learner.update(y0, u, y1)
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
    
