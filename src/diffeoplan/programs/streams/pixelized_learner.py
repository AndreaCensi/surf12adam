from . import logger
from boot_agents.diffeo.learning import DiffeomorphismEstimatorPixelized
from compmake import (comp, batch_command, compmake_console, read_rc_files,
    use_filesystem)
from diffeoplan.library.discdds.diffeo_action import DiffeoAction
from diffeoplan.library.discdds.diffeo_system import DiffeoSystem
from diffeoplan.programs.utils import declare_command
from reprep import Report
from quickapp import ReportManager
import itertools
import numpy as np
import os
import pdb

@declare_command('pixlearn', 'pixlearn  [<stream1> ...]')
def pixlearn(config, parser):
    parser.add_option("-n", "--nthreads", help="Number of threads",
                      type='int', default='4')
    parser.add_option("--distribute", type='str', default='random',
                      help="Strategy to distribute sensles to threads")
    parser.add_option("-s", "--id_stream", help="Which streams to use.",
                      default="*")    
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-pixlearn/')
    parser.add_option("-l", "--id_learner", help="Learner config.", default="*")
    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    options = parser.parse_options()
    
    nthreads = options.nthreads
    id_learner = config.learners.expand_names(options.id_learner) 
    id_stream = config.streams.expand_names(options.id_stream)
    
    id_comb = ",".join(id_stream) + "-" + ",".join(id_learner)
    outdir = os.path.join(options.output, id_comb) 
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    rm = ReportManager(os.path.join(outdir, 'reports'))
    pdb.set_trace()
    if options.distribute == 'random':
        max_nsensels = 40 * 30
        # Generate a list with range of all indexes and assign them to threads
        all_indicies = np.array(range(max_nsensels))
        dist = np.random.randint(nthreads, size=max_nsensels)
        sensel_indexes = []
        for i in range(nthreads):
            sensel_indexes.append(all_indicies[dist == i])
    if options.distribute == 'demo4':
        sensel_indexes = [[0, 1, 2, 3,
                           40, 41, 42, 43,
                           80, 81, 82, 83,
                           120, 121, 122, 123],
                          [340, 341, 342, 343,
                           380, 381, 382, 383,
                           420, 421, 422, 423,
                           460, 461, 462, 463],
                          [650, 651, 652, 653,
                           690, 691, 692, 693,
                           730, 731, 732, 733,
                           770, 771, 772, 773],
                          [1076, 1077, 1078, 1079,
                           1116, 1117, 1118, 1119,
                           1156, 1157, 1158, 1159,
                           1196, 1197, 1198, 1199]] 

    if len(id_learner) > 1 or len(id_stream) > 1:
        logger.warning('learners and streams after index 0 will be ignored')
        
    id_learner = id_learner[0]
    id_stream = id_stream[0]
    
    commands = [[256, 0, 0], [-256, 0, 0]]
    states = [[100]]


    # # Parallel part of code
    # Initiate parallel learning
    group = []
    for i in range(nthreads):        
        estimator_i = comp(sensel_group_learn, config, id_learner, id_stream,
                           sensel_indexes[i])
        group.append(estimator_i)
        
        diffeo_system_i = comp(estimator_summarize, estimator_i,
                               commands, states, 'diffeo' + str(i))
        
        estimator_report = comp(report_estimator, 'estimator' + str(i), estimator_i)
        rm.add(estimator_report, 'estimator' + str(i), id_learner=id_learner,
               id_stream=id_stream)
        
        diffeo_report = comp(report_dds, 'diffeo' + str(i), diffeo_system_i)
        rm.add(diffeo_report, 'diffeo' + str(i), id_learner=id_learner,
               id_stream=id_stream)

    estimator_main = comp(join_estimators, group)
    main_system = comp(estimator_summarize, estimator_main,
                       commands, states, 'diffeo' + str(i))
    
    diffeo_report = comp(report_dds, 'dds-%s-%s' % (id_stream, id_learner),
                         main_system,
                         job_id='learn-%s-%s-summarize-report' % (id_stream, id_learner))
    
    rm.add(comp(empty_report), 'empty')
    rm.add(diffeo_report, 'dds', id_learner=id_learner, id_stream=id_stream)
    
    
    rm.create_index_job()
    
    if options.command:
#        return batch_command(options.command)
        batch_command(options.command)
    else:
        compmake_console()
#        return 0
#    pdb.set_trace()

def sensel_group_learn(config, id_learner, id_stream, sensels):
    learner = config.learners.instance(id_learner)
    group = DiffeomorphismEstimatorPixelized(sensels=sensels,
                                             **learner.diffeo_estimator_params)
    group_inv = DiffeomorphismEstimatorPixelized(sensels=sensels,
                                             **learner.diffeo_estimator_params)
    stream = config.streams.instance(id_stream)
    
    logitems = stream.read_all_state()
    
    nrecords = 0
    for y0, u, y1, x0 in logitems:
        group.update(y0=y0, u0=u, y1=y1, x0=x0)
        group_inv.update(y0=y0, u0=u, y1=y1, x0=x0)
        nrecords += 1
        if nrecords % 10 == 0:
            logger.info('currently %d records' % nrecords)
        if nrecords > 20:
            pass
#            break
    
    return group, group_inv

def join_estimators(groups):
#    pdb.set_trace()
    estimator, estimator_inv = groups[0]
    
    estimators = []
    estimators_inv = []
    
    for i in range(1, len(groups)):
        estimators.append(groups[i][0])
        estimators_inv.append(groups[i][1])
        
    for i in range(0, len(groups) - 1):
        estimator.merge(estimators[i])
        estimator_inv.merge(estimators_inv[i])
    
#    pdb.set_trace()    
    return estimator, estimator_inv

def display_partials():
    pass

def report_dds(id_report, dds):
    # TODO:
    r = Report(id_report)
    dds.display(r)
    return r
def empty_report():
    return Report('empty')

def report_estimator(id_report, estimator):
    r = Report(id_report)
#    pdb.set_trace()
    estimator[0].display(r)
    estimator[1].display(r, sufix='inv')
    return r

def get_diffeo(estimator):
    return estimator.summarize()

def estimator_summarize(estimators, commands, states, name):
    estimator, estimator_inv = estimators
    action_list = []
    for command, state in itertools.product(commands, states):
        diffeo = estimator.summarize(command, state)
        diffeo_inv = estimator_inv.summarize(command, state)
        action = DiffeoAction(name, diffeo, diffeo_inv, command, state)
        action_list.append(action)
    name = 'Uninterpreted Diffeomorphism System'
    system = DiffeoSystem(name, action_list)
    return system
