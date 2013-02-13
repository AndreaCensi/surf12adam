'''
Created on Oct 4, 2012

@author: adam
'''

from . import logger
from compmake import (comp, compmake_console)
from diffeoplan.configuration import DiffeoplanConfigMaster, set_current_config
from diffeoplan.library.online import report_tools
from optparse import OptionParser
from reprep.report_utils import ReportManager
import pickle
import yaml

IMAGE_SERVICE = '/logitech_cam/take_image'
PLAN_EXECUTE_SERVICE = '/logitech_cam/executePlan'

    
def main():
    logger.info('Starting online planning')
    parser = OptionParser()
    parser.add_option("-t", "--id_otc",
                      help="", default=None)
    parser.add_option("-e", "--env",
                      help="environment name", default='default')
    parser.add_option("-o", "--result",
                      help="Result storage file", default=None)
    parser.add_option("-r", "--report",
                      help="Specify output reports", default='')
    (options, _) = parser.parse_args()

    config = DiffeoplanConfigMaster()
    config.load('default:/home/adam/diffeo-data/')
    
    report_opt = options.report.split(',')
    
    set_current_config(config)
    if options.id_otc is not None:
        testcases = options.id_otc.split(',')
    else:
        testcases = []
        
        
    if options.result is not None:
        data_files = options.result.split(',')
    else:
        data_files = []
        
    try:
        all_stats = []
        for data_file in data_files:
            all_stats += pickle.load(open(data_file))
    except:
        pass
        
    for tc in testcases:
        logger.info('Starting up tests with : %s' % tc)
        planning_module = config.online_testcases.instance(tc)
        all_stats += planning_module.run_all_tests(options.env)
    
    pickle.dump(all_stats, open(options.result, 'wb'))

    rm = ReportManager('out/online')
    
    if 'vis' in report_opt:
        for i, stat in enumerate(all_stats):
            report = comp(report_tools.run_report, stat)
            kwargs = dict(env=stat.labels['env'])
            rm.add(report, 'online_run_' + str(i), **kwargs)
    
    stats_file = yaml.load(open('/home/adam/git/boot12env/src/surf12adam/orbit.online_report.yaml'))

    stats_def = []
    for i in range(len(stats_file)):
        if stats_file[i]['id'] in report_opt:
            stats_def.append(stats_file[i])
    
    kwargs = {'summary':'summary'}
    report = comp(report_tools.stat_report, stats_def, all_stats)
    rm.add(report, 'summary_stats', **kwargs)
    
    kwargs = {'summary':'empty'}
    report = comp(report_tools.empty_report)
    rm.add(report, 'empty', **kwargs)
    
    rm.create_index_job()
    compmake_console()
