'''
Created on Oct 4, 2012

@author: adam
'''

from . import logger
from PIL import Image #@UnresolvedImport
from diffeoplan.configuration import DiffeoplanConfigMaster
from diffeoplan.library.images.uncertain_image import UncertainImage
from diffeoplan.library.logs.rosbag.bag_reader import get_image_array
from diffeoplan.programs.bench.bench_jobs import init_algorithm
import camera_actuator.srv #@UnresolvedImport
import numpy as np
import pdb #@UnusedImport
import rospy
from optparse import OptionParser
from reprep import Report
from reprep.report_utils import ReportManager
from rospy.timer import sleep
from compmake import (comp, batch_command, compmake_console, use_filesystem,
    read_rc_files)

IMAGE_SERVICE = '/logitech_cam/take_image'
PLAN_EXECUTE_SERVICE = '/logitech_cam/executePlan'

class OnlinePlan():
    def __init__(self, id_algo, id_discdds, id_config='default:.'):
        self.id_algo = id_algo
        self.id_discdds = id_discdds
        self.config = DiffeoplanConfigMaster()
        self.config.load(id_config)
        
        
        discdds = self.config.discdds.instance(id_discdds)
        algo = init_algorithm(self.config, id_algo, id_discdds, discdds)
        
        self.cmdlist = [discdds.actions[i].original_cmd for i in range(len(discdds.actions))]
        self.algo = algo
        self.discdds = discdds
        
    def get_size(self):
        sizeT = self.discdds.get_shape()
        return [sizeT[1], sizeT[0]]
    
    def run_open(self, y_goal):
        y_start = get_image(self.get_size())
        
        result = run_planning(self.id_algo, self.algo, y_start=y_start, y_goal=y_goal)
        if result['result'].success:
            plan = result['result'].plan
            plan = inverse_plan(plan, self.cmdlist)
        else:
            pass
        
        execute_plan(plan)
        return plan, result
    
    def run_closed(self, y_goal):
        pass
    
    def run_demo(self, plan_true):
        algo = self.algo
        
        # Calculate the inverse plan
        cmdlist = self.cmdlist
        
        logger.info(plan_true)
        
        plan_inverse = inverse_plan(plan_true, cmdlist)
        
        # assert the length of plan and inverse agree
        assert(len(plan_inverse) == len(plan_true))
        
        size = self.get_size()
        
        # Take goal image
        y_goal = get_image(size)
        logger.info('y_goal taken, sleeping for a bit')
        sleep(1)
        logger.info('proceeding')
        
        # Execute plan_inverse to get to start position
        execute_plan(plan_inverse)
        logger.info('plan executed, sleeping for a bit')
        sleep(1)
        logger.info('proceeding')
        
        y_start = get_image(size)
        
        plan_result, result = self.run_open(y_goal)
#        
#        for i in range(4):
#            # Take start image
#            y_start = get_image(size)
#            logger.info('y_start taken')
#            logger.info('proceeding')
#                
#            # Run planning
#    #        result = run_planning(self.id_algo, algo, y_start=y_start, y_goal=y_goal, y_pred=y_pred, plan_true=plan_true)
#            result = run_planning(self.id_algo, algo, y_start=y_start, y_goal=y_goal, y_pred=None, plan_true=plan_true)
#            if result is None:
#                plan_result = [0]
#            else:
#                plan_result = inverse_plan(result['result'].plan, cmdlist)   # Compensate for bug in diffeo
#                print('found plan at this itteration: ' + str(plan_result))
#            
#            # Execute plan_result
#            execute_plan([plan_result[0]])
            
        
        
        # Predict the goal image, not used for planning anymore
        dds = algo.get_dds()
        y_pred = dds.inverse().predict(y_start, plan_true)
        
        # Take result image
        y_result = get_image(size)
        
        
        # Predict the result image
        dds = algo.get_dds()
        y_pred_result = dds.inverse().predict(y_start, plan_result)
        

        return {'y_start': y_start,
                'y_goal':y_goal,
                'y_result':y_result,
                'y_pred':y_pred,
                'y_pred_result':y_pred_result,
                'plan_true':plan_true,
                'plan_result':plan_result,
                'result': result
                }
        
#        filename = '/home/adam/public_html/lasresult.html'
#        report.to_html(filename)
        
def write_report(result, metric):
    report = Report('OnlinePlanning')
    report.text('summary', 'Result report for online planning')
    
    if result is None:
        report.text('status', 'failed')
    
#    report.text('discdds', self.id_discdds)
    
#    report.data('discdds_cmd_list', self.cmdlist)
    
    y_start = result['y_start']
    y_goal = result['y_goal']
    y_result = result['y_result']
    y_pred = result['y_pred']
    y_pred_result = result['y_pred_result']
    
    plan_true = result['plan_true']
    plan_result = result['plan_result']
#    plan_inverse = inverse_plan(plan_true, self.cmdlist)
    
    
    # Write the plans to report
    report.text('plan_true', str(plan_true))
#    report.text('plan_inverse', str(plan_inverse))
    report.text('plan_result', str(plan_result))
            
    # Plot images
    f = report.figure(cols=5)
    with f.plot('y_start', caption='y_start') as pylab:
        cax = pylab.imshow(y_start.get_rgb(), interpolation='nearest') #@UnusedVariable
        
    with f.plot('y_goal', caption='y_goal') as pylab:
        cax = pylab.imshow(y_goal.get_rgb(), interpolation='nearest') #@UnusedVariable
        
    with f.plot('y_result', caption='y_result') as pylab:
        cax = pylab.imshow(y_result.get_rgb(), interpolation='nearest') #@UnusedVariable
        
    with f.plot('y_pred', caption='y_pred') as pylab:
        cax = pylab.imshow(y_pred.get_rgb(), interpolation='nearest') #@UnusedVariable
        
    with f.plot('y_pred_result', caption='y_pred_result') as pylab:
        cax = pylab.imshow(y_pred_result.get_rgb(), interpolation='nearest') #@UnusedVariable

    # Write distances
#    metric = self.algo.metric_goal
    
    images = [y_start, y_goal, y_result, y_pred, y_pred_result]
    images_names = ['y_start', 'y_goal', 'y_result', 'y_pred', 'y_pred_result']
    data = []
    for yr in images:
        this_row = []
        for yc in images:
            yr_yc = metric.distance(yr, yc)
            this_row.append(yr_yc)
        data.append(this_row)
        
    report.table('table', data=data, cols=images_names, rows=images_names, caption='Distances')
    
    report.text('precision', str(result['result']['plan_precision']))
    report.text('min_visibility', str(result['result']['plan_min_visibility']))
    
    return report

def make_stats(planning_results, metric):
    distance = []
    row_lable = []
    for i, result in enumerate(planning_results):
        if result is None:
            pass
        else:
            y_goal = result['y_goal']
            y_result = result['y_result']
        
            distance.append([metric.distance(y_result, y_goal)])
            row_lable.append(str(i))
        
    report = Report('OnlinePlanningSummary')
    report.table('table', data=distance, rows=row_lable, caption='Distance from y_result to y_goal')
    
    # Plot histogram of distance
    f = report.figure(cols=1)
    with f.plot('error', caption='Error distribution') as pylab:
        cax = pylab.hist(distance, len(planning_results)) #@UnusedVariable
    
    return report
    
def main():
    parser = OptionParser()
    parser.add_option("-p", "--plans",
                      help="""
                      int[] of plans between y0 and y1
                      int length of random plans to be generated
                      """, default=None)
    parser.add_option("-l", "--length",
                      help="Numper of plans to use, ignored if plans are specified", default=None)
    parser.add_option("-a", "--algos",
                      help="algorthm id", default='gnbc1_it1000')
    parser.add_option("-d", "--discdds",
                      help="discdds id", default='orbit-pt256-80-n35s')
    parser.add_option("-e", "--env",
                      help="Environment Name", default='')
    (options, _) = parser.parse_args()
    
    id_algo = options.algos
    id_discdds = options.discdds
    online_plan = OnlinePlan(id_algo, id_discdds)
    
    plans = eval(options.plans)
    if plans.__class__ == int:
        n = len(online_plan.cmdlist)
        plans = np.random.randint(0, n, (int(options.plans), int(options.length))).tolist()
    
    
    planning_results = []
    for plan_true in plans:
        print('Planning online with plan: ' + str(plan_true))
        this_result = online_plan.run_demo(plan_true)
        planning_results.append(this_result)

#    write_report = online_plan.write_report

    rm = ReportManager('out/online')
    for i, result in enumerate(planning_results):
        report = comp(write_report, result, online_plan.algo.metric_goal)
        kwargs = dict(id_algo=id_algo, id_discdds=id_discdds, plan_length=options.length, env=options.env)
        rm.add(report, 'online-report-' + str(i), **kwargs)
    
    report = comp(make_stats, planning_results, online_plan.algo.metric_goal)
    kwargs = dict(id_algo=id_algo, id_discdds=id_discdds, plan_length=options.length, env=options.env)
    rm.add(report, 'summary', **kwargs)
    
    rm.create_index_job()
#    read_rc_files()
#    pdb.set_trace()
    compmake_console()
    
def inverse_plan(plan, cmdlist):
    # Calculate the inverse plan
    plan_inverse = []
    for c in plan:
        cmd = cmdlist[c]
        for i in range(len(cmdlist)):
            if all(cmd == -cmdlist[i]):
                plan_inverse.append(i)
    return plan_inverse
    
def run_planning(id_algo, algo, y_start, y_goal, y_pred=None):
    '''
    :param id_algo:
    :param algo: Already init()ed instance of DiffeoPlanningAlgo
    ''' 
    # compute achievable precision using the model
    metric = algo.metric_goal

    dds = algo.get_dds()
    
    if y_pred is not None:
        d = metric.distance(y_goal, y_pred)
        precision = d * 1.01
    else:
        d = np.min([metric.distance(y_goal, action.predict(y_goal)) for action in dds.actions])
        precision = d * 1.01
        
    min_visibility = 0
    
    planning_result = algo.plan(y_start, y_goal, precision=precision,
                                min_visibility=min_visibility)
    if not planning_result.success:
        return None
    logger.info('Plan found: ' + str(planning_result.plan))
    
    cache_stats = {
     'start_tree': algo.start_tree.get_cache_stats(),
     'goal_tree': algo.goal_tree.get_cache_stats(),
     'connector': algo.connector.get_cache_stats(),
    }
    
    algo.start_tree.clear_cache()
    algo.goal_tree.clear_cache()
    algo.connector.clear_cache()
    
    results = {}
    results['id_algo'] = id_algo
    results['id_algo'] = id_algo
    results['result'] = planning_result
    results['cache_stats'] = cache_stats
    results['plan_precision'] = precision
    results['plan_min_visibility'] = min_visibility
    return results

def get_image(size):
    '''
    Requests an image from the ROS service specified by IMAGE_SERVICE. 
    Resize to desired size and returns as an UncertainImage
    
    :param size:    size of the image
    
    :return image_resized: as an UncertainImage
    '''
    # Connection to ROS service
    takeImage = rospy.ServiceProxy(IMAGE_SERVICE, camera_actuator.srv.imageService)
    
    # Get image and convert to numpy
    image = get_image_array(takeImage(0).image)
    
    # Convert to PIL, resize and back to numpy
    image_resized = np.array(Image.fromarray(image).resize(size))
    
    # Return the resized image as UncertainImage
    return UncertainImage(image_resized)
    
def execute_plan(plan):
    '''
    Sends the plan to ROS service and waits for the execution to complete
    
    :param plan: as int[]
    '''
    # Connection to ROS service
    command = rospy.ServiceProxy(PLAN_EXECUTE_SERVICE, camera_actuator.srv.planCommand)
    
    # Send the plan to ROS module
    command(plan)
    
def get_images(size, plan):
    logger.info('Starting online module')
    
    takeImage = rospy.ServiceProxy('/logitech_cam/take_image', camera_actuator.srv.imageService)
    command = rospy.ServiceProxy('/logitech_cam/executePlan', camera_actuator.srv.planCommand)
    
    image_goal = get_image_array(takeImage(0).image)
    logger.info('Goal image taken')
    command(plan)
        
    image_start = get_image_array(takeImage(0).image)
    
    Image.fromarray(image_goal).save('goal.png')
    Image.fromarray(image_start).save('start.png')
    
    Y0 = UncertainImage(np.array(Image.fromarray(image_start).resize(size)))
    Y1 = UncertainImage(np.array(Image.fromarray(image_goal).resize(size)))
    
    return Y0, Y1
