from compmake import comp
from diffeoplan.programs.bench.report import write_report
from reprep import Report, plot_utils
import numpy as np
import os
from reprep.graphics.filter_scale import scale
from contracts import contract
import reprep
     
def create_visualization_jobs(config, outdir, allruns):
    
    for run in allruns:
        id_tc = run['id_tc']
        id_algo = run['id_algo']
        result = allruns[run]
        job_id = 'visualize-%s-%s' % (id_tc, id_algo)
        report = comp(visualize_result, config, id_tc, id_algo, result, job_id=job_id)
        report_basename = os.path.join(outdir, 'visualization', id_algo, id_tc)
        comp(write_report, report, report_basename, job_id=job_id + '-write')


def visualize_result(config, id_tc, id_algo, stats):
    """ Returns a report """
    result = stats['result']
    r = Report('%s-%s' % (id_tc, id_algo))
    tc = config.testcases.instance(id_tc)
    discdds = config.discdds.instance(tc.id_discdds)

    tc.display(r.section('testcase'))
    
    if not result.success:
        r.text('Plannning unsuccesful')
    else:
        rsol = r.section('solution')
        rsol.text('plan', 'Plan: %s' % result.plan)
    
        f = rsol.figure(cols=4)
        y0 = tc.y0
        y1 = tc.y1
        y1plan = discdds.predict(y0, result.plan)
        f.data_rgb('y1plan', y1plan.get_rgb(),
                   caption='plan prediction (certain)')
        f.data_rgb('y1plan_certain', y1plan.get_rgb_uncertain(),
                   caption='certainty of prediction')
        
        mismatch = np.abs(y1.get_values() - y1plan.get_values()).sum(axis=2)
        f.data_rgb('mismatch', scale(mismatch),
                   caption='Mismatch value pixel by pixel (zero for synthetic testcases...)')
    
    
    extra = result.extra
    
    draw_trees(config, r, tc, discdds, extra)
    write_log_lines(r, extra)
    
    return r

def draw_trees(config, r, tc, discdds, extra):
    if not 'start_tree' in extra:
        return
    
    start_tree = extra['start_tree']
    f = r.figure()

    ndim = discdds.actions[0].original_cmd.size
    if ndim != 2:
        msg = ('Sorry, tree visualization only works with 2D commands, '
               'and I got commands like %s' % discdds.actions[0].original_cmd)
        r.text('sorry', msg)
        return

    labels = {}
    
    def add_label(x, y, text, eps=0.3):
        labels[(float(x), float(y))] = ((x, y), '')
        ex, ey = x, y
        while True:
            coord = (float(ex), float(ey))
            if not coord in labels:
                break
            # markov walk
            ex = ex + eps * (np.random.randint(-1, 1)) 
            ey = ey + eps * (np.random.randint(-1, 1)) 
        #print((ex, ey), (x, y), text)
        labels[coord] = ((x, y), text)
        
    def plot_labels(pylab):
        for coord in labels:
            p, text = labels[coord]
            pylab.plot([coord[0], p[0]], [coord[1], p[1]], '-', color=[0.8, 0.8, 0.8])
        for coord in labels:
            p, text = labels[coord]
            pylab.text(coord[0], coord[1], text)
    
    with f.plot('start_tree') as pylab:    
        for i, node in enumerate(start_tree.nodes):
            plan = node.path 
#            point = plan2point(discdds, plan)
            x, y = plan2path(discdds, plan)
            point = x[-1], y[-1]
            pylab.plot(x, y, 'y-')
            pylab.plot(point[0], point[1], 'ro')
            add_label(point[0], point[1], '%d' % i)
                    
        goal = plan2point(discdds, tc.true_plan)
        pylab.plot(goal[0], goal[1], 'go')
        add_label(goal[0], goal[1], 'G')
        
        plot_labels(pylab)
        
        pylab.axis('equal')
        plot_utils.turn_all_axes_off(pylab)
        plot_utils.y_axis_extra_space(pylab, 0.1)
#        plot_utils.x_axis_extra_space(pylab, 0.1)

        
@contract(plan='seq[N](int)', returns='tuple(array[N+1],array[N+1])')
def plan2path(discdds, plan):
    """ 
        Converts a plan to a path of 2d points. 
        Returns x,y arrays. 
    """
    N = len(plan)
    x, y = [0], [0]
    for i in range(N):
        steps = plan[:i + 1]
        point = plan2point(discdds, steps)
        x.append(point[0])
        y.append(point[1])
    x, y = np.array(x), np.array(y)
#
#    avg_norm = np.mean([np.linalg.norm(x) for x in commands])
#    commands = np.array(commands)
    
#    perturbation = 0.05 * avg_norm * (np.random.rand(*commands.shape) - 0.5)
#    pertubation([])
#    print perturbation
#    commands = commands + perturbation
#    e = 0.1
#    x = x + np.random.rand(x.size) * e
#    y = y + np.random.rand(x.size) * e
#    
    #print('plan: %s' % plan)
    #print('path: %s %s' % (x, y))
    return x, y


@contract(plan='seq[>=0](int)', returns='array[2]')
def plan2point(discdds, plan):
    """ Converts a plan to a 2D point (for visualization) """
    if len(plan) == 0:
        return np.array([0, 0]) # origin 
    commands = discdds.indices_to_commands(plan)
    return np.sum(commands, axis=0)
    
    
def write_log_lines(r, extra):    
    if 'log_lines' in extra:
        log_lines = extra['log_lines']
        r.text('execution_log', '\n'.join(log_lines))
    else:
        r.text('execution_log', '(no log recorded)')
    
    
