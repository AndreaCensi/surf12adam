'''
Created on Oct 17, 2012

@author: adam
'''

from reprep import Report
import numpy as np
from matplotlib.colors import ColorConverter

class ReportTools():
    def __init__(self, all_stats=None):
        self.all_stats = all_stats
        self.plots = {}

    def set_result_stats(self, all_stats):
        self.all_stats = all_stats
        
    def add_line_graph_mean_plot(self, x_axis, function, categorize):
        if not self.plots.has_key('line_graph_mean'):
            self.plots['line_graph_mean'] = []
             
        self.plots['line_graph_mean'].append({'x_axis':x_axis,
                                             'function':function,
                                             'categorize':categorize})
    
    def create_report(self):
        report = Report('OnlinePlanning')
        report.text('summary', 'Result report for online planning')
        
            
        # Plot images
        for job in self.plots['line_graph_mean']:
            graph_errorbar(report, self.all_stats, job['x_axis'], job['function'], job['categorize'])
            
        filename = '/home/adam/public_html/testrep.html'
        report.to_html(filename)
#        pdb.set_trace()

def empty_report():
    report = Report('OnlinePlanning')
    report.text('summary', 'Empty report')
    return report

def run_report(stat):
#    def write_report(result, metric):
    report = Report('OnlinePlanningTest')
    report.text('summary', 'Visualizing of online planning test')

    labels = stat.labels
    for key in labels.keys():
        report.text(key, str(labels[key]))
            
    images = {'y_start':stat.y_start,
              'y_goal':stat.y_goal,
              'y_result':stat.y_result,
              'y_pred':stat.y_goal_pred,
              'y_found_pred':stat.y_found_pred}
    
    keys = ['y_start',
              'y_goal',
              'y_result',
              'y_pred',
              'y_found_pred']
    
    # Plot images
    f = report.figure(cols=len(images))
    for key in keys:
        with f.plot(key, caption=key) as pylab:
            pylab.imshow(images[key].get_rgb(), interpolation='nearest')
        
    
    data = []
    for key in keys:
        yr = images[key]
        this_row = []
        for key2 in keys:
            yc = images[key2]
            yr_yc = stat.metric_goal.distance(yr, yc)
            this_row.append(yr_yc)
        data.append(this_row)
        
    report.table('table', data=data, cols=images.keys(), rows=images.keys(), caption='Distances')
    
    return report

def stat_report(stats_def, stats):
#    pdb.set_trace()
    report = Report('OnlinePlanning_statistics')
    report.text('summary', 'Result report for online planning')
    for job in stats_def:
        function = eval(job['type'])
        job['args']['plot_id'] = job['id']
        function(report, stats, **job['args'])
    return report


def graph_errorbar(report, all_stats, plot_id, captions, x_axis, functions, categorize=None, p=1, spread=0.2):
    if categorize is not None:
        sort2 = filter_stats(all_stats, categorize)
        sort1 = {}
        for category in sort2.keys():
            sort1[category] = filter_stats(sort2[category], x_axis)
    else:
        sort1 = {'all':filter_stats(all_stats, x_axis)}
    
    f = report.figure(cols=len(functions))
    for u, fcn in enumerate(functions):
        if captions.__class__ == str:
            caption = captions
        else:
            caption = captions[u]
            
        function = eval(fcn)
        plot_data = {}
        for category in sort1.keys():
            item_label = []
            item_value = []
            for item in sort1[category].keys():
                eval_item = []
                for sample in sort1[category][item]:
                    eval_item.append(function(sample))
                item_label.append(item)
                item_value.append(eval_item)
            plot_data[category] = {'label':item_label, 'value':item_value}
            
        with f.plot(plot_id + '_' + fcn, caption=caption) as pylab:
            pylab.hold(True)
            num_cat = len(plot_data.keys())
            for i, category in enumerate(plot_data.keys()):
                mean = np.array([np.mean(tick) for tick in plot_data[category]['value']])
                std = np.array([np.std(tick) for tick in plot_data[category]['value']])
#                above = [np.percentile(tick, 100 - p) for tick in plot_data[category]['value']] - mean
#                below = mean - [np.percentile(tick, p) for tick in plot_data[category]['value']]
                above = std * p
                below = std * p
                labels = np.array(plot_data[category]['label']) - spread / 2 + spread * i / num_cat
                
                yerr = np.vstack((-above, -below))
                
                pylab.errorbar(labels, mean, yerr=yerr, marker='o',
                               linestyle='',
                               color=ColorConverter.colors.values()[i])
            for i, category in enumerate(plot_data.keys()):    
                x_points = []
                y_points = []
                for j, vset in enumerate(plot_data[category]['value']):
                    label = plot_data[category]['label'][j] - spread / 2 + spread * i / num_cat
                    for tick in vset:
                        x_points.append(label)
                        y_points.append(tick)
                print(str(x_points))
                print(str(y_points))
                pylab.plot(x_points, y_points, marker='.', ls='', color=ColorConverter.colors.values()[i])
            
            pylab.ylabel(fcn)
            pylab.xlabel(x_axis)

            xlim = np.array(pylab.xlim()) + (0, 0.5)
            xlim[0] = 0
            ylim = np.array(pylab.ylim()) + (0, 0.3)
            ylim[0] = -0.1

            pylab.xlim(xlim)
            pylab.ylim(ylim)
            
            pylab.legend(plot_data.keys(), 'upper center')



def graph_errorbar_internal(report, all_stats, plot_id, captions, x_axis, functions, categorize, p=1, spread=0.2):
    
    sort1 = filter_stats(all_stats, categorize)
    
    f = report.figure(cols=len(functions))
    for u, fcn in enumerate(functions):
        if captions.__class__ == str:
            caption = captions
        else:
            caption = captions[u]
            
        function = eval(fcn)
        plot_data = {}
        for category in sort1.keys():
            
            
            
            item_label = []
            item_value = []
            for item in sort1[category].keys():
                eval_item = []
                for sample in sort1[category][item]:
                    eval_item.append(function(sample))
                item_label.append(item)
                item_value.append(eval_item)
            plot_data[category] = {'label':item_label, 'value':item_value}
            
        with f.plot(plot_id + '_' + fcn, caption=caption) as pylab:
            pylab.hold(True)
            num_cat = len(plot_data.keys())
            for i, category in enumerate(plot_data.keys()):
                mean = np.array([np.mean(tick) for tick in plot_data[category]['value']])
                std = np.array([np.std(tick) for tick in plot_data[category]['value']])
#                above = [np.percentile(tick, 100 - p) for tick in plot_data[category]['value']] - mean
#                below = mean - [np.percentile(tick, p) for tick in plot_data[category]['value']]
                above = std * p
                below = std * p
                labels = np.array(plot_data[category]['label']) - spread / 2 + spread * i / num_cat
                
                yerr = np.vstack((-above, -below))
                
                pylab.errorbar(labels, mean, yerr=yerr, marker='o',
                               linestyle='',
                               color=ColorConverter.colors.values()[i])
            for i, category in enumerate(plot_data.keys()):    
                x_points = []
                y_points = []
                for j, vset in enumerate(plot_data[category]['value']):
                    label = plot_data[category]['label'][j] - spread / 2 + spread * i / num_cat
                    for tick in vset:
                        x_points.append(label)
                        y_points.append(tick)
                print(str(x_points))
                print(str(y_points))
                pylab.plot(x_points, y_points, marker='.', ls='', color=ColorConverter.colors.values()[i])
            
            pylab.ylabel(fcn)
            pylab.xlabel(x_axis)

            xlim = np.array(pylab.xlim()) + (0, 0.5)
            xlim[0] = 0
            ylim = np.array(pylab.ylim()) + (0, 0.3)
            ylim[0] = -0.1

            pylab.xlim(xlim)
            pylab.ylim(ylim)
            
            pylab.legend(plot_data.keys(), 'upper center')


def filter_stats(all_stats, keys):
    data = {}
    for stat in all_stats:
        if not keys.__class__ == str:
            vals = tuple(stat.labels[key] for key in keys)
        else:
            vals = stat.labels[keys]
        
        if not data.has_key(vals):
            data[vals] = []
            
        data[vals].append(stat)
    return data
    
def success(stat):
    print stat.labels
    if stat.plan_found_reduced == stat.plan_true_reduced:
        return 1
    else:
        return 0
    
def result_goal_distance(stat):
    y_goal = stat.y_goal
    y_result = stat.y_result
    return stat.metric_goal.distance(y_goal, y_result)
    
def subplan(plan, subp):
    plan = list(plan)
    wrong = ()
    for c in subp:
        if c in plan:
            plan.remove(c)
        else:
            wrong += (c,)
    rest = tuple(plan)
    return {'rest':rest, 'wrong':wrong}
    
def is_subplan(stat):
    plan = stat.labels['plan_true']
    subp = stat.labels['plan_found']
    comps = subplan(plan, subp)
    if len(comps['wrong']) + len(comps['rest']) == 0:
        return 1
    else:
        return 0
    
def plan_correct_ratio(stat):
    plan = stat.labels['plan_true']
    subp = stat.labels['plan_found']
    comps = subplan(plan, subp)
    if len(subp) > 0:
        return 1 - float(len(comps['wrong'])) / float(len(subp))
    else:
        return 1 - float(len(comps['wrong']))
    
def plan_found_ratio(stat):
    plan = stat.labels['plan_true']
    subp = stat.labels['plan_found']
    comps = subplan(plan, subp)
    return 1 - float(len(comps['rest'])) / float(len(plan))

def plan_remaining_length(stat):
    plan = stat.labels['plan_true']
    subp = stat.labels['plan_found']
    comps = subplan(plan, subp)
    return len(comps['rest'])

def plan_wrong_length(stat):
    plan = stat.labels['plan_true']
    subp = stat.labels['plan_found']
    comps = subplan(plan, subp)
    return len(comps['wrong'])
