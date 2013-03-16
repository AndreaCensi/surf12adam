
import pickle
import urllib
from compmake import (batch_command, comp)

from reprep import Report
from reprep.report_utils import ReportManager
import os
import numpy as np


def main():
#    parser.add_option("--res", help="", default='[40, 80, 160, 320]')
#    options = parser.parse_options()
    
    
    rm = ReportManager('timeuse_reports')
    rm.add(comp(timeuse_report), 'timeuse', systems='all')
    rm.create_index_job()
    
    return batch_command('clean; make')

def timeuse_report():
    r = Report('timeuse')
    
    
    resolutions = [40, 80, 160, 320]
#    stoplevels = {40: [1, 2, 1, 5], 80: [1, 2, 1, 6], 160: [1, 3, 1, 7], 320: [1, 4, 1, 8]} # g=5x5
    stoplevels = {40: [1, 2, 1], 80: [1, 3, 2], 160: [1, 3, 2], 320: [1, 4, 3]} # g=9x9
    #stoplevels = {40: [1, 3], 80: [1, 3], 160: [1, 3], 320: [1, 3]}
    index = {40:0, 80:1, 160:2, 320: 3}
    plotdata = {}
    for res in resolutions:
        learnurl = 'http://lambda.cds.caltech.edu/~adam/data/data-1303/out/%s/learn/' % res
    
        types = ['n35s', 'fast35s-9', 'fast35s-15']
        stream = 'orbit-pt256-%s' % res
        
    #    types = ['n10s', 'fast10s-5', 'n75s', 'fast75s-5']
    #    stream = 'youbot-b4w-0525-%s' % res
        
        for i, key in enumerate(types):
    #        key = learner.split('-')[3]
            learner = '%s-%s' % (stream, key)
            if not plotdata.has_key(key):
                plotdata[key] = np.zeros(len(resolutions)) * np.NaN
            
            try:
                tfilename = learnurl + learner + '/times.pickle'
                times = pickle.load(urllib.urlopen(tfilename))
    #            pdb.set_trace()
                levels = times['cputime_index_level_learn'].shape[1]
                
                cputimes = times['cputime_index_level_learn'] + times['cputime_index_level_summarize']
        #        pdb.set_trace()
                plotdata[key][index[res]] = np.max(np.sum(cputimes[:, :stoplevels[res][i]], axis=1))
            except:
                pass
            
    def set_xlim(pylab):
        pylab.xlim(35, 350)
        pylab.gca().set_xscale('log')
        pylab.xticks(stoplevels.keys())
        pylab.gca().set_xticklabels(stoplevels.keys())
        
    print plotdata
    
    level_data = np.array(stoplevels.values())
    
    f = r.figure()
    with f.plot('timeuse', caption='Time used for learning',
                figsize=(8, 5)) as pylab:
                
                
#    pylab.figure(figsize=(8, 5))
        pylab.hold(True)
        style = {'ms': 14, 'ls': '-', 'mew': 1, 'mec':'w', 'lw': 2, 'alpha': 1}
        markers = ['o', 's', 'v', '^']
        colors = ['r', 'g', 'b', 'k']
        
        for i in [0, 1, 2]:
            key = types[i]
            pylab.plot(index.keys(), plotdata[key], color=colors[i], marker=markers[i], **style)
        
        set_xlim(pylab)
        pylab.ylim(30, 25000)
        pylab.gca().set_yscale('log')
        pylab.tight_layout()
#    pylab.savefig('timeuse_a.pdf', dpi=600)
    
    
    f = r.figure()
    with f.plot('timeuse_steps', caption='Required cycles to reach native resolution' ,
                figsize=(8, 2)) as pylab:
#    pylab.figure(figsize=(8, 2))
    
        pylab.hold(True)
        for i in [0, 1, 2]:
            pylab.plot(stoplevels.keys(), level_data[:, i], color=colors[i], marker=markers[i], **style)
        set_xlim(pylab)
        pylab.ylim(0, 5)
        pylab.ylim(0, 4.3)
        pylab.yticks(range(5))
        pylab.tight_layout()
#    pylab.savefig('timeuse_b.png')
    return r
timeuse_report()
