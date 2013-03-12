import pylab
import pickle
import urllib
import pdb

import numpy as np


resolutions = [40, 80, 160, 320]
stoplevels = {40: [1, 2, 1, 5], 80: [1, 2, 1, 6], 160: [1, 3, 1, 7], 320: [1, 4, 1, 8]}
#stoplevels = {40: [1, 3], 80: [1, 3], 160: [1, 3], 320: [1, 3]}
index = {40:0, 80:1, 160:2, 320: 3}
plotdata = {}
for res in resolutions:
    learnurl = 'http://lambda.cds.caltech.edu/~adam/data/data-1303/out/%s/learn/' % res

#    types = ['n35s', 'fast35s-5']
#    stream = 'orbit-pt256-%s' % res
    
    types = ['n10s', 'fast10s-5', 'n75s', 'fast75s-5']
    stream = 'youbot-b4w-0525-%s' % res
    
    for i, key in enumerate(types):
#        key = learner.split('-')[3]
        learner = '%s-%s' % (stream, key)
        if not plotdata.has_key(key):
            plotdata[key] = np.zeros(len(resolutions)) * np.NaN
        
        try:
            tfilename = learnurl + learner + '/times.pickle'
            times = pickle.load(urllib.urlopen(tfilename))
            levels = times['cputime_index_level_learn'].shape[1]
            
            cputimes = times['cputime_index_level_learn'] + times['cputime_index_level_summarize']
    #        pdb.set_trace()
            plotdata[key][index[res]] = np.max(np.sum(cputimes[:, :stoplevels[res][i]], axis=1))
        except:
            pass
    
print plotdata
pylab.hold(True)
style = {'marker': 'o', 'ms': 9, 'ls': '-', 'mew': 0, 'lw': 4, 'alpha': .5}
for key in types:
    
    pylab.plot(index.keys(), plotdata[key], **style)
pylab.legend(types)
pylab.gca().set_yscale('log')
#pylab.gca().set_xscale('log')
pylab.show()

#pdb.set_trace()
