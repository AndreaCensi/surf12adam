from . import DiffeoPlanningAlgo, PlanningResult, contract
from .. import  UncertainImage
import numpy as np
import time


class GraphSearch(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 

    """
    
    @contract(nsteps='int,>=1')
    def __init__(self, nsteps):
        '''
        :param nsteps: Number of steps in the random guess.
        '''
        dds = self.get_dds()
        self.ncommands = len(dds.actions)
        self.nsteps = nsteps
        self.max_cpu_time = max_cpu_time
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): #@UnusedVariable
        
        dds = self.get_dds()
        
        plans = [range(self.ncommands)]
        
        for i in range(self.nsteps-1):
            plans = expand_level(plans)
        
        dist_value = []
        Yp = []
        for plan in plans:
            yp = dds.predict(y0,plan)
            dist_function = uncertain_image.dist_values_L1
            dist_value.append(dist_function(yp,y1))
            Yp.append(yp)

        best_ind = dist_value.index(np.min(dist_value))
        works = self.images_match(y1, Yp[best_ind])
        
        return PlanningResult(works, plan[best_ind], 'Random %d-steps plan.' % self.nsteps)
    
    def images_match(self, y0, y1):
        return True
        
    def expand_alldir(self, plan):
        n = self.ncommands
        expanded = []
        pdb.set_trace()
        for i in range(n):
            plani = copy.deepcopy(plan)
            plani.append(i)
            expanded.append(plani)
        return expanded
    
    def expand_level(self, plans):
        extplans = []
        for i in range(len(plans)):
            extplans.extend(expand_alldir(plans[i]))
        return extplans

