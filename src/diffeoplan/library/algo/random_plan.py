from . import DiffeoPlanningAlgo, PlanningResult, contract
from .. import  UncertainImage
import numpy as np
import time


class RandomGuess(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns a random guess 
        for the plan of the given length. 

    """
    
    @contract(nsteps='int,>=1')
    def __init__(self, nsteps, max_cpu_time=1.0):
        '''
        :param nsteps: Number of steps in the random guess.
        '''
        self.nsteps = nsteps
        self.max_cpu_time = max_cpu_time
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): #@UnusedVariable
        guess = self.get_dds().get_random_plan(self.nsteps)
        wait = np.random.rand() * self.max_cpu_time
        print('Sleeping %f seconds' % wait)
        time.sleep(wait)
        return PlanningResult(True, guess, 'Random %d-steps plan.' % self.nsteps)
