from . import PlanningResult, contract, logger
from .. import DiffeoSystem, UncertainImage

class DiffeoPlanningAlgo:
    """ Interface for a generic planning algorithm. """
    
    def __init__(self):
        self.log_lines = [] # log lines
        
    @contract(dds=DiffeoSystem)
    def init(self, dds):
        """ Might be redefined to add precomputation. """ 
        self._dds = dds
    
    def get_dds(self):
        """ Returns the system that we want to plan on. """
        return self._dds
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): #@UnusedVariable
        """ Must be redefined by subclasses. 
        
            This must return an instance of PlanningResult.
        """
        return PlanningResult(success=False, plan=None, status='Not implemented')

    
    def info(self, s):
        """ Logs a string; saves it for visualization. """
        logger.info(s)
        self.log_lines.append(s)

    def make_extra(self):
        """ 
            Returns the dictionary that we can send as part of PlanningResults. 
            Overload to add more fields. 
        """
        extra = {'log_lines': self.log_lines}
        return extra
