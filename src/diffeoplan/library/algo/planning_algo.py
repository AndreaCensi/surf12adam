from . import PlanningResult, contract, logger
from .. import DiffeoSystem, UncertainImage
from reprep import Report

class DiffeoPlanningAlgo:
    """ Interface for a generic planning algorithm. """
    
    def __init__(self):
        self.log_lines = [] # log lines
        
    @contract(dds=DiffeoSystem)
    def init(self, id_dds, dds):
        """ Might be redefined to add precomputation. """
        self.info('Initialized with dds %r' % id_dds) 
        self._dds = dds
        self.id_dds = id_dds
    
    @contract(report=Report)
    def init_report(self, report):
        """ Creates a report for the initialization phase. """
        report.text('warning', 'init_report() not implemented for this class.')
    
    def get_dds(self):
        """ Returns the system that we want to plan on. """
        return self._dds
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): #@UnusedVariable
        """ Must be redefined by subclasses. 
        
            This must return an instance of PlanningResult.
        """
        return PlanningResult(success=False, plan=None, status='Not implemented')

    def plan_report(self, report):
        """ Report after planning (using own data structures) """
        report.text('warning', 'plan_report() not implemented for this class.')
    
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
