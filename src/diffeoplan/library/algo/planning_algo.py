from . import PlanningResult, contract
from .. import DiffeoSystem, UncertainImage
from diffeoplan.utils import WithInternalLog
from reprep import Report

__all__ = ['DiffeoPlanningAlgo']

class DiffeoPlanningAlgo(WithInternalLog):
    """ Interface for a generic planning algorithm. """
    
    def __init__(self):
        WithInternalLog.__init__(self)
        
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
        
    @contract(y0=UncertainImage, y1=UncertainImage, precision='>=0',
              returns=PlanningResult)
    def plan(self, y0, y1, precision): #@UnusedVariable
        """ 
            Must be redefined by subclasses. 
        
            This must return an instance of PlanningResult.
        """
        return PlanningResult(success=False, plan=None, status='Not implemented')

    @contract(report=Report)
    def plan_report(self, report, result, tc): #@UnusedVariable
        """ Report after planning (using own data structures) """
        report.text('warning', 'plan_report() not implemented for this class.')
    
    def make_extra(self):
        """ 
            Returns the dictionary that we can send as part of PlanningResults. 
            Overload to add more fields. 
        """
        extra = {}
        extra['log_lines'] = self.get_raw_log_lines()
        return extra
    
