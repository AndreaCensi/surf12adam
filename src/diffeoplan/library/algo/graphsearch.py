from . import DiffeoPlanningAlgo, PlanningResult


class GraphSearch(DiffeoPlanningAlgo):
    
    def __init__(self, alpha):
        self.alpha = alpha
    
    
    def plan(self, y0, y1):
        result = PlanningResult(success=False, plan=None, status='not implemented')
        return result
