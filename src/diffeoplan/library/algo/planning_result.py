from . import contract

__all__ = ['PlanningResult']


class PlanningResult:
    """ Results of planning. (more fields might be added in the future) """
    
    @contract(success='bool', plan='None|seq(int)', status='None|str', extra='dict')
    def __init__(self, success, plan, status, extra={}):
        '''
            :param success: True if planning was succesful.
            :param plan: The plan as a sequence of integer, or None.
            :param status: Short status line for debugging purposes.
            :param extra: Extra information used for visualization
                
        '''
        self.success = success
        if plan is not None:
            plan = tuple(plan)
        self.plan = plan
        self.status = status
        self.extra = extra
