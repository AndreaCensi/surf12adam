from . import contract

class PlanningResult:
    """ Results of planning. (more fields might be added in the future) """
    
    @contract(success='bool', plan='None|list(int)', status='None|str')
    def __init__(self, success, plan, status):
        '''
        :param success: True if planning was succesful.
        :param plan: The plan as a sequence of integer, or None.
        :param status: Short status line for debugging purposes.
        '''
        self.success = success
        self.plan = plan
        self.status = status
