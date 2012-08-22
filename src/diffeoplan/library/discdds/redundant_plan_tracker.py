from diffeoplan.library.discdds.diffeo_structure import DiffeoStructure
import pdb


class RedundantPlanTracker():
    """
    
    
        Use it like this: ::
        
            rpt = RedundantPlanTracker(dds, tolerance=0.2)
            
            # consider a new plan
            new_plan = ...
            
            # ask him if it is redundant
            if rpt.is_redundant(new_plan):
                # don't create a new node
                pass
            else:
                # create a node
                ...
                
                # tell it you used it
                rpt.add_plan(new_plan) 
    
    """
    def __init__(self, dds, tolerance):
        self.diffeo_structure = DiffeoStructure(dds=dds, tolerance=tolerance)
        self.used_plans = set()
        
    def add_plan(self, plan):
        # get canonical plan
        plan_tuple = tuple(plan)
        cplan = self.diffeo_structure.get_canonical(plan_tuple)
        self.used_plans.add(cplan)
        
    def is_redundant(self, plan):
        # get canonical plan
        plan_tuple = tuple(plan)
#        pdb.set_trace()
        cplan = self.diffeo_structure.get_canonical(plan_tuple)
        return cplan in self.used_plans
        
