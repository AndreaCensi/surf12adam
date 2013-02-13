'''
Created on Oct 16, 2012

@author: adam
'''
from . import logger
from .online_planning import OnlinePlanning

class OpenPlanning(OnlinePlanning):
    def run_planning(self, active_instance):
        # Capture the initial image
        active_instance.y_start = self.orbit_module.get_image()
        
        precision, min_visibility = self.get_planning_thresholds(algo=self.algo,
                                                            discdds=self.discdds,
                                                            active_instance=active_instance)
        active_instance.labels['precision'] = precision 
        active_instance.labels['min_visibility'] = min_visibility
        
        active_instance.precision = precision 
        active_instance.min_visibility = min_visibility
        
        planning_result = self.algo.plan(active_instance.y_start, active_instance.y_goal, precision=precision,
                                    min_visibility=min_visibility)
        if not planning_result.success:
            return None
        logger.info('Plan found: ' + str(planning_result.plan))

        plan_found = self.orbit_module.inverse_plan(planning_result.plan)
        self.orbit_module.execute_plan(plan_found)
        
        # Update active_instance with results
        active_instance.plan_found = plan_found
        active_instance.plan_found_reduced = self.diffeo_struct.get_canonical(plan_found)
        active_instance.y_result = self.orbit_module.get_image()
        
        active_instance.labels['plan_found'] = plan_found
        active_instance.labels['plan_found_reduced'] = self.diffeo_struct.get_canonical(plan_found)
