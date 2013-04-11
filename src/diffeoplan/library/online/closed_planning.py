'''
Created on Oct 22, 2012

@author: adam
'''
from . import logger
from .online_planning import OnlinePlanning
from diffeoplan.library.online.online_stats import OnlineStats

class ClosedPlanning(OnlinePlanning):
    def run_planning(self, active_instance):
        active_instance.labels['method'] = 'ClosedPlanning'
        planning_done = False
        plan = ()
        plan_list = []
        while not planning_done:
            # Capture the current image
            y_start = self.orbit_module.get_image()
            if not hasattr(active_instance, 'y_start'):
                active_instance.y_start = y_start
            
            precision, min_visibility = self.get_planning_thresholds(algo=self.algo,
                                                                discdds=self.discdds,
                                                                active_instance=active_instance)
            active_instance.labels['precision'] = precision 
            active_instance.labels['min_visibility'] = min_visibility
            
            active_instance.precision = precision 
            active_instance.min_visibility = min_visibility
            
            planning_result = self.algo.plan(y_start, active_instance.y_goal, precision=precision,
                                        min_visibility=min_visibility)
            if not planning_result.success:
                plan_found = ()
#                return None
#            logger.info('Plan found: ' + str(planning_result.plan))
    
            plan_found = self.orbit_module.inverse_plan(planning_result.plan)
            
            if len(plan_found) == 0:
                # planning is done
                planning_done = True
            else:
                # select first command to execute and add to total plan
                plan_exec = (plan_found[0],)
                plan = plan + plan_exec
                self.orbit_module.execute_plan(plan_exec)
            plan_list.append(plan_found)
            
            # If commands are inverses, planning_done
            if len(plan) >= 2 and len(self.diffeo_struct.get_canonical(plan[-2:])) == 0:
                logger.info('removing inverse commands and return')
#                plan = plan[:-2]
#                precision = precision / 2
                planning_done = True
                
            # Interrupt to avoid infinite loops
            if len(plan_list) > 10:
                planning_done = True
                
            
            logger.info(' ')
            logger.info('plan_found:     ' + str(plan_found) + '        true plan was: ' + str(active_instance.labels['plan_true']))
            logger.info(' ')
            logger.info('plan and plan_list is now: ' + str(plan))
            for pl in plan_list:
                logger.info('    ' + str(pl))
            logger.info(' ')
            
        
        # Update active_instance with results
        active_instance.plan_found = plan
        active_instance.plan_found_reduced = self.diffeo_struct.get_canonical(plan)
        active_instance.y_result = self.orbit_module.get_image()
        
        active_instance.labels['plan_found'] = plan
        active_instance.labels['plan_found_reduced'] = self.diffeo_struct.get_canonical(plan)



    def run_test(self, plan, env='default'):
        # Initiate stats object
        labels = {}
        labels['id_discdds'] = self.id_discdds
        labels['diffeo_structure_threshold'] = self.diffeo_structure_threshold
        labels['id_algo'] = self.id_algo
        labels['plan_length'] = self.plan_length
        labels['get_planning_thresholds'] = self.get_planning_thresholds_name
        labels['env'] = env
        active_instance = OnlineStats(labels, self.metric_goal)
        
        # Setup the planning problem
        self.create_testcase(plan_true=plan,
                             active_instance=active_instance)
        
        # Run the planning problem
        self.run_planning(active_instance=active_instance)
        
        self.prediction_images(active_instance)
        # return info about test
        return active_instance
    
    def create_testcase(self, plan_true, active_instance):
        # The goal image is where we start the demo
#        self.y_goal = self.orbit_module.get_image()
        active_instance.y_goal = self.orbit_module.get_image()
        active_instance.labels['plan_true'] = plan_true
        active_instance.labels['plan_true_reduced'] = self.diffeo_struct.get_canonical(plan_true)
        active_instance.plan_true = plan_true
        active_instance.plan_true_reduced = self.diffeo_struct.get_canonical(plan_true)
        
        # Move the camera to the start position
        plan_inverse = self.orbit_module.inverse_plan(plan_true)
        self.orbit_module.execute_plan(plan_inverse)
        
    
