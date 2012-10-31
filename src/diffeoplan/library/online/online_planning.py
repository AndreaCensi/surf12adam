'''
Created on Oct 22, 2012

@author: adam
'''
from . import logger
from diffeoplan.configuration import get_current_config
from diffeoplan.programs.bench.bench_jobs import init_algorithm
from diffeoplan.library.online.orbit_module import OrbitModule
from diffeoplan.library.analysis.structure.diffeo_structure import DiffeoStructure
from diffeoplan.library.online.online_stats import OnlineStats
import numpy as np
import pdb

class OnlinePlanning():
    def __init__(self, id_discdds,
                 diffeo_structure_threshold,
                 id_algo,
                 plan_length,
                 num_tests,
                 get_planning_thresholds,
                 plans):
        
        config = get_current_config()
        self.config = config
        
        
        # Load objects from configuration manager
        self.discdds = config.discdds.instance(id_discdds)
        self.algo = init_algorithm(self.config, id_algo, id_discdds, self.discdds)
        self.cmdlist = [self.discdds.actions[i].original_cmd for i in range(len(self.discdds.actions))]
#        pdb.set_trace()
        self.metric_goal = self.algo.metric_goal 
        # Save input arguments
        self.id_discdds = id_discdds
        self.id_algo = id_algo
        self.diffeo_structure_threshold = diffeo_structure_threshold
        self.plan_length = plan_length
        self.num_tests = num_tests
        
        # Load orbit camera module
        self.orbit_module = OrbitModule(self.discdds.get_shape())
        
        # Set get_planning_thresholds function
        if get_planning_thresholds.__class__ == 'str':
            try:
                self.get_planning_thresholds = eval(get_planning_thresholds)
            except:
                logger.info()  
        else:
            self.get_planning_thresholds = get_planning_thresholds
        self.get_planning_thresholds_name = str(get_planning_thresholds)
        
        # Initiate diffeo_structure
        self.diffeo_struct = DiffeoStructure(self.discdds, diffeo_structure_threshold)
        
        if plans == 'random':
            self.plans = self.gennerate_random_plans()
        elif plans.__class__ in [list, tuple]:
            self.plans = tuple(plans) 
        else:
            self.plans = eval(plans)
            
        logger.info('Initialized with plans: ' + str(self.plans))
        
    def gennerate_random_plans(self):
        
        n_cmd = len(self.discdds.actions)
        plans = []
        for _ in range(self.num_tests):
            plan = ()
            ie = 0
            while len(plan) < int(self.plan_length):
                # Add new command
                plan += (np.random.randint(0, n_cmd),)
                # remove redundant command
                plan = self.diffeo_struct.get_canonical(plan)
                ie += 1
                if ie > 50:
                    print('WARNING, a lot of iterations to generate a non redundant plan.')
                    plans.append(plan)
                    break
                
            plans.append(plan)
        return plans
       
    def run_all_tests(self, env='default'):
        all_stats = []
        for plan in self.plans:
            stat = self.run_test(plan, env)
            all_stats.append(stat)
        return all_stats

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
        
#        self.y_start =  self.orbit_module.get_image()
    


    
    def run_planning(self, active_instance):
        # Capture the initial image
        active_instance.y_start = self.orbit_module.get_image()
#        pdb.set_trace()
        if not self.get_planning_thresholds.__class__ in [list, tuple]:
            precision, min_visibility = self.get_planning_thresholds(algo=self.algo,
                                                                     discdds=self.discdds,
                                                                     active_instance=active_instance)
        else:
            precision, min_visibility = self.get_planning_thresholds
            
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
        

    def prediction_images(self, active_instance):
        '''
        predicts the resulting images from active_instance.y_start and plan 
        :param active_instance:
        '''
        if active_instance.plan_found is not None:
            inv_plan = self.orbit_module.inverse_plan(active_instance.plan_found)
            active_instance.y_found_pred = self.discdds.predict(active_instance.y_start, inv_plan)
            
        if active_instance.plan_true is not None:
            inv_plan = self.orbit_module.inverse_plan(active_instance.plan_true)
            active_instance.y_goal_pred = self.discdds.predict(active_instance.y_start, inv_plan)


def get_planning_thresholds_true(algo, discdds, active_instance):
    if active_instance.y_goal is None:
        logger.warn('y_goal is not initialized, using thresholds 0, 0')
        return 0, 0
    # Apply the diffeo on the goal image and use the minimum distance 
    # as a threshold for the planing precision.  
    d = algo.metric_goal.distance(active_instance.y_goal, discdds.predict(active_instance.y_start, active_instance.plan_true))
    precision = d * 1.01
    
    min_visibility = 0
    return precision, min_visibility


def get_planning_thresholds_estimate(algo, discdds, active_instance):
    if active_instance.y_goal is None:
        logger.warn('y_goal is not initialized, using thresholds 0, 0')
        return 0, 0
    # Apply the diffeo on the goal image and use the minimum distance 
    # as a threshold for the planing precision.  
#    pdb.set_trace()
    d = np.min([algo.metric_goal.distance(active_instance.y_goal, action.predict(active_instance.y_goal)) for action in discdds.actions]) / .7
    precision = d * 1.01
    
    min_visibility = 0
    return precision, min_visibility

