from . import logger
from diffeoplan.configuration import get_current_config
from diffeoplan.library.analysis.structure.diffeo_structure import (
    DiffeoStructure)
from diffeoplan.library.discdds.diffeo_system import DiffeoSystem
from diffeoplan.library.distances.distance_norm import DistanceNorm
import itertools
import numpy as np
 
class ComposeGraph():
    def __init__(self, id_discdds, plan_length):
        '''
        
        '''
        self.plan_time = 1
        
        self.metric = DistanceNorm(2)
        
#        config_path = 'default:/home/adam/diffeo-data/'
#        config = DiffeoplanConfigMaster()
#        config.load(config_path)
#        set_current_config(config)
        config = get_current_config()
        
        self.discdds = config.discdds.instance(id_discdds)
        
        
        # Initiate diffeo_structure
        diffeo_structure_threshold = 0.2
        self.diffeo_struct = DiffeoStructure(self.discdds, diffeo_structure_threshold)
#        pdb.set_trace()
        n = len(self.discdds.actions)

        
        self.all_plans = []
        for i in range(1, plan_length + 1):
            self.all_plans += list(itertools.product(range(n), repeat=i))
        
        
        self.plan_reduced = []
        for plan in self.all_plans:
            canon_plan = self.diffeo_struct.get_canonical(plan)
            if self.plan_reduced.count(canon_plan) == 0 and len(canon_plan) >= 1:
                self.plan_reduced.append(canon_plan)
        
        logger.info('Total number of plans initially: %g ' % len(self.all_plans))
        logger.info('Number of plans after reduction: %g ' % len(self.plan_reduced))
        print('Total number of plans initially: %g ' % len(self.all_plans))
        print('Number of plans after reduction: %g ' % len(self.plan_reduced))
        
        self.generate_diffeo(self.plan_reduced)
        self.D = np.matrix(self.composed_discdds.actions_distance_L2())
        
        logger.info('Action Graph Composed')
        
    def generate_diffeo(self, plan_reduced):
        self.composed_actions = []
        for plan in plan_reduced:
            logger.info('Diffeomorphism generated for plan: ' + str(plan))
            self.composed_actions.append(self.discdds.plan2action(plan))
        self.composed_discdds = DiffeoSystem('Composed diffeo system', self.composed_actions)

        
def main():
    id_discdds = 'orbit-pt256-40-n35s'
    ComposeGraph(id_discdds, 3)
    
def randpmf(pmf):
    # Enforce pmf to be 1-d numpy array
    pmf = np.array(pmf).flatten()
    PF = np.array(np.add.accumulate(pmf)).flatten()
    tot = PF[-1]
#    print('total value of PF = %g' % tot)
    rand = np.random.ranf() * tot
#    print('random value      = %g' % rand)
    for i in range(len(PF) - 1):
        if PF[i] < rand and PF[i + 1] > rand:
            return i
    return len(PF) - 1
    
if __name__ == '__main__':
    main()
