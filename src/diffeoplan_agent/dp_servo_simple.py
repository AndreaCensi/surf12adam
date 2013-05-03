from diffeoplan_agent.dp_agent_simple import DiffeoServoAgentInterface
from contracts import contract
from diffeoplan.library.images.uncertain_image import UncertainImage
from conf_tools.code_specs import instantiate_spec
from diffeoplan.library.discdds.plan_utils import plans_of_max_length
from boot_agents.diffeo.analysis.pil_utils import resize
from reprep import Report
import numpy as np
from diffeoplan.configuration.master import get_dp_config
from diffeoplan.library.analysis.structure.diffeo_structure import DiffeoStructure


class DiffeoPlanServoSimple(DiffeoServoAgentInterface):
            
    def __init__(self, plans_generator, reduce_tolerance, distance):
        self.reduce_tolerance = reduce_tolerance
        self.plans_generator = instantiate_spec(plans_generator)
        _, self.distance = get_dp_config().distances.instance_smarter(distance)

    def set_discdds(self, discdds):
        self.discdds = discdds
        plans = self.plans_generator.get_plans(discdds)
        reducer = DiffeoStructure(discdds, tolerance=self.reduce_tolerance)
        red_plans, _ = reducer.get_minimal_equiv_set(plans)
        # add back the empty plan
        red_plans.append(())
        print('reduced %d plans to %d' % (len(plans), len(red_plans)))
        self.plans = red_plans 
        self.actions = [discdds.plan2action(p) for p in self.plans]
        self.actions_i = [a.inverse() for a in self.actions]
        
        print self.plans 
            
    def init(self, boot_spec):
        # TODO: check consistency
        self.boot_spec = boot_spec

    def process_observations(self, bd):
        y = bd['observations']
        self.current = self.obs2ui(y)
    
    @contract(returns='array')
    def choose_commands(self):
        scores = [self.distance.distance(ui, self.current) for ui in self.a_pred]
        order = np.argsort(scores)
        
#         if False:
        for i in range(3):
            self.info('plan #%d: score %10f plan: %s' % 
                  (i, scores[order[i]], self.plans[order[i]]))

        action = self.actions[order[0]]
        cmds = action.get_original_cmds()
        
        cmd = cmds[0]
        
        return cmd

    @contract(goal='array')
    def set_goal_observations(self, goal):
        self.goal = self.obs2ui(goal)        
        self.a_pred = [a.predict(self.goal) for a in self.actions_i]
        
        r = Report('set_goal_observations')
        self.report(r)
        r.to_html('set_goal_observations.html')
        
    def report(self, r):
        f = r.figure(cols=5)
        for i, ui in enumerate(self.a_pred):
            name = 'cmd%03d' % i
            rgb = ui.get_rgba()
            f.data_rgb(name, rgb)
        
        
    @contract(returns=UncertainImage, y='array[HxWx3](float32)')
    def obs2ui(self, y):
        """ Converts observatiosn from outside in an UncertainImage
            of the right dimensions to deal with this DDS. """
        shape = self.discdds.get_shape()
        y = (y * 255).astype('uint8')
        y = resize(y, width=shape[1], height=shape[0])
        ui = UncertainImage(y)
        return ui
        
class AllMaxLength:
    
    def __init__(self, nsteps):
        self.nsteps = nsteps
    
    def get_plans(self, discdds):
        ncmd = discdds.get_num_commands()
        plans = plans_of_max_length(ncmd, self.nsteps)
        return plans

class AllMaxLengthMult:
    
    @contract(mult='int')
    def __init__(self, nsteps, mult):
        self.nsteps = nsteps
        self.mult = mult
    
    def get_plans(self, discdds):
        ncmd = discdds.get_num_commands()
        plans = plans_of_max_length(ncmd, self.nsteps)
        plans = map(self.multiply, plans)
        return plans
    
    def multiply(self, plan):
        res = []
        for cmd in plan:
            res.extend([cmd] * self.mult)
        return res
        
        

class AllMaxLengthMult2:
    
    @contract(mult='list(int)')
    def __init__(self, nsteps, mult):
        self.nsteps = nsteps
        self.mult = mult
    
    def get_plans(self, discdds):
        ncmd = discdds.get_num_commands()
        plans = plans_of_max_length(ncmd, self.nsteps)
        res = []
        for p in plans:
            res.extend(self.multiply(p))
        return list(set(res))
        
    
    def multiply(self, plan):
        for mult in self.mult:
            res = []
            for cmd in plan:
                res.extend([cmd] * mult)
            yield tuple(res)
        


