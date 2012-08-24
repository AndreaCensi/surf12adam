from ggs import GenericGraphSearch
from contracts import contract
from diffeoplan.library.analysis.structure.plan_reducer import PlanReducer
from diffeoplan.library.images.uncertain_image import UncertainImage
from diffeoplan.utils.memoization import memoize_instance
from boot_agents.diffeo.diffeomorphism2d import Diffeomorphism2D
from diffeoplan.library.discdds.diffeo_action import DiffeoAction
import numpy as np
from diffeoplan.library.discdds.visualization.guess import guess_state_space
import collections

class DiffeoTreeSearch(GenericGraphSearch):
    """
        A tree whose nodes are plans, edges are actions,
        and that already knows:
        1) that some plans are equivalent (modulo canonization by planreducer)
        2) that there is sono
        
    """
    @contract(plan_reducer=PlanReducer)
    def __init__(self, id_dds, dds, plan_reducer, max_depth, max_iterations):
        self.id_dds = id_dds
        self.dds = dds
        self.plan_reducer = plan_reducer
        self.max_iterations = max_iterations
        self.max_depth = max_depth
        GenericGraphSearch.__init__(self)

    @contract(node='tuple,seq(int)')    
    def next_node(self, node, action):
        child = node + (action,)
        return tuple(self.plan_reducer.get_canonical(child))
    
    def available_actions(self, node): #@UnusedVariable
        if len(node) >= self.max_depth:
            return []
        if self.iterations >= self.max_iterations:
            return []
        nactions = len(self.dds.actions)
        return range(nactions)
    
    #@memoize_instance
    @contract(plan='tuple', returns=DiffeoAction)
    def plan2action(self, plan):
        if len(plan) == 0:
            shape = self.dds.actions[0].diffeo.get_shape()
            identity_cmd = np.array([0, 0])
            return DiffeoAction.identity('id', shape, identity_cmd) # XXX
        
        last = self.dds.actions[plan[-1]]
        if len(plan) == 1:
            return last
        else:
            rest = self.plan2action(plan[:-1])
            return DiffeoAction.compose(last, rest)
         
    @memoize_instance
    @contract(plan='tuple', returns=Diffeomorphism2D)
    def compute_diffeomorphism(self, plan):
        action = self.compute_action(plan)
        return action.get_diffeo2d_forward()

    def plot_graph_using_guessed_statespace(self, pylab,
            plan2color=None, plan2label=None, cmap=None, origin=None):
        ss = guess_state_space(self.id_dds, self.dds) 
        
        def plan2xy(plan):
            commands = self.dds.indices_to_commands(plan)
            state = ss.state_from_commands(commands, start=origin)
            xy = ss.xy_from_state(state)
            return 5 * xy
       
        if plan2color is None:
            plan2color = lambda plan: [0, 0, 0] #@UnusedVariable
        if plan2label is None:
            plan2label = lambda plan: '' #@UnusedVariable
        
        nodes = list(self.G.nodes())
         
        pos = dict((n, plan2xy(n)) for n in nodes)
        all_positions = map(tuple, pos.values())
        node_color = map(plan2color, nodes) 
        labels = dict((n, plan2label(n)) for n in self.G)
        
        if len(all_positions) != len(set(all_positions)):
            print('Warning, overlapping nodes')
            y = collections.Counter(all_positions)
            for p, num in y.items():
                if num > 1:
                    print('- %d for %s' % (num, p))
                    for node, node_pos in pos.items():
                        if tuple(node_pos) == p:
                            print('  - %s ' % str(node))

        nx.draw_networkx(self.G, with_labels=True, pos=pos, labels=labels,
                        node_color=node_color, cmap=cmap)
        

import networkx as nx

class DiffeoTreeSearchImage(DiffeoTreeSearch):
    """
        It also knows that:
            - each plan gives an image
    """    
     

    def __init__(self, image, *args, **kwargs):
        DiffeoTreeSearch.__init__(self, *args, **kwargs)
        self.image = image
    
    @memoize_instance
    @contract(returns=UncertainImage)
    def plan2image(self, plan):
        action = self.plan2action(plan)
        return action.predict(self.image)




