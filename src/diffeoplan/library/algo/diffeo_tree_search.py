from . import np, contract
from boot_agents.diffeo import Diffeomorphism2D
from diffeoplan.library.analysis import PlanReducer
from diffeoplan.library.discdds import DiffeoAction, guess_state_space
from diffeoplan.utils import WithInternalLog, memoize_instance
from ggs import GenericGraphSearch
import collections
import networkx as nx
from diffeoplan.library.discdds.diffeo_system import DiffeoSystem
 

class DiffeoTreeSearch(GenericGraphSearch, WithInternalLog):
    """
        A tree whose nodes are plans, edges are actions,
        and that already knows:
        1) that some plans are equivalent (modulo canonization by planreducer)
        2) that there is sono
        
    """
    @contract(id_dds='str', dds=DiffeoSystem,
              plan_reducer=PlanReducer, max_depth='int,>=0', max_iterations='int,>=0')
    def __init__(self, id_dds, dds, plan_reducer, max_depth, max_iterations):
        self.id_dds = id_dds
        self.dds = dds
        self.plan_reducer = plan_reducer
        self.max_iterations = max_iterations
        self.max_depth = max_depth
        GenericGraphSearch.__init__(self)
        WithInternalLog.__init__(self)
        
    def __str__(self):
        return 'DiffeoTreeSearch'

    @contract(node='tuple,seq(int)')    
    def next_node(self, node, action):
        child = node + (action,)
        return tuple(self.plan_reducer.get_canonical(child))
    
    def available_actions(self, node): #@UnusedVariable
        if len(node) >= self.max_depth:
            self.log_node_too_deep(node)
            return []
        if self.iterations >= self.max_iterations:
            self.log_too_many_iterations(node)
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
    
    def log_chosen(self, node):
        pass
        #self.info('chosen %s' % self.node_friendly(node))

    def log_actionless_node(self, node):
        self.info('No actions available for %s' % self.node_friendly(node))

    def log_child_discarded(self, node, action, child, matches): #@UnusedVariable
        self.info('%s->%s but discarded' % (self.node_friendly(node),
                                              self.node_friendly(child)))
        
    def log_child_equivalent_found(self, node, action, child, match): #@UnusedVariable
        self.info('%s->%s but equivalent found' % (self.node_friendly(node),
                                              self.node_friendly(child)))
 
    def log_node_too_deep(self, node):
        self.info('Node %s is too deep. (%s)' % (self.node_friendly(node),
                                                 self.max_depth))

    def log_too_many_iterations(self, node):
        self.info('Cutting because iterations = %s' % self.iterations)

