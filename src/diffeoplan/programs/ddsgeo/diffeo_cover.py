from . import logger, np
from .ggs import GenericGraphSearch, breadth_first
from contracts import contract
from diffeoplan.library.discdds.diffeo_action import DiffeoAction
from diffeoplan.library.discdds.visualization.guess import guess_state_space
from diffeoplan.programs.ddsgeo.ggs.drawing import draw_node_graph
from diffeoplan.utils import memoize
from reprep import Report, MIME_PNG

import os
from diffeoplan.utils.matrices import construct_matrix


class DiffeoCover(GenericGraphSearch):
    
    def __init__(self, id_dds, dds, ds, info_threshold):
        self.info_threshold = info_threshold
        self.dds = dds
        self.ds = ds
        self.id_dds = id_dds
        GenericGraphSearch.__init__(self,
                                    choose_open_nodes=breadth_first,
                                    available_actions=self.available_actions,
                                    next_node=self.next_node,
                                    node_compare=self.node_compare)
    
    @memoize
    def compute_action(self, plan):
        if len(plan) == 0:
            shape = self.dds.actions[0].diffeo.get_shape()
            identity_cmd = np.array([0, 0])
            return DiffeoAction.identity('id', shape, identity_cmd) # XXX
        
        last = self.dds.actions[plan[-1]]
        if len(plan) == 1:
            return last
        else:
            rest = self.compute_action(plan[:-1])
            return DiffeoAction.compose(last, rest)
         
    @memoize
    def compute_diffeomorphism(self, plan):
        action = self.compute_action(plan)
        return action.get_diffeo2d_forward()
    
    @memoize
    def visibility(self, plan):
        if len(plan) == 0:
            return 1
        diffeo = self.compute_diffeomorphism(plan)
        return np.mean(diffeo.variance) 
    
    @contract(plan1='seq(int)', plan2='seq(int)')
    def node_compare(self, plan1, plan2):
        plan1 = tuple(plan1)
        plan2 = tuple(plan2)
        if plan1 == plan2:
            return True
        dn = self.plan_distance(plan1, plan2) / self.ds.scalew
        
        match = dn < self.info_threshold
        if match:
            #print('%10g  %4d %4d' % (dn, len(plan1), len(plan2)))
            print('%10g  %40s %40s' % (dn, plan1, plan2))
        
        return match
#        cplan1 = self.ds.get_canonical(plan1)
#        cplan2 = self.ds.get_canonical(plan2)
#        return cplan1 == cplan2
    
    def next_node(self, node, action):
        child = node + (action,)
        return tuple(self.ds.get_canonical(child))
    
    def available_actions(self, node): #@UnusedVariable
        if self.visibility(node) < self.info_threshold:
            return []
        nactions = len(self.dds.actions)
        return range(nactions)
     
    def go(self):
        first = ()
        return GenericGraphSearch.go(self, first)
        
    def log_chosen(self, node):
        print('It: %d  closed: %5d  Open: %5d (%5d) Chosen %r' % 
              (self.iterations, len(self.closed), len(self.open_nodes),
               len(set(self.open_nodes)), node))
        if self.iterations % 20 == 0:
            self.draw_graph()

    def draw_graph(self):
        
        def label_plan_length(n):
            return '%d' % len(n)
        
        open_nodes = set(self.open_nodes)
        
        def label_child_open(n):
            # number of children in the open list
            children = self.G[n]
            return len(set(children) & open_nodes)
    
        ss = guess_state_space(self.id_dds, self.dds)
        
        def point_func(plan):
            commands = self.dds.indices_to_commands(plan)
            state = ss.state_from_commands(commands)
            xy = ss.xy_from_state(state)
            return 5 * xy
            
        r = Report(self.id_dds)
        
        f = r.figure()
        with f.data_file('planlength', MIME_PNG) as filename:
            draw_node_graph(filename=filename,
                            G=self.G,
                            pos_func=point_func,
                            label_func=label_plan_length,
                            color_func=self.color_open_closed)
         
        with f.data_file('nchildopen', MIME_PNG) as filename:
            draw_node_graph(filename=filename,
                        G=self.G,
                        pos_func=point_func,
                        label_func=label_child_open,
                        color_func=self.color_open_closed)
    
        with f.data_file('info', MIME_PNG) as filename:
            draw_node_graph(filename=filename,
                        G=self.G,
                        pos_func=point_func,
                        label_func=lambda x: '%.2f' % self.visibility(x),
                        color_func=self.visibility)
        
        outdir = 'out/graphs/%s/' % self.id_dds
        filename = os.path.join(outdir, 'graphs.html')
        logger.info('Writing to %r' % filename)
        r.to_html(filename)

    @memoize
    def plan_distance(self, p1, p2, action_distance=DiffeoAction.distance_L2_infow):
        a1 = self.compute_action(p1)
        a2 = self.compute_action(p2)
        return action_distance(a1, a2)
        
    def get_distance_matrix(self, action_distance=DiffeoAction.distance_L2_infow):
        plans = list(self.G.nodes())
        n = len(plans)
        D_ij = lambda i, j: self.plan_distance(plans[i], plans[j],
                                               action_distance=action_distance)
        D = construct_matrix((n, n), D_ij)
        return D                         
        
