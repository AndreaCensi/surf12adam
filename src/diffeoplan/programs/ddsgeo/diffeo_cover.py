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
from geometry.formatting import printm
from geometry.mds_algos import mds
from reprep.plot_utils.axes import turn_all_axes_off
import matplotlib
import random


class DiffeoCover(GenericGraphSearch):
    
    def __init__(self, id_dds, dds, ds, info_threshold, min_visibility=0.5,
                 debug_it=1000, max_it=1000):
        '''
        
        :param id_dds:
        :param dds:
        :param ds:
        :param info_threshold: info distance for two plans to be the same node 
        :param debug_it: Number of iterations for drawing debug graph.
        '''
        self.info_threshold = info_threshold
        self.dds = dds
        self.ds = ds
        self.id_dds = id_dds
        self.debug_graph_iterations = debug_it
        self.max_it = max_it
        self.min_visibility = min_visibility
        printm('samew', self.ds.samew,
               'oppow', self.ds.oppositew,
               'swapw', self.ds.swappablew)
        
        GenericGraphSearch.__init__(self,
                                    choose_open_nodes=breadth_first,
                                    available_actions=self.available_actions,
                                    next_node=self.next_node,
                                    node_compare=self.node_compare)
    
    @memoize
    @contract(plan='tuple', returns=DiffeoAction)
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
    
    @memoize
    @contract(plan1='seq(int)', plan2='seq(int)')
    def node_compare(self, plan1, plan2):
        plan1 = tuple(plan1)
        plan2 = tuple(plan2)
        if plan1 == plan2:
            return True
        dn = self.plan_distance(plan1, plan2, DiffeoAction.distance_L2_infow) / self.ds.scalew
        
        match = dn < self.info_threshold
        if match:
            #print('%10g  %4d %4d' % (dn, len(plan1), len(plan2)))
            print('%10g  %40s %40s' % (dn, plan1, plan2))
        
        return match 
    
    def next_node(self, node, action):
        child = node + (action,)
        return tuple(self.ds.get_canonical(child))
    
    def available_actions(self, node): #@UnusedVariable
        if self.iterations >= self.max_it:
            return []
        if self.visibility(node) < self.min_visibility:
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
        if (self.iterations + 1) % self.debug_graph_iterations == 0:
            self.draw_graph()

    def draw_graph(self):
        use_distance = DiffeoAction.distance_L2_infow
#        use_distance = DiffeoAction.distance_L2
        use_distance = DiffeoAction.distance_L2_infow_scaled
        
        def label_plan_length(n):
            return  len(n)
        
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
        
        f = r.figure(cols=2)
        nolabel = lambda _: ''
        with f.data_file('planlength', MIME_PNG) as filename:
            draw_node_graph(filename=filename,
                            G=self.G,
                            pos_func=point_func,
                            #label_func=label_plan_length,
                            label_func=nolabel,
                            color_func=label_plan_length)
         
        with f.data_file('nchildopen', MIME_PNG) as filename:
            draw_node_graph(filename=filename,
                        G=self.G,
                        pos_func=point_func,
#                        label_func=label_child_open,
                        label_func=nolabel,
                        color_func=label_child_open)
    
        with f.data_file('info', MIME_PNG) as filename:
            draw_node_graph(filename=filename,
                        G=self.G,
                        pos_func=point_func,
                        #label_func=lambda x: '%.2f' % self.visibility(x),
                        label_func=nolabel,
                        color_func=self.visibility)
            
            
            
        with f.plot('visible') as pylab:
            v = map(self.visibility, self.G)
            pylab.hist(v, 100)

        self.draw_embedding(r.section('embedding'), distance=use_distance)
        
        all_plans = list(self.G.nodes())
        random.shuffle(all_plans)
        random_plans = all_plans[:3]
        for pi, ref_plan in enumerate(random_plans):
            color_func = lambda p2: self.plan_distance(ref_plan, p2)
            name = 'random%d' % pi
            with f.data_file(name, MIME_PNG) as filename:
                draw_node_graph(filename=filename,
                        G=self.G,
                        pos_func=point_func,
                        #label_func=lambda x: '%.2f' % self.visibility(x),
                        label_func=nolabel,
                        color_func=color_func)
            
        outdir = 'out/graphs/%s/' % self.id_dds
        filename = os.path.join(outdir, 'graphs.html')
        logger.info('Writing to %r' % filename)
        r.to_html(filename)

    @memoize
    def plan_distance(self, p1, p2, action_distance):
        a1 = self.compute_action(p1)
        a2 = self.compute_action(p2)
        return action_distance(a1, a2)
        
    def get_distance_matrix(self, action_distance=DiffeoAction.distance_L2_infow):
        plans = list(self.G.nodes())
        plans = sorted(plans, key=lambda x: len(x))
        #print "\n".join(map(str, plans))
        n = len(plans)
        D_ij = lambda i, j: self.plan_distance(plans[i], plans[j],
                                               action_distance=action_distance)
        D = construct_matrix((n, n), D_ij)
        return plans, D                         
        
        
    def draw_embedding(self, report, distance):
        f = report.figure('distances')        
        plans, D = self.get_distance_matrix(distance)
        
        plan2index = dict([(p, i) for i, p in enumerate(plans)])
        
        Dn = D #/ self.ds.scalew
        caption = 'min: %g max %g ' % (Dn.min(), Dn.max())
        report.data('Dn', Dn).display('scale').add_to(f, caption)
        with f.plot('D_stats') as pylab:
            pylab.hist(D.flat, 100)


        if D.shape[0] < 20:
            report.table('D', Dn, fmt='%.1f')
        
        p2 = mds(D, 2)
        p3 = mds(D, 3)
        color = [self.visibility(plan) for plan in plans]    
        
        f = report.figure('embedding')

        def get_edges_indices():
            for plan1 in self.G:
                for plan2 in self.G[plan1]:
                    i = plan2index[plan1]
                    j = plan2index[plan2]
                    yield i, j

        with f.plot('3D') as pylab:
            cm = matplotlib.cm.get_cmap('RdYlBu')
            import mpl_toolkits.mplot3d.axes3d as plot3
            fig = pylab.gcf()
            ax = plot3.Axes3D(fig)
            for i, j in get_edges_indices():
                coords = np.vstack((p3[:, i], p3[:, j])).T
                ax.plot3D(xs=coords[0], ys=coords[1], zs=coords[2],
                              linestyle='-', color=[0.8, 0.8, 0.8])
            p = ax.scatter3D(p3[0], p3[1], p3[2], s=40, c=color, cmap=cm)                    
            fig.colorbar(p)
                        
        with f.plot('2D') as pylab:
            pylab.jet()
            for i, j in get_edges_indices():
                coords = np.vstack((p2[:, i], p2[:, j])).T
                pylab.plot(coords[0], coords[1],
                              linestyle='-', color=[0.8, 0.8, 0.8])
            pylab.scatter(p2[0], p2[1], s=40, c=color)
            pylab.axis('equal')
            
            turn_all_axes_off(pylab)
            pylab.colorbar()
