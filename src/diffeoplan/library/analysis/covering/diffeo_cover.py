from . import logger, np, contract
from boot_agents.diffeo import Diffeomorphism2D
from diffeoplan.library.discdds import (diffeoaction_distance_L2_infow,
    plan_friendly, guess_state_space, DiffeoAction)
from diffeoplan.utils import construct_matrix
from diffeoplan.utils.memoize_limits import memoize_limited
from ggs import draw_node_graph, GenericGraphSearch
from reprep import Report, MIME_PNG
import os
import random
from diffeoplan.library.analysis.covering.diffeo_cover_visualization import get_nodes_distance_matrix, \
    edges_type_to_color, plot_3d_graph, get_embedding_mds, plot_2d_graph
from diffeoplan.utils.with_internal_log import WithInternalLog

def dp_memoize_instance(f):
    memoize = memoize_limited(max_size=None, max_mem_MB=100)
    return memoize(f)
        

class DiffeoCover(GenericGraphSearch, WithInternalLog):
    
    def __init__(self, id_dds, dds, ds, collapse_threshold, min_visibility=0.5,
                 debug_it=1000, max_it=1000):
        '''
        
        :param id_dds:
        :param dds:
        :param ds:
        :param collapse_threshold: info distance for two plans to be the same node 
        :param debug_it: Number of iterations for drawing debug graph.
        '''
        WithInternalLog.__init__(self)
        self.collapse_threshold = collapse_threshold
        self.dds = dds
        self.ds = ds
        self.id_dds = id_dds
        self.debug_graph_iterations = debug_it
        self.max_it = max_it
        self.min_visibility = min_visibility
        from geometry import formatm
        self.info('Min_visibility: %s' % min_visibility)
        self.info('collapse_threshold: %s' % collapse_threshold)
        self.info(formatm('samew', self.ds.samew,
                            'oppow', self.ds.oppositew,
                            'swapw', self.ds.swappablew))
        
        GenericGraphSearch.__init__(self)
    
    ######## GenericGraphSearch interface
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

    def choose_open_nodes(self, nodes):
        return self.max_visibility_first(nodes)
    
    @dp_memoize_instance
    @contract(plan1='seq(int)', plan2='seq(int)')
    def node_compare(self, plan1, plan2):
        plan1 = tuple(plan1)
        plan2 = tuple(plan2)
        if plan1 == plan2:
            return True
        
        dn = self.plan_distance(plan1, plan2, diffeoaction_distance_L2_infow) / self.ds.scalew
        
        match = dn < self.collapse_threshold
        if match:
            #print('%10g  %4d %4d' % (dn, len(plan1), len(plan2)))
            logger.info('Found match at %g' % dn)
            logger.info('- plan1: %s' % plan_friendly(plan1))
            logger.info('- plan1: %s' % plan_friendly(plan2))
        
        return match 
    
    def log_child_generated(self, node, action, child): #@UnusedVariable
        #logger.info('Generated %s from %s' % (child, node))
        pass
    
    def log_child_equivalent_found(self, node, action, child, match): #@UnusedVariable
        logger.info('Equiv found: %s = %s' % (child, match))
        
    def log_chosen(self, node):
        all_visibility = map(self.visibility, self.G.nodes())
        
        logger.info('#%4d closed %4d open %4d [minvis: %.4g] pop %s' % 
              (self.iterations, len(self.closed), len(self.open_nodes),
               np.min(all_visibility),
               plan_friendly(node)))
        if (self.iterations + 1) % self.debug_graph_iterations == 0:
            self.save_graph()

    def log_child_discarded(self, node, action, child, matches): #@UnusedVariable
        logger.info('Discarding %s because it matches %s' % (str(child), matches))

    ######## / GenericGraphSearch interface
    
    def max_visibility_first(self, nodes):
        v = map(self.visibility, nodes)
        i = np.argmax(v)
        return nodes[i]
    
    @dp_memoize_instance
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
         
    @dp_memoize_instance
    @contract(plan='tuple', returns=Diffeomorphism2D)
    def compute_diffeomorphism(self, plan):
        action = self.compute_action(plan)
        return action.get_diffeo2d_forward()
    
    @dp_memoize_instance
    @contract(returns='>=0')
    def visibility(self, plan):
        if len(plan) == 0:
            return 1
        diffeo = self.compute_diffeomorphism(plan)
        return np.mean(diffeo.variance) 
     
    def go(self):
        first = ()
        return GenericGraphSearch.go(self, first)
        
    def save_graph(self):
        """ Saves a copy of the progress so far """
        r = Report(self.id_dds)
        outdir = 'out/cover-progress/%s/' % self.id_dds
        self.draw_graph(r)
        filename = os.path.join(outdir, 'graphs.html')
        logger.info('Writing to %r' % filename)
        r.to_html(filename, write_pickle=True)

        
    def draw_graph(self, r):
        
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

        with r.subsection('embedding') as s:
            self.draw_embeddings(s)
        
        all_plans = list(self.G.nodes())
        random.shuffle(all_plans)
        random_plans = all_plans[:3]
        for pi, ref_plan in enumerate(random_plans):
            color_func = lambda p2: self.plan_distance(ref_plan, p2,
                                            diffeoaction_distance_L2_infow)
            name = 'random%d' % pi
            with f.data_file(name, MIME_PNG) as filename:
                draw_node_graph(filename=filename,
                        G=self.G,
                        pos_func=point_func,
                        #label_func=lambda x: '%.2f' % self.visibility(x),
                        label_func=nolabel,
                        color_func=color_func)
            

    @dp_memoize_instance
    def plan_distance(self, p1, p2, action_distance):
        a1 = self.compute_action(p1)
        a2 = self.compute_action(p2)
        return action_distance(a1, a2)
        
    def get_plans_sorted(self):
        plans = list(self.G.nodes())
        plans = sorted(plans, key=lambda x: len(x))
        return plans
    
    def get_distance_matrix(self, plans, action_distance):
        n = len(plans)
        D_ij = lambda i, j: self.plan_distance(plans[i], plans[j],
                                               action_distance)
        D = construct_matrix((n, n), D_ij)
        return plans, D                         
        
    def set_edge_field_to_plan_distance(self, field_name, function):
        """ Sets the given field to the given function(plan1, plan2) 
            between plans. """
        for plan1 in self.G:
            for plan2 in self.G[plan1]:
                # remember multigraph
                distance = function(plan1, plan2)
                for edge in self.G[plan1][plan2].values():
                    edge[field_name] = distance
    
    @contract(returns='tuple(list, list(tuple(str, array[NxN])))')
    def compute_distances(self):
        """ Computes a bunch of distance measures. 
            Returns plans, list of name, distance
        """
        plans = self.get_plans_sorted()
        distances = []
        
        # plan-to-plan distance
        _, D_L2_infow = self.get_distance_matrix(plans, diffeoaction_distance_L2_infow)
        
        # plan-to-plan distance, but only on edge path
        function = lambda p1, p2: self.plan_distance(p1, p2, diffeoaction_distance_L2_infow)
        self.set_edge_field_to_plan_distance('L2_infow', function)
        D_L2_infow_edges = get_nodes_distance_matrix(self.G, plans, 'L2_infow')
        
        D_pathlength = get_nodes_distance_matrix(self.G, plans)
        
        distances.append(('L2_infow', D_L2_infow))
        distances.append(('L2_infow_edges', D_L2_infow_edges))
        distances.append(('pathlength', D_pathlength))
        
        return plans, distances
    
        
    def draw_embeddings(self, report):
        print('computing distances...')
        plans, distances = self.compute_distances()
        print('done')
        for name, D in distances: 
            print name   
            self.draw_embedding(report.section(name), plans, D)
        
    @contract(plans='seq(tuple)', D='array[NxN]')
    def draw_embedding(self, report, plans, D):
        
        f = report.figure('distances')
        
        report.data('plans', plans)
        report.data('distance', D).display('scale').add_to(f)
        with f.plot('D_stats', caption='Distance') as pylab:
            pylab.hist(D.flat, 100)

        if len(plans) < 3:
            msg = 'Cannot draw embedding with only %d plans. ' % (len(plans))
            report.text('warn', msg)
            return

        edges2color = lambda n1, n2: edges_type_to_color(self.G, n1, n2)

        with report.subsection('embed2d') as r2d:
            f = r2d.figure('embedding')
            with f.plot('2D') as pylab:
                plan2point2 = get_embedding_mds(plans, D, ndim=2)
                plan2color = self.visibility
                plot_2d_graph(pylab, self.G, plan2point2.__getitem__, plan2color,
                              edges2color)

        with report.subsection('embed3d') as r3d:
            f = r3d.figure()
            with f.plot('3D') as pylab:
                plan2point3 = get_embedding_mds(plans, D, ndim=3)
                plan2color = self.visibility
                plot_3d_graph(pylab, self.G, plan2point3.__getitem__,
                              plan2color, edges2color)
            
            
    def color_open_closed(self, n):
        if n in self.closed:
            return 0.6
        else:
            return 0.8



