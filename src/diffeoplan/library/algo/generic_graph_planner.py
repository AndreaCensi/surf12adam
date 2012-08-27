from . import DiffeoPlanningAlgo, PlanningResult, contract
from .. import UncertainImage
from diffeoplan.configuration import get_current_config
from diffeoplan.library.algo import Connector, DiffeoTreeSearchImage
from diffeoplan.library.analysis import PlanReducer
from diffeoplan.library.discdds import (DiffeoSystem, guess_state_space,
    plan_friendly)
from matplotlib.cm import get_cmap
from reprep.plot_utils import turn_all_axes_off

 
class GenericGraphPlanner(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 
        
        Pieces:
        - metric
        - expand_start_tree
        - expand_goal_tree
    """
    
    def __init__(self, bidirectional,
                 metric_goal, metric_goal_threshold,
                 metric_collapse, metric_collapse_threshold,
                 max_depth=10000, max_iterations=10000):
        DiffeoPlanningAlgo.__init__(self)
        self.bidirectional = bidirectional
        config = get_current_config()
        self.metric_goal = config.distances.instance(metric_goal)        
        self.metric_goal_threshold = metric_goal_threshold
        self.metric_collapse = config.distances.instance(metric_collapse)        
        self.metric_collapse_threshold = metric_collapse_threshold
        self.max_iterations = max_iterations
        self.max_depth = max_depth
        
    def __strparams__(self):
        return ("%s;g:%s<=%s;c:%s<=%s" % 
                ('1' if self.bidirectional else '2',
                 self.metric_goal,
                 self.metric_goal_threshold,
                 self.metric_collapse,
                 self.metric_collapse_threshold))

    def __str__(self):
        return 'GenericGraphPlanner(%s)' % self.__strparams__()
        
    @contract(dds=DiffeoSystem)
    def init(self, id_dds, dds):
        """ Might be redefined to add precomputation. """
        DiffeoPlanningAlgo.init(self, id_dds, dds)
    
    
    def plan_init(self, y0, y1):
        self.y0 = y0
        self.y1 = y1
        self.start_tree = self.init_start_tree(y0)
        self.goal_tree = self.init_goal_tree(y1)
        plan0 = ()
        self.start_tree.init_search(plan0)
        self.goal_tree.init_search(plan0)

        self.connector = self.init_connector(self.start_tree, self.goal_tree)
        
        self.log_planning_init()

    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1):
        self.plan_init(y0, y1)
        
        while self.start_tree.has_next() or self.goal_tree.has_next():
            if self.start_tree.has_next():
                added1 = self.start_tree.do_iteration()
                self.log_start_tree_expanded(added1)
            else:
                added1 = []
                
            if self.goal_tree.has_next(): 
                added2 = self.goal_tree.do_iteration()
                self.log_goal_tree_expanded(added2)
            else:
                added2 = []
                
            self.connector.update(added1, added2)
            connections = list(self.connector.get_connections())
            if connections:
                self.log_connections_found(connections)
                # let's choose any one
                connection = connections[0]
                d, p1, p2 = connection 
                # this can be redundant
                plan_red = p1 + tuple(reversed(p2))
                # let's simplify it
                plan = self.get_plan_reducer().get_canonical(plan_red)
                self.log_plan_found(d, p1, p2, plan_red, plan)
                
                return PlanningResult(True, plan, 'Found',
                                      extra=self.make_extra())        
        self.log_planning_failed()
        
        return PlanningResult(False, None, 'not found',
                              extra=self.make_extra())
    
    def log_start_tree_expanded(self, added):
        if False:
            if added:
                added = map(self.start_tree.node_friendly, added)
                self.info('Adding to start tree: %s' % ", ".join(added))
            else:
                self.info('Nothing added to start tree.')
    
    def log_goal_tree_expanded(self, added):
        if False:
            if added:
                added = map(self.goal_tree.node_friendly, added)
                self.info('Adding to goal tree: %s' % ", ".join(added))
            else:
                self.info('Nothing added to goal tree.')
                pass
    
    def log_connections_found(self, connections):
        self.info('Found %d connections' % len(connections))
        for c in connections:
            d, p1, p2 = c
            self.info(' - between %s and %s' % 
                      (self.start_tree.node_friendly(p1),
                       self.goal_tree.node_friendly(p2)))
            
    def log_planning_init(self):
        self.info('Planning started')
        
    
    def log_planning_failed(self):
        self.info('Planning failed')
        
    def log_plan_found(self, d, p1, p2, plan_red, plan):
        self.info('Connection between %s and %s' % 
                  (self.start_tree.node_friendly(p1),
                   self.goal_tree.node_friendly(p2)))
        self.info(' concat: %s' % plan_friendly(plan_red))
        self.info('reduced: %s' % plan_friendly(plan))
        
    def make_extra(self):
        """ Extra information to return about the search """
        extra = DiffeoPlanningAlgo.make_extra(self)
        extra['start_tree'] = self.start_tree
        extra['goal_tree'] = self.goal_tree
        extra['connector'] = self.connector
        return extra

    def init_connector(self, start_tree, end_tree):
        connector = Connector(start_tree, end_tree,
                              self.metric_goal, self.metric_goal_threshold)
        return connector
    
    def get_plan_reducer(self):
        """ Returns a dummy plan reducer. """
        dds = self.get_dds()
        nactions = len(dds.actions)
        plan_reducer = PlanReducer.dummy(labels=range(nactions))
        return plan_reducer
     
    def init_start_tree(self, y0):
        """
            Start tree, by default first node open, may be override by subclass.  
        """
        dds = self.get_dds()
        id_dds = self.id_dds
        max_depth = self.max_depth 
        max_iterations = self.max_iterations
        plan_reducer = self.get_plan_reducer()
        dts = DiffeoTreeSearchImage(image=y0, id_dds=id_dds,
                        dds=dds, plan_reducer=plan_reducer,
                        max_depth=max_depth, max_iterations=max_iterations,
                        metric_collapse=self.metric_collapse,
                        metric_collapse_threshold=self.metric_collapse_threshold)
        return dts
    
    def init_goal_tree(self, y1):
        """
        Goal tree, by default no open nodes, may be override by subclass.
        """
        dds = self.get_dds().inverse() # <-- note inverse()
        id_dds = self.id_dds
        if self.bidirectional:
            max_depth = self.max_depth
            max_iterations = self.max_iterations
        else:
            max_depth = 0 # do not expand anything
            max_iterations = 0 # do not expand anything
        plan_reducer = self.get_plan_reducer()
        dts = DiffeoTreeSearchImage(image=y1, id_dds=id_dds,
                        dds=dds, plan_reducer=plan_reducer,
                        max_depth=max_depth, max_iterations=max_iterations,
                        metric_collapse=self.metric_collapse,
                        metric_collapse_threshold=self.metric_collapse_threshold)
        return dts
    
    def plan_report(self, report, tc):
        """
            We pass the testcase structure, so we can use the ground
            truth (if available) to make a nicer visualization.
        """
        f = report.figure(cols=3)
        plan2length = lambda x: len(x)
        cmap_start = get_cmap('bones') 
        cmap_goal = get_cmap('jet')
        
        def distance_to(tree, y, node):
            image = tree.plan2image(node)
            return self.metric_goal.distance(image, y)
             
        from functools import partial
        start_dist_y0 = partial(distance_to, self.start_tree, self.y0)
        goal_dist_y0 = partial(distance_to, self.goal_tree, self.y0)
        start_dist_y1 = partial(distance_to, self.start_tree, self.y1)
        goal_dist_y1 = partial(distance_to, self.goal_tree, self.y1)
        
        with f.plot('start_tree') as pylab:
            self.start_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=plan2length, cmap=cmap_start,
                show_plan=tc.true_plan)
            pylab.title('plan length')
            turn_all_axes_off(pylab)
            pylab.colorbar()
            
        with f.plot('goal_tree') as pylab:
            self.goal_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=plan2length, cmap=cmap_goal)
            pylab.title('plan length')
            turn_all_axes_off(pylab)
            pylab.colorbar()
            
        true_plan = tc.true_plan
        ss = guess_state_space(self.id_dds, self._dds)
        u0 = self.get_dds().indices_to_commands(true_plan)
        origin = ss.state_from_commands(u0)

        with f.plot('joint') as pylab:
            self.start_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=plan2length, cmap=cmap_start)
            self.goal_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=plan2length, cmap=cmap_goal, origin=origin)
            turn_all_axes_off(pylab)
            pylab.colorbar()
            
        with f.plot('start_tree_y0') as pylab:
            self.start_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=start_dist_y0, cmap=cmap_start)
            pylab.title('distance to $y_0$')
            turn_all_axes_off(pylab)
            pylab.colorbar()
            
        with f.plot('goal_tree_y0') as pylab:
            self.goal_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=goal_dist_y0, cmap=cmap_goal, origin=origin)
            pylab.title('distance to y_0')
            turn_all_axes_off(pylab)
            pylab.colorbar()
            
        with f.plot('joint_y0') as pylab:
            self.start_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=start_dist_y0, cmap=cmap_start)
            self.goal_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=goal_dist_y0, cmap=cmap_goal, origin=origin)
            turn_all_axes_off(pylab)
            pylab.colorbar()
        
        with f.plot('start_tree_y1') as pylab:
            self.start_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=start_dist_y1, cmap=cmap_start)
            pylab.title('distance to $y_1$')
            turn_all_axes_off(pylab)
            pylab.colorbar()
            
        with f.plot('goal_tree_y1') as pylab:
            self.goal_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=goal_dist_y1, cmap=cmap_goal)
            pylab.title('distance to y_1')
            turn_all_axes_off(pylab)
            pylab.colorbar()
            
        with f.plot('joint_y1') as pylab:
            self.start_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=start_dist_y1, cmap=cmap_start)
            self.goal_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=goal_dist_y1, cmap=cmap_goal, origin=origin)
            turn_all_axes_off(pylab)
            pylab.colorbar()
        
