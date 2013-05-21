from . import (DiffeoPlanningAlgo, PlanningResult, contract, Connector,
    DiffeoTreeSearchImage)
from diffeoplan import get_current_config
from diffeoplan.library import (DiffeoSystem, guess_state_space, plan_friendly,
    DiffeoAction, PlanReducer, UncertainImage)
from matplotlib.cm import get_cmap
from reprep import Report
from reprep.plot_utils import turn_all_axes_off
import time
from diffeo2dds.model.plan_utils import plan_steps

__all__ = ['GenericGraphPlanner']
 
class GenericGraphPlanner(DiffeoPlanningAlgo):
    
    @contract(bidirectional='bool',
              metric_collapse_threshold='>=0')
    def __init__(self, bidirectional,
                 metric_goal,
                 metric_collapse, metric_collapse_threshold,
                 max_depth=10000, max_iterations=10000, max_time=120,
                 max_memory_MB=25,):
        '''
        
        :param bidirectional:
        :param metric_goal:
        :param metric_collapse:
        :param metric_collapse_threshold:
        :param max_depth:
        :param max_iterations:
        :param max_time:
        :param max_memory_MB:
        '''
        super(GenericGraphPlanner, self).__init__()
        self.bidirectional = bidirectional
        config = get_current_config()
        self.metric_goal = config.distances.instance(metric_goal)        
        self.metric_collapse = config.distances.instance(metric_collapse)        
        self.metric_collapse_threshold = metric_collapse_threshold
        self.max_iterations = max_iterations
        self.max_depth = max_depth
        self.max_time = max_time
        self.max_memory_MB = max_memory_MB
        
    def __strparams__(self):
        return ("%s;g:%s;c:%s<=%s" % 
                ('1' if self.bidirectional else '2',
                 self.metric_goal,
                 self.metric_collapse,
                 self.metric_collapse_threshold))

    def __str__(self):
        return 'GenericGraphPlanner(%s)' % self.__strparams__()
        
    @contract(dds=DiffeoSystem)
    def init(self, id_dds, dds):
        """ Might be redefined to add precomputation. """
        super(GenericGraphPlanner, self).init(id_dds, dds)
        
    @contract(report=Report)
    def init_report(self, report):
        """ Creates a report for the initialization phase. """
        super(GenericGraphPlanner, self).init_report(report)
        diffeosystem_display_products(self.get_dds(),
                                      report.section('actions'), 5)

    def plan_init(self, y0, y1):
        self.clock_start = time.clock()
        self.y0 = y0
        self.y1 = y1
        self.start_tree = self.init_start_tree(y0)
        self.goal_tree = self.init_goal_tree(y1)
        
        self.start_tree.set_min_visibility(self.min_visibility)
        self.goal_tree.set_min_visibility(self.min_visibility)
                
        self.start_tree.memoize_cache.set_max_memory_MB(self.max_memory_MB)
        self.goal_tree.memoize_cache.set_max_memory_MB(self.max_memory_MB)
        
        plan0 = ()
        self.start_tree.init_search(plan0)
        self.goal_tree.init_search(plan0)

        self.connector = self.init_connector(self.start_tree, self.goal_tree)

        # Initialize logging system
        self.log_add_child('start_tree', self.start_tree)
        self.log_add_child('goal_tree', self.goal_tree)
        self.log_add_child('connector', self.connector)
        self.log_planning_init()

    @contract(y0=UncertainImage, y1=UncertainImage,
              precision='>=0', min_visibility='>=0',
              returns=PlanningResult)
    def plan(self, y0, y1, precision, min_visibility):
        self.log_plan(y0, y1, precision, min_visibility)
        
        self.metric_goal_threshold = precision
        self.min_visibility = min_visibility
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
                d = connection.distance_prediction
                p1 = connection.n1
                p2 = connection.n2
                                
                # this can be redundant
                plan_red = connection.plan
                # let's simplify it
                plan = self.get_plan_reducer().get_canonical(plan_red)
                self.log_plan_found(d, p1, p2, plan_red, plan)
                result = PlanningResult(True, plan, 'Found',
                                        extra=self.make_extra())
                self.log_final_result(result)
                return result
                
            # Check if too much time passed
            passed = time.clock() - self.clock_start
            if passed >= self.max_time:
                self.log_time_expired(passed)
                result = PlanningResult(False, None, 'Time expired',
                                      extra=self.make_extra())
                self.log_final_result(result)
                return result
                    
        self.log_planning_failed()
        result = PlanningResult(False, None, 'not found',
                                       extra=self.make_extra())
        self.log_final_result(result)
        return result
        
    def log_plan(self, y0, y1, precision, min_visibility):  # @UnusedVariable
        self.info('Planning query; precision: %s  min_visibility: %s' 
                  % (precision, min_visibility))
        
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
            self.info(' - %s' % c.__str__()) 

    def log_planning_init(self):
        self.info('Planning started')
        
    def log_planning_failed(self):
        self.info('Planning failed')
        
    def log_plan_found(self, d, p1, p2, plan_red, plan):  # @UnusedVariable
        self.info('Connection between %s and %s of distance %s' % 
                  (self.start_tree.node_friendly(p1),
                   self.goal_tree.node_friendly(p2), d))
        self.info(' concat: %s' % plan_friendly(plan_red))
        self.info('reduced: %s' % plan_friendly(plan))

    def init_connector(self, start_tree, end_tree):
        connector = Connector(start_tree, end_tree,
                              self.metric_goal, self.metric_goal_threshold,
                              min_visibility=self.min_visibility)
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
        dds = self.get_dds().inverse()  # <-- note inverse()
        id_dds = self.id_dds
        if self.bidirectional:
            max_depth = self.max_depth
            max_iterations = self.max_iterations
        else:
            max_depth = 0  # do not expand anything
            max_iterations = 0  # do not expand anything
        plan_reducer = self.get_plan_reducer()
        dts = DiffeoTreeSearchImage(image=y1, id_dds=id_dds,
                        dds=dds, plan_reducer=plan_reducer,
                        max_depth=max_depth, max_iterations=max_iterations,
                        metric_collapse=self.metric_collapse,
                        metric_collapse_threshold=self.metric_collapse_threshold)
        return dts
    
    def plan_report(self, report, result, tc):
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
        
        if result.success:
            # Plot the distance as a function of step
            steps = plan_steps(result.plan)  # (), (0,) (0, 1), etc.
            distance_to_y0 = map(start_dist_y0, steps) 
            distance_to_y1 = map(start_dist_y1, steps)
            with f.plot('distances_along_path') as pylab:
                lenghts = map(len, steps) 
                pylab.plot(lenghts, distance_to_y0, 'r-', label='to $y_0$')
                pylab.plot(lenghts, distance_to_y1, 'b-', label='to $y_1$')
                pylab.xlabel('subplan length')
                pylab.legend()
                
        
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
        
    def log_time_expired(self, passed):
        self.info('Stopping because time expired: %s > %s' % 
                  (passed, self.max_time))
    
    def log_final_result(self, result):  # @UnusedVariable
        """ Called last, before returning """
        self.info('Summary of memory for start_tree:\n %s' 
                  % self.start_tree.memoize_cache.summary())
        self.info('Summary of memory for goal_tree:\n %s' 
                  % self.goal_tree.memoize_cache.summary())
        self.info('Summary of memory for connecor:\n %s' 
                  % self.connector.memoize_cache.summary())
        
def diffeosystem_display_products(dds, report, nsteps):
    # XXX: make separate
    for i, a in enumerate(dds.actions):
        f = report.figure('cmd%s' % i, cols=nsteps)
        A = a
        for k in range(nsteps):
            A = DiffeoAction.compose(A, a)
            rgb = A.get_diffeo2d_forward().get_rgb_info()
            f.data_rgb('%s_%s' % (i, k), rgb)

