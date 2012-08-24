from . import DiffeoPlanningAlgo, PlanningResult, contract
from .. import UncertainImage
from diffeoplan.configuration import get_current_config
from diffeoplan.library.algo import Connector, DiffeoTreeSearchImage
from diffeoplan.library.analysis import PlanReducer
from diffeoplan.library.discdds import guess_state_space
from diffeoplan.library.discdds.diffeo_system import DiffeoSystem

 
class GenericGraphPlanner(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 
        
        Pieces:
        - metric
        - expand_start_tree
        - expand_goal_tree
        

    """
    
    def __init__(self,
                 metric_goal, metric_goal_threshold,
                 metric_collapse, metric_collapse_threshold,
                 max_depth=1000, max_iterations=1000):
        '''
        :param threshold:
        :param metric: ID of metric
        :param max_iterations:
        :param max_depth:
        '''
        DiffeoPlanningAlgo.__init__(self)
        
        config = get_current_config()
        self.metric_goal = config.distances.instance(metric_goal)        
        self.metric_goal_threshold = metric_goal_threshold
        self.metric_collapse = config.distances.instance(metric_collapse)        
        self.metric_collapse_threshold = metric_collapse_threshold
        self.max_iterations = max_iterations
        self.max_depth = max_depth

    @contract(dds=DiffeoSystem)
    def init(self, id_dds, dds):
        """ Might be redefined to add precomputation. """
        DiffeoPlanningAlgo.init(self, id_dds, dds)
    
    
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1):
        self.start_tree = self.init_start_tree(y0)
        self.goal_tree = self.init_goal_tree(y1)
        self.connector = self.init_connector(self.start_tree, self.goal_tree)
        
        self.log_planning_init()

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
                # let's choose one
                connection = connections[0]
                p1, p2 = connection
                plan = p1 + tuple(reversed(p2))
                return PlanningResult(True, plan, 'Found', extra=self.make_extra())        
        self.log_planning_failed()
        
        return PlanningResult(False, None, 'not found', extra=self.make_extra())
    
    def log_start_tree_expanded(self, added):
        self.info('Adding %s' % added)
        pass
    
    def log_goal_tree_expanded(self, added):
        pass
    
    def log_connections_found(self, connections):
        pass
            
    def log_planning_init(self):
        pass
    
    def log_planning_failed(self):
        pass
    
    def make_extra(self):
        """ Extra information to return about the search """
        extra = DiffeoPlanningAlgo.make_extra(self)
        extra['start_tree'] = self.start_tree
        extra['goal_tree'] = self.goal_tree
        extra['connector'] = self.connector
        return extra

    def init_connector(self, start_tree, end_tree):
#        G1 = start_tree.G
#        G2 = end_tree.G
#        G1_node2value = start_tree.plan2image
#        G2_node2value = end_tree.plan2image
        
        connector = Connector(#G1, G1_node2value, G2, G2_node2value,
                              start_tree, end_tree,
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
        max_depth = self.max_depth # do not expand anything
        max_iterations = self.max_iterations # do not expand anything
        plan_reducer = self.get_plan_reducer()
        dts = DiffeoTreeSearchImage(image=y0, id_dds=id_dds,
                        dds=dds, plan_reducer=plan_reducer,
                        max_depth=max_depth, max_iterations=max_iterations,
                        metric_collapse=self.metric_collapse,
                        metric_collapse_threshold=self.metric_collapse_threshold)
        plan0 = ()
        dts.init_search(plan0)
        return dts
    
    def init_goal_tree(self, y1):
        """
        Goal tree, by default no open nodes, may be override by subclass.
        """
        dds = self.get_dds().inverse() # <-- note inverse()
        id_dds = self.id_dds
        max_depth = 0 # do not expand anything
        max_iterations = 0 # do not expand anything
        plan_reducer = self.get_plan_reducer()
        dts = DiffeoTreeSearchImage(image=y1, id_dds=id_dds,
                        dds=dds, plan_reducer=plan_reducer,
                        max_depth=max_depth, max_iterations=max_iterations,
                        metric_collapse=self.metric_collapse,
                        metric_collapse_threshold=self.metric_collapse_threshold)
        plan0 = ()
        dts.init_search(plan0)
        return dts
    
    def plan_report(self, report, tc):
        """
            We pass the testcase structure, so we can use the ground
            truth (if available) to make a nicer visualization.
        """
        f = report.figure()
        with f.plot('start_tree') as pylab:
            cmap = None
            plan2color = lambda x: len(x)
            self.start_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=plan2color, cmap=cmap)
            
        with f.plot('goal_tree') as pylab:
            cmap = None
            plan2color = lambda x: len(x)
            self.goal_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=plan2color, cmap=cmap)

        true_plan = tc.true_plan
        with f.plot('joint') as pylab:
            #print('true plan: %s' % str(true_plan))
            ss = guess_state_space(self.id_dds, self._dds)
            u0 = self.get_dds().indices_to_commands(true_plan)
            origin = ss.state_from_commands(u0)
            #print('origin: %s' % str(origin))

            self.start_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=plan2color, cmap=cmap)
            self.goal_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=plan2color, cmap=cmap, origin=origin)

