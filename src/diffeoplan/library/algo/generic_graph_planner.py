from . import DiffeoPlanningAlgo, PlanningResult, contract
from .. import UncertainImage
from ..graph import Node
from diffeoplan.configuration import get_current_config
from diffeoplan.library.algo import Connector, DiffeoTreeSearchImage
from diffeoplan.library.analysis import PlanReducer
from diffeoplan.library.discdds.visualization.guess import guess_state_space

 
class GenericGraphPlanner(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 
        
        Pieces:
        - metric
        - expand_start_tree
        - expand_goal_tree
        

    """
    
    def __init__(self, metric, metric_threshold, max_depth=1000, max_iterations=1000):
        '''
        
        :param threshold:
        :param metric: ID of metric
        :param max_iterations:
        :param max_depth:
        '''
        DiffeoPlanningAlgo.__init__(self)
        
        config = get_current_config()
        self.metric = config.distances.instance(metric)        
        self.metric_threshold = metric_threshold
        self.max_iterations = max_iterations
        self.max_depth = max_depth

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
        self.info()
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
                              self.metric, self.metric_threshold)
        return connector
    
    def init_start_tree(self, y0):
        """
        Start tree, by default first node open, may be override by subclass.  
        """
        dds = self.get_dds()
        id_dds = self.id_dds
        nactions = len(dds.actions)
        plan_reducer = PlanReducer.dummy(labels=range(nactions))
        max_depth = self.max_depth # do not expand anything
        max_iterations = self.max_iterations # do not expand anything
    
        dts = DiffeoTreeSearchImage(image=y0, id_dds=id_dds,
                        dds=dds, plan_reducer=plan_reducer,
                        max_depth=max_depth, max_iterations=max_iterations)
        plan0 = ()
        dts.init_search(plan0)
        return dts
    
    def init_goal_tree(self, y1):
        """
        Goal tree, by default no open nodes, may be override by subclass.
        """
        dds = self.get_dds().inverse() # <-- note inverse()
        id_dds = self.id_dds
        nactions = len(dds.actions)
        plan_reducer = PlanReducer.dummy(labels=range(nactions))
        max_depth = 0 # do not expand anything
        max_iterations = 0 # do not expand anything
    
        dts = DiffeoTreeSearchImage(image=y1, id_dds=id_dds,
                        dds=dds, plan_reducer=plan_reducer,
                        max_depth=max_depth, max_iterations=max_iterations)
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
            print('true plan: %s' % str(true_plan))
            ss = guess_state_space(self.id_dds, self._dds)
            u0 = self.get_dds().indices_to_commands(true_plan)
            origin = ss.state_from_commands(u0)
            print('origin: %s' % str(origin))

            self.start_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=plan2color, cmap=cmap)
            self.goal_tree.plot_graph_using_guessed_statespace(
                pylab, plan2color=plan2color, cmap=cmap, origin=origin)


            
            
#    
#    def should_add_node(self, tree, node):
#        y = node.y
#        distances = tree.get_distances(y)
#        someone_too_close = np.any(distances < self.thresh) 
#        return not someone_too_close
#    
#    @staticmethod
#    def is_unique(path, tree):
#        for p in tree.blocked:
#            if list(p) == list(path):
#                return False
#        return True
#    
#
#    def expand_start_tree(self, G):
#        """ Can return a Node or None if there is nothing 
#            else to expand (according to some internal condition)
#        """
#        raise ValueError('not implemented')
#        
#    def expand_goal_tree(self, G): #@UnusedVariable
#        """ Can return a Node or None if there is nothing 
#            else to expand (according to some internal condition)
#        """
#        return None    

    
class GraphSearchQueue(GenericGraphPlanner):
    """  
    GenericGraphPlanner using a queue of open nodes for expansion.
    """
    
    def __init__(self, thresh, metric, max_ittr, nsteps):
        GenericGraphPlanner.__init__(self, thresh, metric, max_ittr)
#        self.start_open = None
        self.nsteps = nsteps
    
    def get_new_node(self, tree):
        assert(tree.open_nodes != None)
        if len(tree.open_nodes) == 0:
            return None
        
        toexpand = self.get_next_index(tree, tree.open_nodes) 
        
        dds = self.get_dds()
        all_actions = range(len(dds.actions))
        available = tree.actions_available_for_node(toexpand, all_actions)
        assert len(available) >= 1
        
        next_cmd = self.get_next_cmd(tree, toexpand, available)
                 
        next_node = get_next_node(tree, toexpand, next_cmd,
                                    dds=self.get_dds())
        
        # put the new one
        next_index = len(tree.nodes) # xxx don't like
        
        if len(next_node.path) < self.nsteps:
            tree.open_nodes.append(next_index)
        
        if len(available) == 1:
            # this node is now closed
            tree.open_nodes.remove(toexpand)
        
        assert next_node.__class__ == Node
        return next_node

    def get_next_cmd(self, tree, node_index, available): #@UnusedVariable
        return available[0]

    def expand_start_tree(self, start_tree):
        return self.get_new_node(start_tree)
    
    def expand_goal_tree(self, goal_tree):
        return self.get_new_node(goal_tree)

    def get_next_index(self, tree, open_nodes):
        raise ValueError('to implement')
#    
#def get_next_node(tree, parent_index, cmd, dds): # todo: move
#    parent = tree.nodes[parent_index]
#    path = list(parent.path) + [cmd]
#    next_action = dds.actions[cmd]
#    y_new = next_action.predict(parent.y)
#    return Node(y=y_new,
#                path=path,
#                parent=parent_index,
#                children=[])
#    
    
        
