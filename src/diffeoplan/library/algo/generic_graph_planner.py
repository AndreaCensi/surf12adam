from . import DiffeoPlanningAlgo, PlanningResult, contract
from .. import UncertainImage
from ..graph import Node, TreeConnector
from diffeoplan.configuration import get_current_config
from diffeoplan.library.graph.graph import Graph

import numpy as np
import pdb
 
 
class GenericGraphPlanner(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 
        
        Pieces:
        - metric
        - expand_start_tree
        - expand_goal_tree
        

    """
    def __init__(self, thresh, metric, max_ittr=1000):
        DiffeoPlanningAlgo.__init__(self)
        self.thresh = thresh # are two states the same?
        self.max_ittr = max_ittr # todo: move in subclasses
        config = get_current_config()
        self.metric = config.distances.instance(metric)
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1):
        start_tree = self.init_start_tree(y0, self.metric, self.thresh)
        goal_tree = self.init_goal_tree(y1, self.metric, self.thresh)
        connector = TreeConnector(start_tree, goal_tree, self.thresh)

        def make_extra():
            """ Extra information to return about the search """
            extra = self.make_extra()
            extra['start_tree'] = start_tree
            extra['goal_tree'] = goal_tree
            extra['connector'] = connector
            return extra

        self.info('GraphSearch starting')
        while True:
            start_expansion_nodes = self.expand_start_tree(start_tree)
            
#            if self.should_add_node(start_tree, new_start_node):
#                self.info('Chosen new_start_node = %s' % new_start_node)
#                start_tree.add_node(new_start_node)
            
            goal_expansion_nodes = self.expand_goal_tree(goal_tree)
            
            if not start_expansion_nodes and not goal_expansion_nodes:
                # We don't expand anymore, so we failed 
                self.info('Breaking and failing.')
                break

            
#            if not start_expansion_nodes:
                # We don't expand anymore, so we failed 
                
#            new_goal_node = self.expand_goal_tree(goal_tree)
            
#            if new_goal_node is not None:            
#                if self.should_add_node(goal_tree, new_goal_node):
#                    self.info('Chosen new_goal_node = %s.' % new_goal_node)
#                    goal_tree.add_node(new_goal_node)

            #if len(new_goal_node.path) <= self.nsteps:
#            pdb.set_trace()

            nplans = connector.connect_update()
            if nplans > 0:
                plan = connector.get_connection()
                self.info('Returning plan: ' + str(plan))
                return PlanningResult(True, plan, 'Graph Search Plan',
                                      extra=make_extra())
        
        self.info('Planning failed.')
        return PlanningResult(False, None, 'GraphSearch failed',
                              extra=make_extra())
        
    def init_start_tree(self, y0, metric, thresh):
        """
        Start tree, by default first node open, may be override by subclass.  
        """
        # todo: use add_node2
        start_node = Node(y=y0, path=[], parent=[], parent_cmd=[], children=[])
        start_tree = Graph(start_node, metric, thresh)
        start_tree.open_nodes = [0]
        return start_tree
    
    def init_goal_tree(self, y1, metric, thresh):
        """
        Goal tree, by default no open nodes, may be override by subclass.
        """
        # todo: use add_node2
        goal_node = Node(y=y1, path=[], parent=[], parent_cmd=[], children=[])
        goal_tree = Graph(goal_node, metric, thresh)
        return goal_tree
    
#    @contract(y1=Node)
    def accept_state(self, tree, y):
        """ Should I accept this new state? """
        distances = tree.get_distances(y)
        someone_too_close = np.any(distances < self.thresh) 
        return not someone_too_close 
    
    def expand_start_tree(self, tree):
        """ Returns a list of indices possibly empty. 
        """
        raise ValueError('not implemented')
        
    def expand_goal_tree(self, tree): #@UnusedVariable
        """ Returns a list of indices possibly empty. """ 
        return None    
    
    
class GraphSearchQueue(GenericGraphPlanner):
    """  
    GenericGraphPlanner using a queue of open nodes for expansion.
    """
    
    def __init__(self, thresh, metric, max_ittr, nsteps):
        GenericGraphPlanner.__init__(self, thresh, metric, max_ittr)
        self.nsteps = nsteps
    
    @contract(returns='list(int)')
    def expand_new_node(self, tree, dds): 
        assert(tree.open_nodes != None)
        if len(tree.open_nodes) == 0:
            return []
        
        toexpand = self.get_next_index(tree, tree.open_nodes) 
    
        available = self.actions_from_node(toexpand)            
        children = []
        for next_cmd in available:
            y1 = self.get_next_state(toexpand.y, next_cmd)
            if self.accept_state(tree, y1):
                continue            
            index = tree.add_node2(parent=toexpand, cmd=next_cmd, y=y1)
            children.append(index)
            tree.open_nodes.append(index)
        tree.open_nodes.remove(toexpand)
        return children
    
    def actions_from_node(self, node):
        if len(node.path) >= self.nsteps:
            return [] 
        nactions = len(self.get_dds().actions)
        return range(nactions)
 

    def expand_start_tree(self, start_tree):
        return self.get_new_node(start_tree, self.dds)
    
    def expand_goal_tree(self, goal_tree):
        return self.get_new_node(goal_tree, self.dds.inverse()) 

    def get_next_index(self, tree, open_nodes):
        raise ValueError('to implement')
    
    def get_next_state(self, y0, cmd, dds):
        next_action = dds.actions[cmd]
        y_new = next_action.predict(y0)
        return y_new
#    
#def get_next_node(tree, parent_index, cmd, dds): # todo: move
#    parent = tree.nodes[parent_index]
#    path = list(parent.path) + [cmd]
#    next_action = dds.actions[cmd]
#    y_new = next_action.predict(parent.y)
#    return Node(y=y_new,
#                path=path,
#                parent=parent_index,
#                children=[],
#                parent_cmd=cmd)
    


        
