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
        self.max_ittr = max_ittr
        #self.comp_ind = 0 # Dont look for nodes of lower inde than this
        config = get_current_config()
        self.metric = config.distances.instance(metric)
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1):
        
#        pdb.set_trace()
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
            new_start_node = self.expand_start_tree(start_tree)
            self.info('Chosen new_start_node = %s' % new_start_node)
            
            if new_start_node is None:
                # We don't expand anymore, so we failed 
                self.info('Breaking and failing.')
                break
            
            if self.should_add_node(start_tree, new_start_node):
                start_tree.add_node(new_start_node)
            
            new_goal_node = self.expand_goal_tree(goal_tree)
            if new_goal_node is not None:
                self.info('Goal node is %s.' % new_goal_node)
            
                if self.should_add_node(goal_tree, new_goal_node):
                    goal_tree.add_node(new_goal_node)

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
        start_node = Node(y=y0, path=[], parent=[], children=[])
        start_tree = Graph(start_node, metric, thresh)
        start_tree.open_nodes = [0]
        return start_tree
    
    def init_goal_tree(self, y1, metric, thresh):
        """
        Goal tree, by default no open nodes, may be override by subclass.
        """
        goal_node = Node(y=y1, path=[], parent=[], children=[])
        goal_tree = Graph(goal_node, metric, thresh)
        return goal_tree
    
    def should_add_node(self, tree, node):
        # TODO later if needed: keep track of alternative paths
        assert node.__class__ == Node 
        y = node.y
        distances = tree.get_distances(y)
        someone_too_close = np.any(distances < self.thresh) 
        return not someone_too_close
    
    @staticmethod
    def is_unique(path, tree):
        for p in tree.blocked:
            if list(p) == list(path):
                return False
        return True
    
    def expand_start_tree(self, tree):
        """ Can return a Node or None if there is nothing 
            else to expand (according to some internal condition)
        """
        raise ValueError('not implemented')
        
    def expand_goal_tree(self, tree): #@UnusedVariable
        """ Can return a Node or None if there is nothing 
            else to expand (according to some internal condition)
        """
        return None    
    
    
class GraphSearchQueue(GenericGraphPlanner):
    """  
    GenericGraphPlanner using a queue of open nodes for expansion.
    """
    
    def __init__(self, thresh, metric, max_ittr, nsteps):
        GenericGraphPlanner.__init__(self, thresh, metric, max_ittr)
#        self.start_open = None
        self.nsteps = nsteps
    
    def get_new_node(self, tree):
#        
#        # First time: I put the initial one
#        if tree.open_nodes is None:
#            tree.open_nodes = [0]
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
    
def get_next_node(tree, parent_index, cmd, dds): # todo: move
    parent = tree.nodes[parent_index]
    path = list(parent.path) + [cmd]
    next_action = dds.actions[cmd]
    y_new = next_action.predict(parent.y)
    return Node(y=y_new,
                path=path,
                parent=parent_index,
                children=[])
    
    
        
