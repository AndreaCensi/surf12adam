from . import DiffeoPlanningAlgo, PlanningResult, contract
from .. import UncertainImage
from ..graph import Node, TreeConnector
from diffeoplan.configuration import get_current_config
from diffeoplan.library.graph.graph import Graph

import numpy as np
 
 
class GenericGraphPlanner(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 
        
        Pieces:
        - metric
        - expand_start_tree
        - expand_goal_tree
        

    """
    
    def __init__(self,  thresh, metric, max_ittr=1000):
        DiffeoPlanningAlgo.__init__(self)
        self.thresh = thresh # are two states the same?
        self.max_ittr = max_ittr
        #self.comp_ind = 0 # Dont look for nodes of lower inde than this
        config = get_current_config()
        self.metric = config.distances.instance(metric)
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1):
        
        start_node = Node(y=y0, path=[])
        start_tree = Graph(start_node, self.metric, self.thresh)
        
        goal_node = Node(y=y1, path=[])
        goal_tree = Graph(goal_node, self.metric, self.thresh)
        
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
            
            if self.should_add_node(start_tree, new_start_node.y):
                start_tree.add_node(new_start_node)
            
            new_goal_node = self.expand_goal_tree(goal_tree)
            if new_goal_node is not None:
                self.info('Goal node is %s.' % new_goal_node)
            
                if self.should_add_node(goal_tree, new_goal_node.y):
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
        
    
    def should_add_node(self, tree, y):
        # TODO later if needed: keep track of alternative paths
        distances = tree.get_distances(y)
        someone_too_close = np.any(distances < self.thresh) 
        return not someone_too_close
    
    @staticmethod
    def is_unique(path, tree):
        for p in tree.blocked:
#            pdb.set_trace()
            if list(p) == list(path):
                return False
        return True
    
    
    def expand_start_tree(self, tree):
        """ Can return a Node or None if there is nothing 
            else to expand (according to some internal condition)
        """
        raise ValueError('not implemented')
        
    def expand_goal_tree(self, tree): 
        """ Can return a Node or None if there is nothing 
            else to expand (according to some internal condition)
        """
        return None    
        #raise ValueError('not implemented')
    

    
    
        
