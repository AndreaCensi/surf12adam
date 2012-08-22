'''
Created on Aug 20, 2012

@author: adam
'''
import numpy as np
from ..generic_graph_planner import GraphSearchQueue
from diffeoplan.library.discdds.redundant_plan_tracker import RedundantPlanTracker
from diffeoplan.library.graph.node import Node

class StructuredGraphPlanner(GraphSearchQueue):
    '''
    Graph search queue with redundant plan tracking.
    '''
    
    def __init__(self, thresh, metric, max_ittr, nsteps,
                 tracker_thresh):
        GraphSearchQueue.__init__(self, thresh, metric, max_ittr, nsteps)
        self.tracker_thresh = tracker_thresh
        self.tracker = None
        
    def should_add_node(self, tree, node):
        '''
        tells if the node should be added to tree by checking if the 
        image is close to an existing image, and by checking if the 
        plan is redundant. 
        :param tree:
        :param node:
        '''
        assert node.__class__ == Node 
        # TODO later if needed: keep track of alternative paths
        y = node.y
        distances = tree.get_distances(y)
        someone_too_close = np.any(distances < self.thresh)
        
        # Check for tracker and create if not existing
        if self.tracker is None:
            tracker = RedundantPlanTracker(self.get_dds(), self.tracker_thresh)
        else:
            tracker = self.tracker
        
        plan = node.path
        
        if tracker.is_redundant(plan):
            return False
        else:
            return not someone_too_close
        
