from . import DiffeoPlanningAlgo, PlanningResult, contract, np
from .. import UncertainImage
from ..graph import Node, TreeConnector
from diffeoplan.configuration import get_current_config
from diffeoplan.library.graph.graph import Graph
import pdb
#import copy
#import pdb

class GraphSearch(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 

    """
    
    @contract(nsteps='int,>=1')
    def __init__(self, nsteps, tresh, metric, directions=1, match_thresh=0):
#        pdb.set_trace()        
        '''
        :param nsteps: Number of steps in the random guess.
        '''
        config = get_current_config()
        self.directions = directions
        self.match_thresh = match_thresh
        self.metric = config.distances.instance(metric)
        self.nsteps = nsteps
        self.tresh = tresh
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): #@UnusedVariable
        print('Engering graphsearch plan()')
        
        self.last_path = []
        
        start_node = Node(y0, [])
        start_tree = Graph(start_node, self.metric, self.match_thresh)
        
        goal_node = Node(y1, [])
        goal_tree = Graph(goal_node, self.metric, self.match_thresh)
        
        connector = TreeConnector(start_tree, goal_tree, self.tresh)
        
        while True:
            new_start_node = self.get_new_node(start_tree)
            if new_start_node <> None:
                if len(new_start_node.path) <= self.nsteps:
                    start_tree.add_node(new_start_node)
                else:
                    break
                if self.directions == 2:
                    new_goal_node = self.get_new_node(goal_tree)
                    if len(new_goal_node.path) <= self.nsteps:
                        goal_tree.add_node(new_goal_node)
                
                nplans = connector.connect_update()
                if nplans > 0:
                    plan = connector.get_connection()
                    print('Returning plan: ' + str(plan))
                    return PlanningResult(True, plan, 'Graph Search Plan')
        return PlanningResult(True, [0], 'Graph Search Plan')
     
    @staticmethod
    def is_unique(path, tree):
        for p in tree.blocked:
#            pdb.set_trace()
            if list(p)==list(path):
                return False
        return True
    
    def get_new_node(self, tree):
        dds = self.get_dds()
#        last_path = copy.deepcopy(np.array(tree.nodes[-1].path))
        last_path = self.last_path
        
        
        ncommand = len(dds.actions)
        if np.sum(last_path == ncommand - 1) == len(last_path):
            new_path = np.zeros(len(last_path) + 1).astype(np.int)
        else:
            new_path = np.zeros(len(last_path)).astype(np.int)
            add_next = 1
            for i in range(1, len(last_path) + 1):
                new_path[-i] = (last_path[-i] + add_next) % ncommand
                add_next = (last_path[-i] + add_next) / ncommand
#        pdb.set_trace()
             
        if self.is_unique(last_path, tree):
            new_node = Node(dds.predict(tree.nodes[0].y, new_path), new_path)
        else:
            new_node = None
            
        self.last_path = new_path
        return new_node
