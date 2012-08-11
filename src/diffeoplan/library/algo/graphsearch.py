from . import DiffeoPlanningAlgo, PlanningResult, contract
from .. import UncertainImage
from ..graph import Node, TreeConnector
from diffeoplan.configuration import get_current_config
from diffeoplan.library.graph.graph import Graph

class GraphSearch(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 

    """
    
    @contract(nsteps='int,>=1')
    def __init__(self, nsteps, thresh, metric, directions=1):
        '''
        :param nsteps: Number of steps in the random guess.
        '''
        config = get_current_config()
        self.metric = config.distances.instance(metric)
        self.thresh = thresh
        self.nsteps = nsteps
        self.directions = directions
        
        self.comp_ind = 0 # Dont look for nodes of lower inde than this
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): #@UnusedVariable
        print('Engering graphsearch plan()')
        dds = self.get_dds()
        
        ncmd = len(dds.actions)
        
        start_node = Node(y0, [])
        start_node.command_stack = range(ncmd)
        start_node.child_nodes = []
        start_tree = Graph(start_node, self.metric, self.thresh)
        
        goal_node = Node(y1, [])
        goal_node.command_stack = range(ncmd)
        goal_tree = Graph(goal_node, self.metric, self.thresh)
        
        connector = TreeConnector(start_tree, goal_tree, self.tresh)
        
        while True:
            new_start_node = self.get_new_node(start_tree)
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
    
    def get_new_node(self):
        return None
