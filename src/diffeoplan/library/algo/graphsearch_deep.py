from . import DiffeoPlanningAlgo, PlanningResult, contract, np
from .. import UncertainImage
from ..graph import Node, Graph, TreeConnector
from diffeoplan.configuration import get_current_config
import copy
import pdb
from diffeoplan.library.algo.graphsearch import GraphSearch

class GraphSearchDeep(GraphSearch):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 

    """
    
#    @contract(nsteps='int,>=1')
#    def __init__(self, nsteps, tresh, metric, directions=1):
##        pdb.set_trace()        
#        '''
#        :param nsteps: Number of steps in the random guess.
#        '''
#        config = get_current_config()
#        self.metric = config.distances.instance(metric)
#        
#        self.tresh = tresh
#        self.nsteps = nsteps
#        self.directions = directions
        
        
#    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
#    def plan(self, y0, y1): 
#        print('Engering graphsearch plan()')
#        dds = self.get_dds()
#        
#        ncmd = len(dds.actions)
#        
#        start_node = Node(y0, [])
#        start_node.command_stack = range(ncmd)
#        start_node.child_nodes = []
#        start_tree = Graph(start_node, self.metric)
#        
#        goal_node = Node(y1, [])
#        goal_node.command_stack = range(ncmd)
#        goal_tree = Graph(goal_node, self.metric)
#        
#        connector = TreeConnector(start_tree, goal_tree, self.tresh)
#        
#        while True:
#            new_start_node = self.get_new_node(start_tree)
#            if len(new_start_node.path) <= self.nsteps:
#                start_tree.add_node(new_start_node)
#            else:
#                break
#            if self.directions == 2:
#                new_goal_node = self.get_new_node(goal_tree)
#                if len(new_goal_node.path) <= self.nsteps:
#                    goal_tree.add_node(new_goal_node)
#            
#            nplans = connector.connect_update()
#            if nplans > 0:
#                plan = connector.get_connection()
#                print('Returning plan: ' + str(plan))
#                return PlanningResult(True, plan, 'Graph Search Plan')
#        return PlanningResult(True, [0], 'Graph Search Plan')
        
    def get_next_node(self,tree):
        # Search for node with unevaluated command
        for i in range(len(tree.nodes)-1, -1, -1):
            if len(tree.nodes[i].command_stack) > len(tree.nodes[i].child_nodes):
                return i
        
        return None # Algorithm complete
    
    def get_next_cmd(self, node):
        next_cmd = node.command_stack[len(node.child_nodes)]
        return next_cmd
    
    def get_new_node(self, tree):
        dds = self.get_dds()
        ncmd = len(dds.actions)
        
        # Node index to expand
        next_node = self.get_next_node(tree)
        if next_node == None:
            return None
        node = tree.nodes[next_node]
        path = copy.deepcopy(node.path)
        next_cmd = self.get_next_cmd(node)
        next_action = dds.actions[next_cmd]
        y_new = next_action.predict(node.y)
        path.append(next_cmd)
        node_new = Node(y_new, path)
        
        # Set data for node
        node_new.parent = next_node
        if len(path) < self.nsteps:
            node_new.command_stack = range(ncmd)
        else:
            node_new.command_stack = []
        node_new.child_nodes = []
        
        return node_new