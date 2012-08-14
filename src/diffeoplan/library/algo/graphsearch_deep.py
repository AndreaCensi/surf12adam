from ..graph import Node
import copy
from diffeoplan.library.algo.graphsearch import GraphSearch

class GraphSearchDeep(GraphSearch):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 

    """
    def get_next_node(self, tree): 
        # Search for node with unevaluated command
        for i in range(len(tree.nodes) - 1, -1, -1): 
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
