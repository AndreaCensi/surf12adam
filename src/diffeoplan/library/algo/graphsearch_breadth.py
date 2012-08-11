from ..graph import Node
import copy
from diffeoplan.library.algo.graphsearch import GraphSearch

class GraphSearchBreadth(GraphSearch):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 

    """
    def get_next_node(self,tree):
        # Search for node with unevaluated command
        for i in range(self.comp_ind, len(tree.nodes)):
            if len(tree.nodes[i].command_stack) > len(tree.nodes[i].child_nodes):
                self.comp_ind = i # don't look for nodes with lower index than this
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
        node_new.command_stack = range(ncmd)
        node_new.child_nodes = []
        
        # Add child index to parent
        node.child_nodes.append(next_node)
        
        return node_new
        