from ..graph import Node
import copy
import numpy as np
from diffeoplan.library.algo.generic_graph_planner import GraphSearchQueue

class InformedSearch(GraphSearchQueue):
    """  TODO: add in docs

    """
    def get_next_node(self, tree): 
        return len(tree.nodes) - 1 
#        try:
##        if self.active_node == None:
##            self.active_node = 0
##            
#            if len(tree.nodes[self.active_node].command_stack) == len(tree.nodes[self.active_node].child_nodes):
#                self.active_node = tree.get_closest_node()
#        except AttributeError:
#            self.active_node = 0
#            
#        return self.active_node
    
    def get_next_cmd(self, node):
        next_cmd = node.command_stack[len(node.child_nodes)]
        return next_cmd
    
    def get_next_index(self, tree, open_nodes):
#        tree.get_nodes_distance(i,j)
        pass
        
    def expand_start_tree(self, tree):       
        dds = self.get_dds()
        ncmd = len(dds.actions)
        
        next_node = self.get_next_node(tree)
        node = tree.nodes[next_node]
        
        min_dist = np.zeros(ncmd)
        node_new = [0] * ncmd 
        for i in range(ncmd):
            
            y_new = dds.actions[i].predict(node.y)
            path = copy.deepcopy(node.path) 
            path.append(i)
            node_new[i] = Node(y_new, path)
            ###
            min_dist[i] = np.min(tree.connector.T2.get_distances(node_new[i]))
            ### has to fix for dual direction search
        
        best_ind = np.nonzero(min_dist == np.min(min_dist))[0] 
        node_new = node_new[best_ind]
        
        # Set data for node
        node_new.parent = next_node
        node_new.command_stack = range(ncmd)
        node_new.child_nodes = []
                
        return node_new
        
