import numpy as np
from contracts import contract

class Graph():
    def __init__(self, root_node, metric, match_thresh):
        self.nodes = [root_node]
        self.distances = np.zeros((1, 1))
        self.metric = metric
        self.match_thresh = match_thresh
        self.blocked = []
        self.child_nodes = []
        
        self.open_nodes = []
        self.cached_distance = {} # (i,j) -> D_ij
        
        
    def get_distances(self, y):
        """ Returns a list of distances from this image to all nodes """
        return [self.metric.distance(y, n.y) for n in self.nodes]
        # TODO: cache
        
    def add_node(self, node):
        # Add child-parent index references
        self.nodes[node.parent].child_nodes.append(len(self.nodes))
        self.nodes.append(node)
        
#        
             
    @contract(i='int', all_actions='list(int)', returns='list(int)')
    def actions_available_for_node(self, i, all_actions):
        """ Returns a list of actions available for the node. """
        # get the node
        node = self.nodes[i]
        # get the children nodes
        children = [self.nodes[j] for j in node.child_nodes]
        # get the actions
        actions = [child.path[-1] for child in children]
        # Find the elements of all_actions not in actions
        available = list(set(all_actions) - set(actions))
        return available
    
    
    def get_nodes_distance_slow(self, i, j):
        ni = self.nodes[i]
        nj = self.nodes[j]
        d = self.metric.distance(ni.y, nj.y)
        return d
    
    def get_nodes_distance(self, i, j):
        if not (i,j) in self.cached_distance:
            d = self.get_nodes_distance_slow(i,j)
            self.cached_distance[(i,j)] =  d
            self.cached_distance[(j,i)] =  d
        return self.cached_distance[(i,j)] 

    def get_closest(self, tree):
        return None