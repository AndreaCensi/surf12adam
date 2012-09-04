import numpy as np
from contracts import contract
from diffeoplan.library.graph.node import Node
import pdb

class Graph():
    def __init__(self, root_node, metric, match_thresh):
        assert root_node.__class__ == Node
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
        assert node.__class__ == Node
        # Add child-parent index references
#        pdb.set_trace()
        self.nodes[node.parent].child_nodes.append(len(self.nodes))
        self.nodes.append(node)
             
#    @contract(i='int', all_actions='list(int)', returns='list(int)')
#    def actions_available_for_node(self, i, all_actions):
#        """ Returns a list of actions available for the node. """
#        # get the node
#        node = self.nodes[i]
#        if node.__class__ != Node:
#            pdb.set_trace()
#        # get the children nodes
#        children = [self.nodes[j] for j in node.child_nodes]
#        
#        # get the closed nodes
#        closed = [self.nodes[j] for j in node.closed_cmd]
#        # get the actions
##        actions = [child.path[-1] for child in children]
#        child_actions = [child.parent_cmd for child in children]
#        closed_actions = [close.parent_cmd for close in closed]
#        # Find the elements of all_actions not in actions
#        available = list(set(all_actions) - set(child_actions) - set(closed_actions))
#        return available
    
    def get_nodes_distance_slow(self, i, j):
        ni = self.nodes[i]
        nj = self.nodes[j]
        d = self.metric.distance(ni.y, nj.y)
        return d
    
    @contract(parent='int', cmd='int', y='array', returns='int')
    def add_node2(self, parent, cmd, y):
        """ Adds a node and return its index. """
        parent_node = self.nodes[parent]
        path = list(parent_node.path) + [cmd]
        
        node = Node(y=y,
                    path=path,
                    parent=parent,
                    parent_cmd=cmd,
                    children=[])
        self.nodes.append(node)
        return len(self.nodes) - 1
        
    
    
    def get_nodes_distance(self, i, j):
        if not (i, j) in self.cached_distance:
            d = self.get_nodes_distance_slow(i, j)
            self.cached_distance[(i, j)] = d
            self.cached_distance[(j, i)] = d
        return self.cached_distance[(i, j)] 

    def get_closest(self, tree):
        return None
