from diffeoplan.library.graph.tree import Tree
from diffeoplan.library.images.distance.distance_L2 import Distance_L2
import numpy as np
import pdb

class Graph(Tree):
    def __init__(self, root_node, metric=Distance_L2(), match_thresh=0.0):
        self.nodes = [root_node]
        self.distances = np.zeros((1, 1))
        self.size = 1
        self.metric = metric
        self.match_thresh = match_thresh
        self.blocked = []
        self.child_nodes = []
        
    def add_node(self, node):
        # Add child-parent index references
        self.nodes[node.parent].child_nodes.append(len(self.nodes))
#        print('Node ' + str(node.parent) + 'now has childs: ' + str(self.nodes[node.parent].child_nodes))
        
        # Check distance to all existing nodes
        dist_list = self.get_distances(node)
        dist_min = np.min(dist_list)
        dist_min_ind = np.nonzero(dist_list == dist_min)[0]
#        print(' ')
#        print('This path is: ' + str(node.path))
#        print('Closest Node in tree is on distance :' + str(dist_min))
        if dist_min < self.match_thresh:
            # The new node is matching an old node, do not add to graph
            
            self.nodes[dist_min_ind].alt_paths.append(node.path)
            self.blocked.append(node.path)
#            print('Node already exist with path' + str(self.nodes[dist_min_ind].path) + ', adding alternative path %')
        else:
            # The new node is unique, add to graph
            self.nodes.append(node)
            
            # Add distance list to distance matrix
            n = len(dist_list)
            self.size = n+1
            self.distances = extend_array(self.distances,(n+1, n+1)) 
            self.distances[n,:n] = dist_list
            self.distances[:n,n] = dist_list
            self.distances[n,n] = 0
    
    def calculate_distances(self, node_index):
        dist_list = self.getDistances(self.nodes[node_index])
        self.distances[node_index,:] = dist_list
        self.distances[:,node_index] = dist_list
    
    def get_distances(self, node):
        dist_list = []
        for tnode in self.nodes:
            dist_list.append(self.metric.distance(tnode.y, node.y))
        return dist_list
        
def extend_distances(distances):
    """
        Takes a distances array as input and returns 
        the array with one more row and one more 
        column filled with zeros.
    """
    return extend_array(distances, np.array(distances.shape) + (1, 1))

def extend_array(array, size):
    """
        Takes a array as input and returns 
        the array with shape size
    """
    s0, s1 = array.shape
    new_dist = -np.ones(size)
    new_dist[:s0, :s1] = array
    return new_dist
