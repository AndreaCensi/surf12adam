import numpy as np
from contracts import contract

class Graph():
    def __init__(self, root_node, metric, match_thresh):
        self.nodes = [root_node]
        self.distances = np.zeros((1, 1))
        self.size = 1
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
#        # Check distance to all existing nodes
#        dist_list = self.get_distances_except_me(node)
#        dist_min = np.min(dist_list)
#        dist_min_ind = np.nonzero(dist_list == dist_min)[0]
#        if dist_min < self.match_thresh:
#            # The new node is matching an old node, do not add to graph            
#            self.nodes[dist_min_ind].alt_paths.append(node.path)
#            self.blocked.append(node.path)
#        else:
#            # The new node is unique, add to graph
#            self.nodes.append(node)
#            
#            # Add distance list to distance matrix
#            n = len(dist_list)
#            self.size = n + 1 
#            self.distances = extend_array(self.distances, (n + 1, n + 1))  
#            self.distances[n, :n] = dist_list 
#            self.distances[:n, n] = dist_list 
#            self.distances[n, n] = 0 
    
#    def calculate_distances(self, node_index):
#        dist_list = self.getDistances(self.nodes[node_index])
#        self.distances[node_index, :] = dist_list 
#        self.distances[:, node_index] = dist_list 
#    
#    def get_distances(self, node):
#        dist_list = []
#        for tnode in self.nodes:
#            dist_list.append(self.metric.distance(tnode.y, node.y))
#        return dist_list
#    
#    def get_closest_node(self):
#        '''
#        
#        '''
##        pdb.set_trace()
#        min_ind = np.nonzero(self.connector.distances == np.min(self.connector.distances)) 
#        if self.connector.T1 == self:
#            print('current path' + str(self.nodes[min_ind[0][0]].path))
#            return min_ind[0][0]
#        if self.connector.T2 == self:
#            return min_ind[0][1]
#        print('Node evaluation failed, get_closest_node')
#        return None
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

    
#    
#def extend_distances(distances):
#    """
#        Takes a distances array as input and returns 
#        the array with one more row and one more 
#        column filled with zeros.
#    """
#    return extend_array(distances, np.array(distances.shape) + (1, 1))
#
#def extend_array(array, size):
#    """
#        Takes a array as input and returns 
#        the array with shape size
#    """
#    s0, s1 = array.shape
#    new_dist = -np.ones(size)
#    new_dist[:s0, :s1] = array
#    return new_dist
