import numpy as np
from diffeoplan.library.graph.node import Node
from diffeoplan.library.algo.graphsearch_breadth import GraphSearchQueue
from contracts import contract

class RandomizedExpansion(GraphSearchQueue):
    """  
    """

    def __init__(self,  thresh, metric, max_ittr, nsteps, 
                   region):
        GraphSearchQueue.__init__(self,  thresh, metric, max_ittr, nsteps)
        self.region = region
            
        
    def get_next_index(self, tree, open_nodes):
        """ Choose one element of open """
        number_of_neighbors = [self.get_number_of_neighbors(tree, i)
                               for i in self.open]
        assert np.all(number_of_neighbors >= 1)
        pmf = 1.0 / number_of_neighbors
        
        exp_index = random_pmf(pmf)
        return open_nodes[exp_index]
        
    @contract(returns='int,>=1')
    def get_number_of_neighbors(self, tree, node_index):
        """ Returns the number of neighbors for a certain node. 
            Returns at least one because every one is a self-neighbor.
        """
        num = 0
        for j in range(len(tree.nodes)):
            d = tree.get_nodes_distance(j, node_index)
            if d <= self.region:
                num += 1
        return num

#    
#        
#    def get_next_node_index(self, tree):
#        # Sample-bias-neighbord
#        close_enough = tree.distances < self.region
#        pmf = 1.0 / np.sum(close_enough, axis=0)
#        
#        # Make sure full-length paths is not expanded
#        for i in range(len(pmf)):
#            too_deep = len(tree.nodes[i].path) >= self.nsteps 
#            if too_deep:
#                pmf[i] = 0
#                
#        # If they are all zero, it means 
#        # that the maximum length has been reached
#        # so we return None
#        if np.all(pmf == 0):
#            return None
#                
#        exp_index = random_pmf(pmf)
#        return exp_index
    
    
    def get_next_cmd(self, tree, node_index, available):
        choice = np.random.randint(0, len(available) - 1)
        return available[choice]
#    
#    def get_next_cmd(self):
#        dds = self.get_dds()
#        n = len(dds.actions)
#        return np.random.randint(0, n - 1)
#    
#    def expand_start_tree(self, tree):       
#        # Node index to expand
#        next_node_index = self.get_next_node_index(tree)
#        if next_node_index == None:
#            return None
#        
#        next_cmd = self.get_next_cmd()
#        
#        return get_next_node(tree, next_node_index, next_cmd, 
#                             dds= self.get_dds())

        

        

def random_pmf(pmf):
    """
        returns a random integer from the probability mass function <pmf> given as a vector.    
    """
    total_prob = sum(pmf)
    pmf = np.array(pmf) / total_prob
    prob = pmf[0]
#    pdb.set_trace()
    rval = np.random.rand()
    
    for i in range(len(pmf) - 1):
        if rval < prob:
            return i
        prob = prob + pmf[i + 1]
    return len(pmf) - 1
