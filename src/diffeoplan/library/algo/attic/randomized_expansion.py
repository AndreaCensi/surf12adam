import numpy as np
from diffeoplan.library.algo.graphsearch_breadth import GraphSearchQueue
from contracts import contract

class RandomizedExpansion(GraphSearchQueue):
    """  
    Chooses a node to expand according to a bias-function
    """

    def __init__(self, thresh, metric, max_ittr, nsteps,
                       region):
        GraphSearchQueue.__init__(self, thresh, metric, max_ittr, nsteps)
        self.region = region
        
    def get_next_index(self, tree, open_nodes):
        """ Choose one element of open """
        number_of_neighbors = np.array([self.get_number_of_neighbors(tree, i)
                               for i in open_nodes])
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

    def get_next_cmd(self, tree, node_index, available):
        if len(available)==1:
            return 0
        else:
            choice = np.random.randint(0, len(available) - 1)
            return available[choice]
        

def random_pmf(pmf):
    """
        Returns a random integer from the probability mass 
        function <pmf> given as a vector.    
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
