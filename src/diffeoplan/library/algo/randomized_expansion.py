from diffeoplan.configuration import get_current_config
import copy
import numpy as np
import numpy.linalg as LA
from diffeoplan.library.algo.graphsearch import GraphSearch
from diffeoplan.library.graph.node import Node


class RandomizedExpansion(GraphSearch):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 

    """
    
    def __init__(self, max_ittr, thresh, region, nsteps, metric, directions=1):
        '''
        :param nsteps: Number of steps in the random guess.
        '''        
        config = get_current_config()
        self.metric = config.distances.instance(metric)
        self.thresh = thresh
        self.nsteps = nsteps
        self.directions = directions
        
        self.max_ittr = max_ittr
        self.directions = directions
        self.region = region
        
        config = get_current_config()
        self.metric = config.distances.instance(metric)

    def get_next_node(self, tree):
        pmf = 1 / np.sum((tree.distances < self.region).astype(np.float), 0)
        
        # Make sure full-length paths is not expanded
        for i in range(len(pmf)):
            if len(tree.nodes[i].path) >= self.nsteps:
                pmf[i] = 0
        print('Norm pmf: ' + str(LA.norm(pmf)))
        exp_index = random_pmf(pmf)
        return exp_index
    
    def get_next_cmd(self):
        dds = self.get_dds()
        n = len(dds.actions)
        return np.random.randint(0, n - 1)
    
    def get_new_node(self, tree):
                
        dds = self.get_dds()
        ncmd = len(dds.actions)
        
        # Node index to expand
        next_node = self.get_next_node(tree)
        if next_node == None:
            return None
        
        node = tree.nodes[next_node]
        path = copy.deepcopy(node.path)
        next_cmd = self.get_next_cmd()
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
