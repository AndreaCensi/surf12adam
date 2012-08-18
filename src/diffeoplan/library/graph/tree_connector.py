import numpy as np

from numpy.core.numeric import inf

class TreeConnector():
    
    def __init__(self, T1, T2, tresh):
        self.tresh = tresh
#        self.connections = -np.ones((len(T1.nodes), len(T2.nodes))) 
#        self.distances = inf * np.ones((len(T1.nodes), len(T2.nodes))).astype(np.float) 
        self.T1 = T1
        self.T2 = T2
        self.T1.connector = self
        self.T2.connector = self
        self.connect_reset()
        
    def connection(self, n1, n2):
#        pdb.set_trace()
        dist = self.T1.metric.distance(self.T1.nodes[n1].y, self.T2.nodes[n2].y)
        print('Distance %s' % dist)
        if dist < self.tresh:
            return 1, dist
        else:
            return 0, dist
    
    def connect_reset(self):
        self.connections = -np.ones((len(self.T1.nodes), len(self.T2.nodes))) 
        self.distances = inf * np.ones((len(self.T1.nodes), len(self.T2.nodes))).astype(np.float) 
        
        
    def connect_update(self):
        self.connections = extend_array(self.connections, (len(self.T1.nodes), len(self.T2.nodes))) 
        self.distances = extend_array(self.distances, (len(self.T1.nodes), len(self.T2.nodes))) 
        undef = np.array(np.nonzero((self.connections == -1).astype(np.int))) 
        for i in range(undef.shape[1]):
            n1 = undef[0, i] 
            n2 = undef[1, i] 
            con, dis = self.connection(n1, n2)
            print('connection %s distance %s' % (con, dis))
            self.connections[n1, n2] = con
            self.distances[n1, n2] = dis
    
             
        npaths = np.sum((self.connections == 1).astype(np.int)) 
        return npaths
    
    def get_closest(self):
        self.connect_update()
        min_list = np.nonzero(self.nonnections == np.min(self.connections))
        return min_list[0][0], min_list[1][0]
    
    def get_connection(self):
        npaths = np.sum((self.connections == 1).astype(np.int)) 
        if npaths > 0: 
            paths = np.array(np.nonzero((self.connections == 1).astype(np.int)))
#            pdb.set_trace() 
            plan_start = self.T1.nodes[paths[0][0]].path 
            plan_goal = self.T2.nodes[paths[1][0]].path
            plan = list(np.concatenate((plan_start, np.flipud(plan_goal))).astype(np.int)) 
            return plan
        else:
            return None
        

def extend_array(array, size):
    """
        Takes a array as input and returns 
        the array with shape array.shape + extend.
    """
    s0, s1 = array.shape
    new_dist = -np.ones(size)
    new_dist[:s0, :s1] = array
    return new_dist
