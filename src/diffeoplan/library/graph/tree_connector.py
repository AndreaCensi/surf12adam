import numpy as np

#import pdb
from diffeoplan.library.graph.tree import extend_array
from numpy.core.numeric import inf

class TreeConnector():
    inf = 1e10
    def __init__(self, T1, T2, tresh):
        self.tresh = tresh
        self.connections = -np.ones((T1.size,T2.size))
        self.distances = inf*np.ones((T1.size,T2.size)).astype(np.float)
        self.T1 = T1
        self.T2 = T2
        self.T1.connector = self
        self.T2.connector = self
        
    def connection(self, n1, n2):
#        pdb.set_trace()
        dist = self.T1.metric.distance(self.T1.nodes[n1].y, self.T2.nodes[n2].y)
#        print('Distance ' + str(dist))
        if dist < self.tresh:
            return 1, dist
        else:
            return 0, inf
    
    def connect_reset(self):
        self.connections = -np.ones((self.T1.size,self.T2.size))
        self.distances = inf*np.ones((self.T1.size,self.T2.size)).astype(np.float)
        
        
    def connect_update(self):
        self.connections = extend_array(self.connections,(self.T1.size, self.T2.size))
        self.distances = extend_array(self.connections,(self.T1.size, self.T2.size))
        undef = np.array(np.nonzero((self.connections==-1).astype(np.int)))
        for i in range(undef.shape[1]):
            n1 = undef[0,i]
            n2 = undef[1,i]
#            pdb.set_trace()
            self.connections[n1,n2], self.distances[n1,n2] = self.connection(n1, n2)
             
        npaths = np.sum((self.connections==1).astype(np.int))
        return npaths
    
    def get_connection(self):
        npaths = np.sum((self.connections==1).astype(np.int))
        if npaths >0:
            paths = np.array(np.nonzero((self.connections==1).astype(np.int)))
            plan = self.T1.nodes[paths[0,0]].path
            return plan
        else:
            return None
#            for i in range(npaths):
#                plans.append(self.T1.nodes[paths[0,i]])
#                ToDo implement support for second tree's plan
#                self.T2.nodes[paths[1,i]]

        