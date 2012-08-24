from diffeoplan.utils.memoization import memoize_instance
from networkx.classes.multigraph import MultiGraph
from contracts import contract

class Connector():
    
#    @contract(G1=MultiGraph, G2=MultiGraph)
#    def __init__(self, G1, G1_node2value, G2, G2_node2value, metric, threshold):
    def __init__(self, tree1, tree2, metric, threshold):
        '''
        :param G1: First graph
        :param G1_node2value: how to map each node to a value
        :param G2: Second graph
        :param G2_node2value: how to map each node to a value
        :param metric: a metric on values
        :param threshold: threshold for matching
        '''
#        self.G1 = G1
#        self.G2 = G2
#        self.G1_node2value = G1_node2value
#        self.G2_node2value = G2_node2value
        self.tree1 = tree1
        self.tree2 = tree2
        self.metric = metric
        self.threshold = threshold
        
    def update(self, new_nodes_G1, new_nodes_G2):
        pass
    
    @memoize_instance
    def distance(self, node1, node2):
        '''
        Returns the distance between nodes
        :param node1: A node in G1
        :param node2: A node in G2
        '''
        v1 = self.value1(node1)
        v2 = self.value2(node2)
        return self.metric.distance(v1, v2)
    
    @memoize_instance
    def close_enough(self, node1, node2):
        return self.distance(node1, node2) <= self.threshold
        
    @memoize_instance
    def value1(self, node1):
        assert node1 in self.tree1.G
        return self.tree1.plan2image(node1)
    
    @memoize_instance
    def value2(self, node2):
        assert node2 in self.tree2.G
        return self.tree2.plan2image(node2)
    
    def get_connections(self):
        """ Returns all connections between the two graphs. """
        for n1 in self.tree1.G:
            for n2 in self.tree2.G:
                if self.close_enough(n1, n2):
                    yield n1, n2
            
            
            
    
