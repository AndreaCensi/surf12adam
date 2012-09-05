from . import np
from diffeoplan.utils import (WithInternalLog, assert_allclose,
    construct_matrix_iterators, memoize_instance)
import itertools

__all__ = ['Connector']

class Connector(WithInternalLog):
    
    def __init__(self, tree1, tree2, metric, threshold):
        '''
        :param tree1: First graph
        :param tree2: Second graph        
        :param metric: a metric on values
        :param threshold: threshold for matching
        '''
        self.tree1 = tree1
        self.tree2 = tree2
        self.metric = metric
        self.threshold = threshold
        WithInternalLog.__init__(self)
        
        # no output
        self.set_log_output(False)
        
    def __str__(self):
        return "Connector(%s<=%s)" % (self.metric, self.threshold)
    
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
        #assert node1 in self.tree1.G
        return self.tree1.plan2image(node1)
    
    @memoize_instance
    def value2(self, node2):
        #assert node2 in self.tree2.G
        return self.tree2.plan2image(node2)
    
    def get_connections(self):
        """ Returns all connections between the two graphs. """
        matches = []
        for n1, n2 in itertools.product(self.tree1.G, self.tree2.G):    
            if self.close_enough(n1, n2):
                matches.append((self.distance(n1, n2), n1, n2))
        # Choose the best one
        matches.sort(key=lambda x: x[0])
        self.print_minimum()
        return matches

    def print_minimum(self):
        self.info('print_minimum:')
        for n1, n2 in itertools.product(self.tree1.G, self.tree2.G):
            d = self.distance(n1, n2)    
            s1 = self.tree1.node_friendly(n1)
            s2 = self.tree1.node_friendly(n2)
            self.info('- %5f %20s %20s' % (d, s1, s2))
        #for n1, n2 in itertools.product(self.tree1.G, self.tree2.G)
        
        nodes1 = list(self.tree1.G.nodes())
        nodes2 = list(self.tree2.G.nodes())
        
        f = lambda n1, n2: self.distance(n1, n2)
        D = construct_matrix_iterators((nodes1, nodes2), f)
        m = md_argmin(D)
        n1 = nodes1[m[0]]                    
        n2 = nodes2[m[1]]
        assert_allclose(D[m], self.distance(n1, n2)) 
        s1 = self.tree1.node_friendly(n1)
        s2 = self.tree1.node_friendly(n2)
        self.info('Minimum distance: %g between %s and %s' % (np.min(D), s1, s2))
        #self.info('y1: %s' % self.value1(n1))            
        #self.info('y2: %s' % self.value2(n2))

def md_argmin(a):
    """ Returns the index coordinate of a multidimensional array """
    index_flat = np.argmin(a)
    ij = np.unravel_index(index_flat, a.shape)
    ij = tuple(ij)
    assert_allclose(a[ij], np.min(a))
    return ij
