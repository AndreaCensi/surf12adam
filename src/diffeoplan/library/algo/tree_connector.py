from . import  np
from diffeoplan.utils import (WithInternalLog, assert_allclose,
    construct_matrix_iterators)
import itertools
import warnings
from diffeoplan.library.algo.memoize_strategy import Memoized


__all__ = ['Connector', 'ConnectorMatch']


class ConnectorMatch(object):
    """ Information associated to a connection between the two trees. """
    def __init__(self, n1, n2, s1, s2, plan, v1, v2, vplan, distance_branch,
                 distance_prediction):
        self.n1 = n1
        self.n2 = n2
        self.s1 = s1
        self.s2 = s2
        self.plan = plan
        self.v1 = v1
        self.v2 = v2
        self.vplan = vplan
        self.distance_branch = distance_branch
        self.distance_prediction = distance_prediction
        
    def __str__(self):
        return ('%20s -> %20s (br: %5f pr: %5f) (v1 %.2f v2 %.2f v %.2f)' %
                (self.s1, self.s2, self.distance_branch, self.distance_prediction,
                 self.v1, self.v2, self.vplan))
        
    
class Connector(WithInternalLog, Memoized):
    
    def __init__(self, tree1, tree2, metric, threshold, min_visibility):
        '''
        :param tree1: First graph
        :param tree2: Second graph        
        :param metric: a metric on values
        :param threshold: threshold for matching
        :param min_vibility: A threshold for the visibility of the joint plan.
        '''
        self.tree1 = tree1
        self.tree2 = tree2
        self.metric = metric
        self.threshold = threshold
        self.min_visibility = min_visibility
        
        WithInternalLog.__init__(self)
        Memoized.__init__(self)
        # no output
        #self.set_log_output(False)
        
    def __str__(self):
        return "Connector(%s<=%s)" % (self.metric, self.threshold)
    
    def update(self, new_nodes_G1, new_nodes_G2):
        pass
    
    @Memoized.dec()
    def distance(self, node1, node2):
        '''
        Returns the distance between nodes as the distance
        between computed images according to our metric.
        
        :param node1: A node in G1
        :param node2: A node in G2
        '''
        warnings.warn('Remember to make this choice a configuration switch')
        # this is good as heuristics, but we want the real distance
        # to check that we are below the threshold
        v1 = self.value1(node1)
        v2 = self.value2(node2)
        return self.metric.distance(v1, v2)
    
    @Memoized.dec()
    def distance_prediction(self, node1, node2):
        """
            Returns the distance between nodes as the distance
            between (node1+R(node2)) * start and goal.
        
        """    
        plan = node1 + tuple(reversed(node2))
        v1 = self.value1(plan)
        v2 = self.value2(())
        return self.metric.distance(v1, v2)
        
    def close_enough(self, node1, node2):
        """ 
            Returns True if the connection can be considered a solution.
            This implies that the distance is close enough and 
            that the visiblity of the plan is high enough (min_visibility). 
        """    
        # First check that the distance between images is small
        # because that is cheap to compute
        branches_threshold = self.threshold * 1.2 # this should be a parameter
        branches_close_enough = self.distance(node1, node2) <= branches_threshold 
        if not branches_close_enough:
            # This assumes that d(p,q) < d(pq,0), but needs more thought.
            return False

        # Now check that the prediction is close enough
        # as this takes longer to compute.
        prediction_close_enough = self.distance_prediction(node1, node2) <= self.threshold
        
        if not prediction_close_enough:
            return False
        
        # Now check the visibility
        plan = self._nodes_to_plan(node1, node2)
        visibility = self.tree1.visibility(plan)
        
        visible = visibility >= self.min_visibility
        if not visible:
            return False 
        
        return True
        
    # Don't memoize this, it is already memoized in tree1
    def value1(self, node1):
        return self.tree1.plan2image(node1)
    
    # Don't memoize this, it is already memoized in tree2
    def value2(self, node2):
        return self.tree2.plan2image(node2)
    
    def get_connections(self):
        """ Returns all connections between the two graphs. """
        matches = self.compute_connections(only_close_enough=True)
        if matches:
            self.show_connections(matches, 'matches')
            
        # self.print_minimum()
        return matches

    def _nodes_to_plan(self, node1, node2):
        """ Returns the plan obtained by joining the trees at the given
            nodes. Note that the sequence of node2 must be reversed. """
        return node1 + tuple(reversed(node2))
    
    def compute_connections(self, only_close_enough=False):
        """ If only_close_enough is True, only returns those
            that pass the tests to be a solution (that is, they
            are close in the metric and the visibility is high.) """
        connections = []
        for n1, n2 in itertools.product(self.tree1.G, self.tree2.G):
            close = self.close_enough(n1, n2)
            if not close and only_close_enough:
                continue
            plan = self._nodes_to_plan(n1, n2)
            conn = ConnectorMatch(n1=n1,
                                    s1=self.tree1.node_friendly(n1),
                                    v1=self.tree1.visibility(n1),
                                    n2=n2,
                                    s2=self.tree2.node_friendly(n2),
                                    v2=self.tree1.visibility(n2),
                                    plan=plan,
                                    vplan=self.tree1.visibility(plan),
                                    distance_branch=self.distance(n1, n2),
                                    distance_prediction=self.distance_prediction(n1, n2))
            connections.append(conn)
        connections.sort(key=lambda x: x.distance_prediction)
        return connections
    
    
    def show_connections(self, connections, title, num=10):
        self.info('Connections (%s):' % title)
        
        if len(connections) > num:
            self.info(' (showing %d of %d)' % (num, len(connections)))
            connections = connections[:num]
            
        for c in connections: 
            self.info('- %s' % c.__str__())
        
    def print_minimum(self):
        self.info('print_minimum:')
        connections = self.compute_connections(only_close_enough=False)
        
        self.show_connections(connections, 'all of them')    
    
        nodes1 = list(self.tree1.G.nodes())
        nodes2 = list(self.tree2.G.nodes())
        
        # TODO: do not compute all of this
        Db = construct_matrix_iterators((nodes1, nodes2),
                                       lambda n1, n2: self.distance(n1, n2))
        Dp = construct_matrix_iterators((nodes1, nodes2),
                                       lambda n1, n2: self.distance_prediction(n1, n2))
        Db_min, m = md_argmin(Db)
        n1 = nodes1[m[0]]                    
        n2 = nodes2[m[1]]
        assert_allclose(Db_min, self.distance(n1, n2)) 
        self.info('Minimum distance_branch: %g between %s and %s' % 
                  (Db_min, self.tree1.node_friendly(n1),
                            self.tree2.node_friendly(n2)))

        Dp_min, m = md_argmin(Dp)
        n1 = nodes1[m[0]]                    
        n2 = nodes2[m[1]]
        assert_allclose(Dp_min, self.distance_prediction(n1, n2))
        self.info('Minimum distance_prediction: %g between %s and %s' % 
                  (Dp_min, self.tree1.node_friendly(n1),
                           self.tree2.node_friendly(n2)))


def md_argmin(a):
    """ Returns the value and the index coordinate of a multidimensional array """
    min_value = np.min(a)
    index_flat = np.argmin(a)
    ij = np.unravel_index(index_flat, a.shape)
    ij = tuple(ij)
    assert_allclose(a[ij], min_value)
    return a[ij], ij

