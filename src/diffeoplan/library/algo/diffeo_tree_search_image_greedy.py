from . import DiffeoTreeSearchImage, contract, np
from diffeoplan.library.images import UncertainImage
from diffeoplan.utils import memoize_instance
from functools import partial
       
class DiffeoTreeSearchImageGreedy(DiffeoTreeSearchImage):
    """ 
        A greedy variant that expands the node which is closest 
        to a given attractor. 
    """
    def __init__(self, metric_attractor, **kwargs):
        self.metric_attractor = metric_attractor
        super(DiffeoTreeSearchImageGreedy, self).__init__(**kwargs)
        
    @contract(attractor=UncertainImage)
    def set_attractor(self, attractor):
        self.attractor = attractor
        
    #@memoize_instance # XXX: temporary
    def distance_to_attractor(self, n):
        image = self.plan2image(n)
        return self.metric_attractor.distance(image, self.attractor)
        
    def choose_open_nodes(self, open_nodes):
        closeness = map(self.distance_to_attractor, open_nodes)
        best = np.argmin(closeness)
        chosen = open_nodes[best]
        chosen_dist = closeness[best]
        self.log_chosen_dist(open_nodes, closeness, chosen, chosen_dist)
        return chosen
        
    def log_chosen_dist(self, open_nodes, closeness, chosen, chosen_dist): #@UnusedVariable
        self.info('open nodes and distances:')
        for i, n in enumerate(open_nodes):
            self.info('- %10f %s' % (closeness[i], self.node_friendly(n)))
        
        self.info('Greedily opening dist %f plan %s' % 
                  (chosen_dist, self.node_friendly(chosen)))
        

class DiffeoTreeSearchImageGreedyTree(DiffeoTreeSearchImage):
    """ 
        A greedy variant that expands the node which is closest 
        to the other tree. 
    """
    def __init__(self, metric_attractor, **kwargs):
        self.metric_attractor = metric_attractor
        super(DiffeoTreeSearchImageGreedyTree, self).__init__(**kwargs)
        
    @contract(other_tree=DiffeoTreeSearchImage)
    def set_other_tree(self, other_tree):
        """ Sets the other tree. """
        self.other_tree = other_tree
        
    @memoize_instance
    def distance_to_other_tree_node(self, node, node_other):
        '''
            Computes the distance between a node and the node in another tree.
        
            :param node: A plan.
            :param node_other: A plan in ``other_tree``.
        '''
        image1 = self.plan2image(node)
        image2 = self.other_tree.plan2image(node_other)
        return self.metric_attractor.distance(image1, image2)
    
    def distance_to_attractor(self, node):
        """ 
            Returns the distance and the node it is closest to
            in the other tree. 
        """
        other_nodes = list(self.other_tree.G)
        d = partial(self.distance_to_other_tree_node, node)
        distances = map(d, other_nodes)
        i = np.argmin(distances)
        return distances[i], other_nodes[i]
        
    def choose_open_nodes(self, open_nodes):
        closeness = [self.distance_to_attractor(x)[0] for x in open_nodes]
        best = np.argmin(closeness)
        chosen = open_nodes[best]
        chosen_dist = closeness[best]
        other_closest = self.distance_to_attractor(chosen)[1]
        self.log_chosen_dist(open_nodes, closeness, chosen, chosen_dist, other_closest)
        return chosen
        
    def log_chosen_dist(self, open_nodes, closeness, chosen, chosen_dist, other_closest): #@UnusedVariable
        self.info('open_nodes: %s' % open_nodes)
        self.info('Distances: %s' % closeness)
        self.info('Greedily-tree opening dist %f plan %s close to %s' % 
                  (chosen_dist, self.node_friendly(chosen),
                   self.other_tree.node_friendly(other_closest)))
        
        
