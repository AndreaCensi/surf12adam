from . import DiffeoTreeSearchImage, contract, np
from diffeoplan.library import UncertainImage
from diffeoplan.utils import memoize_instance

       
__all__ = ['DiffeoTreeSearchImageGreedy']

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
        
    @memoize_instance 
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
        

