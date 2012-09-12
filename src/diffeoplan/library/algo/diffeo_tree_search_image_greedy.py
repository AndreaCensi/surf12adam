from . import DiffeoTreeSearchImage, contract
from diffeoplan.library import UncertainImage
from . import dp_memoize_instance


__all__ = ['DiffeoTreeSearchImageGreedy']

class NodeStats(object):
    
    """ Statistics for each node that can be possibly be expanded. """
    def __init__(self, node, node_string, closeness, visibility):
        self.node = node
        self.node_string = node_string
        self.closeness = closeness
        self.visibility = visibility
        
    def __str__(self):
        return ('%20s closeness: %.5f visibility: %.2f' % 
                (self.node_string, self.closeness, self.visibility))
        

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
        
    # TODO: make sure we don't use memoize_instance anymore
    @dp_memoize_instance 
    def distance_to_attractor(self, n):
        image = self.plan2image(n)
        return self.metric_attractor.distance(image, self.attractor)
        
    def choose_open_nodes(self, open_nodes):
        # compute all statistics
        def compute_stats(node):
            node_string = self.node_friendly(node)
            visibility = self.visibility(node)
            closeness = self.distance_to_attractor(node)
            return NodeStats(node=node, node_string=node_string,
                             visibility=visibility, closeness=closeness)
        
        # compute stats for all open nodes
        stats = map(compute_stats, open_nodes)
        
        # find the closest node
        stats.sort(key=lambda x: x.closeness)
        chosen = stats[0].node
        self.log_chosen_dist(chosen, stats)
        return chosen
        
    def log_chosen_dist(self, chosen, stats):
        self.info('open nodes and distances:')
        max_num = 10
        if len(stats) > max_num:
            self.info('(showing %d of %d nodes)' % (max_num, len(stats)))
            stats = stats[:max_num]
            
        for s in stats:
            self.info('- %s' % s)
        
        self.info('Greedily opening %s (%s)' % 
                  (chosen, self.node_friendly(chosen)))
        self.info('= %s' % stats[0])

