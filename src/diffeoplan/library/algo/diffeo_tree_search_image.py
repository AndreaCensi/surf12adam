from . import DiffeoTreeSearch, contract, dp_memoize_instance
from diffeo2dds.model.uncertain_image import UncertainImage


__all__ = ['DiffeoTreeSearchImage']


class DiffeoTreeSearchImage(DiffeoTreeSearch):
    """
        It also knows that:
            - each plan gives an image
    """    
     
    def __init__(self, image, metric_collapse,
                        metric_collapse_threshold, **kwargs):
        DiffeoTreeSearch.__init__(self, **kwargs)
        self.image = image
        self.metric_collapse = metric_collapse        
        self.metric_collapse_threshold = metric_collapse_threshold
        
    def __str__(self):
        return 'DiffeoTreeSearchImage'

    @dp_memoize_instance
    @contract(plan='seq(int)', returns=UncertainImage)
    def plan2image(self, plan):
        action = self.plan2action(plan)
        return action.predict(self.image)

    @dp_memoize_instance
    @contract(plan1='seq(int)', plan2='seq(int)', returns='>=0')
    def distance_image(self, plan1, plan2):
        """ Returns the distance between images obtained by the plans. """
        return self.metric_collapse.distance(self.plan2image(plan1),
                                             self.plan2image(plan2))

    def node_compare(self, n1, n2):
        """ Subclass this for adding better conditions """
        if DiffeoTreeSearch.node_compare(self, n1, n2):
            return True
        d = self.distance_image(n1, n2) 
        return d <= self.metric_collapse_threshold

    def available_actions(self, node):
        image = self.plan2image(node)
        if image.get_scalar_uncertainty().max() == 0:
            self.log_node_uncertain(node)
            return []
        return DiffeoTreeSearch.available_actions(self, node)
        
    def log_node_uncertain(self, node):
        self.info('Skipping plan %s because uncertainty is too much.' 
                  % self.node_friendly(node))
        self.info('image: %s' % self.plan2image(node))
        
 
