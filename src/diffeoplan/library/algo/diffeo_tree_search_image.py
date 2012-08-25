from . import contract
from diffeoplan.library.images import UncertainImage
from diffeoplan.utils import memoize_instance
from . import DiffeoTreeSearch

class DiffeoTreeSearchImage(DiffeoTreeSearch):
    """
        It also knows that:
            - each plan gives an image
    """    
     

    def __init__(self, image, metric_collapse,
                        metric_collapse_threshold, *args, **kwargs):
        DiffeoTreeSearch.__init__(self, *args, **kwargs)
        self.image = image
        self.metric_collapse = metric_collapse        
        self.metric_collapse_threshold = metric_collapse_threshold
    
    def __str__(self):
        return 'DiffeoTreeSearchImage'

    @memoize_instance
    @contract(plan='seq(int)', returns=UncertainImage)
    def plan2image(self, plan):
        action = self.plan2action(plan)
        return action.predict(self.image)

    @memoize_instance
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


