from . import contract, np
from ..  import UncertainImage
 

class DistanceNorm():

    @contract(order='int,>=1')
    def __init__(self, order):
        self.order = order
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns='>=0')
    def distance(self, y0, y1):
        diff = self.error_field(y0, y1)        
        distance = element_by_element_norm_scaled(diff, self.order)
        return distance

    @contract(y0=UncertainImage, y1=UncertainImage, returns='array[HxW]')
    def error_field(self, y0, y1):
        """ Returns the error per pixel. """
        v0 = y0.get_values()
        v1 = y1.get_values()        
        # compute difference
        diff = np.abs(v0 - v1)
        # flatten if 3dim
        if diff.ndim == 3:
            diff = diff.sum(axis=2)
        return diff

    def __repr__(self):
        return 'DistanceNorm(%s)' % self.order

    def __str__(self):
        return 'L%d' % self.order

@contract(x='array', order='int,>=1', returns='>=0')
def element_by_element_norm(x, order):
    """ Note that np.linalg.norm does something different. """
    return np.power(np.sum(np.power(np.abs(x), order)), 1.0 / order)

@contract(x='array', order='int,>=1', returns='>=0,<=1')
def element_by_element_norm_scaled(x, order):
    """ Normalizes the norm in [0,1], assuming that x is normalized in [0,1]. """
    n = float(x.size)
    max_norm = np.power(n, 1.0 / order)
    norm = element_by_element_norm(x, order)
    return norm / max_norm

    
    
    
