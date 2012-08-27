from . import DistanceNorm, contract, np, element_by_element_norm
from .. import UncertainImage

class DistanceNormWeighted(DistanceNorm):
    """ 
        Like DistanceNorm but we only count the percentage
        that they have in common.
    """
    
    def __repr__(self):
        return 'DistanceNormWeighted(%s)' % self.order
    
    def __str__(self):
        return 'L%dw' % self.order
    
    @contract(y0=UncertainImage, y1=UncertainImage, returns='float,>=0')
    def distance(self, y0, y1):
        diff = self.error_field(y0, y1)
        
        u0 = y0.get_scalar_uncertainty()
        u1 = y1.get_scalar_uncertainty()
         
        w = np.sqrt(u0 * u1)
        w_sum = w.sum()
        
        if w_sum == 0:
            return 1.0 # Not sure, like always
        else:
            diff_norm = element_by_element_norm(diff * w, self.order)
            factor = np.power(w_sum, 1.0 / self.order)
            assert factor >= 0
        
        return diff_norm / factor
        
