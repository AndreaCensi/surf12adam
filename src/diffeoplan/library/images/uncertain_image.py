from . import contract, np, logger


class UncertainImage():
    
    @contract(values='array[HxWx...]', scalar_uncertainty='None|array[HxW]')
    def __init__(self, values, scalar_uncertainty=None):
        self.values = values
        self.scalar_uncertainty = scalar_uncertainty
        
    def get_values(self):
        return self.values
        

    @staticmethod
    @contract(returns='dict(str:number)')
    def compute_all_distances(y0, y1):
        """ Computes all measures of difference between y0 and y1. 
            Returns a dictionary. """
        r = {}
        r['values_L2'] = UncertainImage.dist_values_L2(y0, y1)
        r['values_L1'] = UncertainImage.dist_values_L1(y0, y1)
        # ...
        return r
    
    @staticmethod
    def dist_values_L2(y0, y1):
        diff = (y0.values - y1.values).flatten()
        return np.linalg.norm(diff, 2)

    @staticmethod
    def dist_values_L1(y0, y1):
        diff = (y0.values - y1.values).flatten()
        return np.linalg.norm(diff, 1)
    
