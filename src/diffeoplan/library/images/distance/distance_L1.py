import numpy as np
import pdb



class DistanceNorm():

    def __init__(self, order):
        self.order = order
        
    def distance(self, y0, y1):
        v0 = y0.get_values()
        v1 = y1.get_values()
        diff = (v0 - v1).flatten()
        pdb.set_trace()
        var = (y0.scalar_uncertainty * y0.scalar_uncertainty).flatten()
        var_flat = []
        for _ in range(2):
            var_flat = np.concatenate(var_flat,var)
        return float(np.linalg.norm(diff, self.order)) / diff.size

