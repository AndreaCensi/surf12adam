import numpy as np



class DistanceNorm():

    def __init__(self, order):
        self.order = order
        
    def distance(self, y0, y1):
        v0 = y0.get_values()
        v1 = y1.get_values()
        diff = (v0 - v1).flatten()
        return float(np.linalg.norm(diff, self.order)) / diff.size

