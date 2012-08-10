import numpy as np



class Distance_L1():
    def distance(self, y0, y1):
        print('Using distance_L1 :).')
        diff = (y0.get_values() - y1.get_values()).flatten()
        return float(np.linalg.norm(diff, 1)) / y0.size

