import numpy as np




class Distance_L2():
    def __init__(self):
        self.name = 'Distance_L2'
        
    def distance(self, y0, y1):
        diff = (y0.get_values() - y1.get_values()).flatten()
        return float(np.linalg.norm(diff, 2)) #/ y0.size

