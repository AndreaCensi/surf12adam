from . import contract, np

class SquareDomain:
    @contract(bounds='seq[N,>0](seq[2])')
    def __init__(self, bounds):
        self.bounds = bounds
        self.upper = np.array([x[1] for x in bounds])
        self.lower = np.array([x[0] for x in bounds])
    
    @contract(p='array[N]')
    def belongs(self, p):
        return np.all(np.logical_and(p >= self.lower, p <= self.upper))
