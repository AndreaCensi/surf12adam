from . import contract, np, SymbolicDiffeo

class LinearCoordinateChange(SymbolicDiffeo):
    
    @contract(b1='seq[N](seq[2](number))',
              b2='seq[N](seq[2](number))')
    def __init__(self, b1, b2):
        """
            b1 = [ [xmin, xmax], [ymin, ymax] , [ ... ]]
        """
        self.n = len(b1)
        self.b1 = b1
        self.b2 = b2
        SymbolicDiffeo.__init__(self, 'plane')
        
    @contract(p='array[N]', returns='array[N]')
    def apply(self, p):
        q = np.zeros(self.n)
        for i in range(self.n):
            q[i] = self.apply1d(p[i], self.b1[i], self.b2[i])
        return q
    
    def apply1d(self, p, b1, b2):
        q = (p - b1[0]) / (b1[1] - b1[0])
        return q * (b2[1] - b2[0]) + b2[0]

    def get_inverse(self):
        return LinearCoordinateChange(self.b2, self.b1)
