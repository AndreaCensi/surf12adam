from contracts import contract
from abc import ABCMeta, abstractmethod
from geometry import R2, Torus01


class Topology:
    PLANE = 'plane'
    TORUS = 'torus'
    KNOWN = {}
    KNOWN[PLANE] = R2 
    KNOWN[TORUS] = Torus01(2)


class SymbolicDiffeo:
    """ This class is the interface for a symbolic diffeomorphism. """
    
    __metaclass__ = ABCMeta
    
    def __init__(self, topology):
        self.topology_s = topology
        if not topology in Topology.KNOWN:
            raise ValueError('Unknown topology %r' % topology)
        self.topology = Topology.KNOWN[topology]

    def get_topology(self):
        """ Returns the topology under which this is a diffeomorphism """
        return self.topology
    
    @abstractmethod
    def get_inverse(self):
        """ 
            Returns the symbolic diffeomorphism that is the inverse 
            of this one. 
        """
         
    @abstractmethod
    @contract(point='array[2]')
    def apply(self, point):
        """ 
            Computes the diffeomorphism applied to a point. 
        """

