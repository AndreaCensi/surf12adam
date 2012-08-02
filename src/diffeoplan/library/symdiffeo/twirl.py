from . import SymbolicDiffeo, Topology, NoInverseAvailable
from boot_agents.diffeo.library.deformations import twirl


class Twirl(SymbolicDiffeo):
    
    def __init__(self):
        SymbolicDiffeo.__init__(self, Topology.PLANE)
        
    def get_inverse(self):
        raise NoInverseAvailable()
   
    def apply(self, p):
        return twirl(p)
