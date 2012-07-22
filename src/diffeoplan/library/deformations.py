from boot_agents.diffeo.library.deformations import twirl
from diffeoplan.library.sym_diffeo import (SymbolicDiffeo, Topology,
    NoInverseAvailable)


class Twirl(SymbolicDiffeo):
    
    def __init__(self):
        SymbolicDiffeo.__init__(self, Topology.PLANE)
        
    def get_inverse(self):
        raise NoInverseAvailable()
   
    def apply(self, p):
        return twirl(p)
