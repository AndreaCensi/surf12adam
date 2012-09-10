from . import SymbolicDiffeo
from .. import get_current_config
from contracts import contract
import geometry
import numpy as np


def rotdeg(p, deg):
    R = geometry.rot2d(np.deg2rad(deg))
    return np.dot(R, p)


class EvalDiffeo(SymbolicDiffeo):

    @contract(function='str', inverse='str', topology='str')
    def __init__(self, topology, function, inverse):
        self.topology_s = topology
        SymbolicDiffeo.__init__(self, topology)
        self.function = function
        self.inverse = inverse
            
    def get_inverse(self):
        return EvalDiffeo(topology=self.topology_s,
                          function=self.inverse,
                          inverse=self.function)
    
    @contract(point='array[2]', returns='array[2]')
    def apply(self, point):
        # These are the symbols that can be used 
        p = point#@UnusedVariable
        x = point[0]#@UnusedVariable
        y = point[1]#@UnusedVariable
        dot = np.dot #@UnusedVariable
        res = eval(self.function)
        res = np.array(res)
        return res
    
    def __repr__(self):
        return ("EvalDiffeo(%s,%s,%s)" % 
                 (self.function, self.inverse, self.topology_s))
    

class SymDiffeoComposition(SymbolicDiffeo):
    
    @contract(chain='list[>=1]')
    def __init__(self, chain):
        self.chain = chain
        # TODO: check same topology
        SymbolicDiffeo.__init__(self, chain[0].topology_s)

    def get_inverse(self):
        chain = [x.get_inverse() for x in self.chain[::-1]]
        return SymDiffeoComposition(chain)
    
    def apply(self, point):
        for d in self.chain:
            point = d.apply(point)
        return point
        
    def __repr__(self):
        return ("Comp(%s)" % self.chain)

    
def make_chain(diffeos):
    symdiffeos = get_current_config().symdiffeos
    chain = map(symdiffeos.instance, diffeos) 
    return SymDiffeoComposition(chain)     

def make_inverse(id_diffeo):
    symdiffeos = get_current_config().symdiffeos 
    return symdiffeos.instance(id_diffeo).get_inverse()     


@contract(times='>=1')
def repeat(id_diffeo, times):
    symdiffeos = get_current_config().symdiffeos 
    diffeo = symdiffeos.instance(id_diffeo)
    chain = [diffeo] * times
    return SymDiffeoComposition(chain)
    
    
    

