from boot_agents.diffeo.diffeomorphism2d import Diffeomorphism2D
from contracts import contract
from diffeoplan.library.discdds.diffeo_action import DiffeoAction
from diffeoplan.library.symdiffeo.square_domain import SquareDomain
from diffeoplan.library.symdiffeo.sym_diffeo import SymbolicDiffeo
from diffeoplan.library.symdiffeo.viewport import diffeo_from_function_viewport


@contract(symdiffeo=SymbolicDiffeo, shape='seq[2](int)',
          label='str', original_cmd='list(array[N])')
def diffeo_action_from_symdiffeo(symdiffeo, shape, label, original_cmd):
    """
        Creates a discrete action from a 
        
        :param symdiffeo: ID of symdiffeo or list of names.
    """
    viewport = SquareDomain([[-1, +1], [-1, +1]])
    manifold = symdiffeo.get_topology()
    D, Dinfo = diffeo_from_function_viewport(symdiffeo, manifold,
                                             viewport, shape)    
    D2d = Diffeomorphism2D(D, Dinfo)
    
    symdiffeo_inv = symdiffeo.get_inverse()
    D_inv, Dinfo_inv = diffeo_from_function_viewport(symdiffeo_inv, manifold,
                                                      viewport, shape)    
    D2d_inv = Diffeomorphism2D(D_inv, Dinfo_inv) 

    action = DiffeoAction(label=label,
                              diffeo=D2d,
                              diffeo_inv=D2d_inv,
                              original_cmd=original_cmd)
    
    return action


# def diffeo2d_from_symdiffeo():
