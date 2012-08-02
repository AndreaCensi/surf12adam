from . import SquareDomain, diffeo_from_function_viewport, logger, np
from .. import DiffeoAction, DiffeoSystem
from boot_agents.diffeo import Diffeomorphism2D
from diffeoplan import DiffeoplanConfig


def DDSFromSymbolic(resolution, actions, topology=None):
    """ 
        Creates a synthetic example of a DDS from synthetic diffeomorphisms. 
    """  
    logger.info('Creating symbolic diffeomorphism (resolution = %d)' % 
                resolution)
    
    diffeoactions = []
    for i, a in enumerate(actions):
        logger.info('Getting symbolic diffeomorphism %r' % a)
        diffeo = DiffeoplanConfig.symdiffeos.instance(a) #@UndefinedVariable
        
        shape = (resolution, resolution)
        viewport = SquareDomain([[-1, +1], [-1, +1]])
        manifold = diffeo.get_topology()
        D, Dinfo = diffeo_from_function_viewport(diffeo, manifold, viewport, shape)    
        D2d = Diffeomorphism2D(D, Dinfo)
        
        diffeo_inv = diffeo.get_inverse()
        D_inv, Dinfo_inv = diffeo_from_function_viewport(diffeo_inv,
                                                          manifold, viewport, shape)    
        D2d_inv = Diffeomorphism2D(D_inv, Dinfo_inv) 

        original_cmd = np.array([i + 1])
        action = DiffeoAction(label=a,
                              diffeo=D2d,
                              diffeo_inv=D2d_inv,
                              original_cmd=original_cmd)
        diffeoactions.append(action)
        
    dds = DiffeoSystem('%s' % actions, actions=diffeoactions)
    return dds

