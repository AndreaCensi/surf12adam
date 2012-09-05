from . import (SquareDomain, diffeo_from_function_viewport, logger, np,
    get_current_config)
from .. import DiffeoAction, DiffeoSystem
from boot_agents.diffeo import Diffeomorphism2D

def DDSFromSymbolic(resolution, actions, topology=None): #@UnusedVariable
    """ 
        Creates a DiffeoSystem from synthetic diffeomorphisms. 
    """  
    config = get_current_config()
    
    logger.info('Creating symbolic diffeomorphism (resolution = %d)' % 
                resolution)
    
    diffeoactions = []
    for _, action in enumerate(actions):
        
        id_diffeo = action['diffeo']
        original_cmd = np.array(action['original_cmd'])
        logger.info('Getting symbolic diffeomorphism %r' % id_diffeo)
        
        diffeo = config.symdiffeos.instance(id_diffeo) 
        
        shape = (resolution, resolution)
        viewport = SquareDomain([[-1, +1], [-1, +1]])
        manifold = diffeo.get_topology()
        D, Dinfo = diffeo_from_function_viewport(diffeo, manifold, viewport, shape)    
        D2d = Diffeomorphism2D(D, Dinfo)
        
        diffeo_inv = diffeo.get_inverse()
        D_inv, Dinfo_inv = diffeo_from_function_viewport(diffeo_inv,
                                                          manifold, viewport, shape)    
        D2d_inv = Diffeomorphism2D(D_inv, Dinfo_inv) 

        action = DiffeoAction(label=id_diffeo,
                              diffeo=D2d,
                              diffeo_inv=D2d_inv,
                              original_cmd=original_cmd)
        diffeoactions.append(action)
        
    dds = DiffeoSystem('%s' % actions, actions=diffeoactions)
    return dds

