from . import contract, DiffeoSystemStateSpace, logger
from diffeoplan.library.discdds.visualization.reals import Reals


@contract(returns=DiffeoSystemStateSpace)
def guess_state_space(id_discdds, dds):
    """ Returns the proper DiffeoSystemStateSpace for the given
        dynamics. Hackish -- but remember this is just for visualization. """
    
#    if id_discdds == 'dpx1':
#        return Reals(1)
#    
    msg = 'Using default state space for %r.' % id_discdds
    logger.debug(msg)
    ndim = dds.actions[0].original_cmd.size
    return Reals(ndim)
          
        
