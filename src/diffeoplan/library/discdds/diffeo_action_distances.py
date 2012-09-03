from . import DiffeoAction, contract
from boot_agents.diffeo import Diffeomorphism2D

@contract(a1=DiffeoAction, a2=DiffeoAction)
def diffeoaction_distance(a1, a2, diffeo_distance):
    """ 
        Returns the distance between two DiffeoActions
        as the average of the L2 distance between
        forward and backward diffeomorphism.
    """
    # Note:  forward, forward
    d = diffeo_distance(a1.diffeo, a2.diffeo)
    # Note: backward, backward
    d_inv = diffeo_distance(a1.diffeo_inv, a2.diffeo_inv)
    return 0.5 * d + 0.5 * d_inv

@contract(a1=DiffeoAction, a2=DiffeoAction)
def diffeoaction_distance_L2(a1, a2):
    return diffeoaction_distance(a1, a2, Diffeomorphism2D.distance_L2)

@contract(a1=DiffeoAction, a2=DiffeoAction)
def diffeoaction_distance_L2_infow(a1, a2):
    return diffeoaction_distance(a1, a2, Diffeomorphism2D.distance_L2_infow)

@contract(a1=DiffeoAction, a2=DiffeoAction)
def diffeoaction_distance_L2_infow_scaled(a1, a2):
    return diffeoaction_distance(a1, a2, Diffeomorphism2D.distance_L2_infow_scaled)


@contract(a1=DiffeoAction, a2=DiffeoAction)
def diffeoaction_anti_distance(a1, a2, diffeo_distance):
    """ 
        Returns the anti-distance between two DiffeoActions
        as the average of the L2 anti-distances.
    """
    # Note: forward, backward
    d = diffeo_distance(a1.diffeo, a2.diffeo_inv)
    # Note: backward, forward 
    d_inv = diffeo_distance(a1.diffeo_inv, a2.diffeo)
    return 0.5 * d + 0.5 * d_inv

@contract(a1=DiffeoAction, a2=DiffeoAction)
def diffeoaction_anti_distance_L2(a1, a2):
    return diffeoaction_anti_distance(a1, a2, Diffeomorphism2D.distance_L2)

@contract(a1=DiffeoAction, a2=DiffeoAction)
def diffeoaction_anti_distance_L2_infow(a1, a2):
    return diffeoaction_anti_distance(a1, a2, Diffeomorphism2D.distance_L2_infow)

@contract(a1=DiffeoAction, a2=DiffeoAction)
def diffeoaction_comm_distance(a1, a2, action_distance):
    """ 
        Returns the commutation-distance between two DiffeoActions
        which is d(a1a2, a2a1).
    """
    a1a2 = DiffeoAction.compose(a1, a2)
    a2a1 = DiffeoAction.compose(a2, a1)
    return action_distance(a1a2, a2a1)

@contract(a1=DiffeoAction, a2=DiffeoAction)
def diffeoaction_comm_distance_L2(a1, a2):
    return diffeoaction_comm_distance(a1, a2, diffeoaction_distance_L2)

@contract(a1=DiffeoAction, a2=DiffeoAction)
def diffeoaction_comm_distance_L2_infow(a1, a2):
    return diffeoaction_comm_distance(a1, a2, diffeoaction_distance_L2_infow)

        
