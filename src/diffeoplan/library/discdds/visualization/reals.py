from diffeoplan.library.discdds.visualization.statespace import DiffeoSystemStateSpace
import numpy as np
from contracts import contract
from geometry.manifolds import SE2, se2
from geometry.poses import rotation_translation_from_SE2
        
class Reals(DiffeoSystemStateSpace):
    
    @contract(ndim='int,>=1', dt='>0')
    def __init__(self, ndim, dt=1):
        self.ndim = ndim
        self.dt = dt

    @contract(returns='array[N]')
    def default_state(self):
        return np.zeros(self.ndim)
    
    @contract(state='array[N]', command='array[N]')
    def integrate(self, state, command):
        assert state.size == self.ndim
        assert command.size == self.ndim
        return state + command * self.dt

    @contract(state='array[N]', returns='array[2]')
    def xy_from_state(self, state):
        if self.ndim == 1:
            return np.array([state[0], 0])
        else:
            return state[:2]
            
     
class EuclideanMotions(DiffeoSystemStateSpace):
    """ Visualization of Euclidean motions """
    
    @contract(dt='>0')
    def __init__(self, dt=1):
        self.dt = dt

    @contract(returns='SE2')
    def default_state(self):
        return SE2.identity()        
    
    @contract(state='SE2', command='array[3]')
    def integrate(self, state, command):
        v = se2.algebra_from_vector(command)
        delta = SE2.group_from_algebra(v * self.dt)
        return np.dot(state, delta)

    @contract(state='SE2', returns='array[2]')
    def xy_from_state(self, state):
        _, t = rotation_translation_from_SE2(state)
        return t
            
