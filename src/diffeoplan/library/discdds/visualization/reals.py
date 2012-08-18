from diffeoplan.library.discdds.visualization.statespace import DiffeoSystemStateSpace
import numpy as np
from contracts import contract
        
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
            
