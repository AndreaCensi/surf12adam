from contracts import contract
from abc import ABCMeta, abstractmethod

class DiffeoSystemStateSpace:
    """ 
        This class if for visualization/debug only.
        It integrates what would be "uninterpreted" commands
        to get a "state", and provides functions to plot
        those states (i.e. embedding in R^2).
    """
    __metaclass__ = ABCMeta
    
    def __init__(self):
        pass
    
    @abstractmethod
    @contract(returns='array')
    def default_state(self):
        """ Returns the default state. """    
    
    @abstractmethod
    @contract(command='seq(array)', returns='array')
    def integrate(self, state, command):
        """ Integrates the dynamics from the state. """
    
    @contract(commands='seq(array)', returns='array')
    def state_from_commands(self, commands, start=None):
        """ 
            Returns the state corresponding to a sequence of commands,
            starting from the identity. 
        """
        if start is None:
            start = self.default_state()
            
        state = start
        for c in commands:
            state = self.integrate(state, c)
        return state
        
    @abstractmethod
    def xy_from_state(self, state):
        """ Convert a state to a 2D point representation. """
        
  
