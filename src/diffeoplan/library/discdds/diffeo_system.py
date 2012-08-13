from . import DiffeoAction, np, contract, UncertainImage

class DiffeoSystem():
    """
        A DiffeoSystem is a set of discretized diffeomorphisms.
        
    """
    @contract(label='str', actions='list[>=1]')
    def __init__(self, label, actions):
        """
            :param label: A descriptive label for the system.
            :param actions: a list of DiffeoAction objects.
        """
        self.label = label
        self.actions = actions
        
        for a in actions:
            # TODO: check that the discretization is the same
            assert isinstance(a, DiffeoAction)

    @contract(returns='tuple(int,int)')
    def get_shape(self):
        """ Returns the resolution of these diffeomorphisms. """
        return self.actions[0].diffeo.get_shape()

    @contract(plan_length='K,>=1', returns='list[K](int)')
    def get_random_plan(self, plan_length):
        """ 
            Returns a random plan of the given length 
            --- a sequence of integers 
            each corresponding to one command.
        """
        n = len(self.actions)
        if n == 1:
            rplan = np.zeros((plan_length,), 'int')
        else:
            rplan = np.random.randint(low=0, high=(n - 1), size=plan_length)
        return rplan.tolist() 

    @contract(plan='list[>=1](int)', y0=UncertainImage, returns=UncertainImage)
    def predict(self, y0, plan):
        """ 
            Predicts the result of applying the given plan to the image. 
        """
        # TODO: check that the integers have correct range
        y1 = y0
        for p in plan:
            action = self.actions[p]
            y1 = action.predict(y1)
        return y1
    
    @contract(us='list[N](array)', returns='list[N](int)')
    def commands_to_indices(self, us):
        """ Given the sequence of commands (e.g. [[0,0,+100], [0,100,0],...]),
            return the corresponding indices. """
        return [self.command_to_index(u) for u in us]
    
    @contract(u='array[K]', returns='int,>=0')
    def command_to_index(self, u):
        """ Return the index for a given command. """
        for i, action in enumerate(self.actions):
            same = np.all(action.original_cmd == u)
            if same:
                return i
        msg = 'Could not find action corresponding to command %s.\n' % u
        msg += 'This DiffeoSystem has %s' % [a.original_cmd for a in self.actions]
        raise ValueError(msg)
    
