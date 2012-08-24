from . import DiffeoAction, np, contract, UncertainImage, logger
from reprep import Report
from diffeoplan.utils.matrices import construct_matrix
from diffeoplan.library.discdds.diffeo_action import diffeoaction_comm_distance_L2_infow, \
    diffeoaction_comm_distance_L2, diffeoaction_anti_distance_L2_infow, \
    diffeoaction_anti_distance_L2, diffeoaction_distance_L2_infow, \
    diffeoaction_distance_L2

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
    
    @contract(plan='seq[>=1](int)', returns=DiffeoAction)
    def plan2action(self, plan):
        """ Creates the DiffeoAction for the given composition. """
        a = self.actions[plan[0]]
        for i in plan:
            a_i = self.actions[i]
            a = DiffeoAction.compose(a, a_i) # XXX: check
        return a
    
    
    @contract(us='seq[N](array)', returns='list[N](int)')
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
        if False:
            raise ValueError(msg)
        else:
            logger.error(msg)
            logger.error('I will continue just because Andrea needs to debug '
                         'other code. I will return the command 0.')
            return 0
    
    @contract(returns='list[N](array)', plan='seq[N](int,>=0)')
    def indices_to_commands(self, plan):
        """ Converts from indices to the original commands. """
        return [self.actions[x].original_cmd for x in plan]
  
    @contract(report=Report, image=UncertainImage)
    def display(self, report, image):
        '''
            Displays this diffeo system in a report.
        
            :param report: instance of reprep.Report to fill.
            :param image: RGB image to use as test.
        '''
        
        overview = 'Displaying a discrete DDS with %d actions' % len(self.actions)
        report.text('overview', overview)
    
        for i, action in enumerate(self.actions):
            sec = report.section('action%d' % i)
            action.display(report=sec, image=image)
            
    @contract(returns='array[NxN](>=0)')
    def actions_distance(self, distance):
        def entries(i, j):
            a1 = self.actions[i]
            a2 = self.actions[j]
            return distance(a1, a2)
        K = len(self.actions)
        return construct_matrix((K, K), entries)
        
    def actions_distance_L2(self):
        return self.actions_distance(diffeoaction_distance_L2)
    
    def actions_distance_L2_infow(self):
        return self.actions_distance(diffeoaction_distance_L2_infow)
    
    def actions_anti_distance_L2(self):
        return self.actions_distance(diffeoaction_anti_distance_L2)

    def actions_anti_distance_L2_infow(self):
        return self.actions_distance(diffeoaction_anti_distance_L2_infow)
        
    def actions_comm_distance_L2(self):
        return self.actions_distance(diffeoaction_comm_distance_L2)
        
    def actions_comm_distance_L2_infow(self):
        return self.actions_distance(diffeoaction_comm_distance_L2_infow)
    
