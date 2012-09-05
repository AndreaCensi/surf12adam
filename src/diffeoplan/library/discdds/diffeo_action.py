from . import contract
from .. import UncertainImage
from boot_agents.diffeo import Diffeomorphism2D
from reprep import Report
 
    
class DiffeoAction():
    """ 
        An "action" is a couple of diffeomoprhims
    """
    @contract(diffeo=Diffeomorphism2D,
              diffeo_inv=Diffeomorphism2D,
              label=str,
              original_cmd='None|array|list[>=1](array)')
    def __init__(self, label, diffeo, diffeo_inv, original_cmd):
        """
            :param label: A descriptive label for the command.
            :param diffeo: The diffeomorphism.
            :param diffeo_inv: The inverse of the diffeomorphism.
            :param original_cmd: A list of original commands (e.g. [[0,100],[0,100]])
        """
        self.label = label
        self.diffeo = diffeo
        self.diffeo_inv = diffeo_inv
        self.original_cmd = original_cmd
    
    def inverse(self):
        """ Return the action with swapped diffeomorphisms. """
        label = self.label + '_inv'
        diffeo = self.diffeo_inv # <-- note swapped
        diffeo_inv = self.diffeo # <-- note swapped
        cmd = -self.original_cmd # XXX
        return DiffeoAction(label, diffeo, diffeo_inv, cmd)
    
    def __sizeof__(self):
        """ Returns approximate size in bytes. """
        m = 0
        m += self.diffeo.__sizeof__()
        m += self.diffeo_inv.__sizeof__()
        return m

    @staticmethod
    def identity(label, shape, original_cmd):
        """ Constructs the identity action of the given shape. """
        diffeo = Diffeomorphism2D.identity(shape)
        diffeo_inv = diffeo
        return DiffeoAction(label, diffeo, diffeo_inv, original_cmd)
    
    def get_diffeo2d_forward(self):
        return self.diffeo
    
    def get_diffeo2d_backward(self):
        return self.diffeo_inv
    
    
    @contract(returns='list[>=1](array)')
    def get_original_cmds(self):
        """ Returns the commands as a list """
        if isinstance(self.original_cmd, list):
            return self.original_cmd
        else:
            return [self.original_cmd]
        
    @contract(y0=UncertainImage, returns=UncertainImage)
    def predict(self, y0, apply_function='self.diffeo.apply'):
        """ 
            Returns the prediction of applying this action to the 
            given input y0. 
        """
        apply_function = eval(apply_function)
        y1, var1 = apply_function(y0.get_values(),
                                     y0.scalar_uncertainty)
        return UncertainImage(y1, var1) 
        
    @contract(report=Report)
    def display(self, report, image=None): #@UnusedVariable
        report.data('label', self.label)
        report.data('original_cmd', self.original_cmd)
        
        s1 = report.section('forward')
        self.diffeo.display(s1) 
        s2 = report.section('backward')
        self.diffeo_inv.display(s2)
        
        # TODO: to finish
        
    @staticmethod
    def compose(a1, a2):
        label = '%s+%s' % (a1.label, a2.label)
        diffeo = Diffeomorphism2D.compose(a1.diffeo, a2.diffeo)
        # note the order is inverted
        diffeo_inv = Diffeomorphism2D.compose(a2.diffeo_inv, a1.diffeo_inv)
        # XXX: double check the order
        original_cmds = a1.get_original_cmds() + a2.get_original_cmds()
        return DiffeoAction(label, diffeo, diffeo_inv, original_cmds)
        
        
