from . import contract
from .. import UncertainImage
from boot_agents.diffeo import Diffeomorphism2D
from reprep import Report
import numpy as np
import numpy.linalg as la
import itertools
    
class DiffeoAction():
    """ 
        An "action" is described by a pair of diffeomorphism.
        
    """
    @contract(diffeo=Diffeomorphism2D,
              diffeo_inv=Diffeomorphism2D,
              label=str,
              original_cmd='None|array|list[>=1](array)')
    def __init__(self, label, diffeo, diffeo_inv, original_cmd, state=None):
        """
            :param label: A descriptive label for the command.
            :param diffeo: The diffeomorphism.
            :param diffeo_inv: The inverse of the diffeomorphism.
            :param original_cmd: A list of original commands (e.g. [[0,100],[0,100]])
            :param state: optional argument to tell which state the action was learned at.
        """
        self.label = label
        self.diffeo = diffeo
        self.diffeo_inv = diffeo_inv
        self.original_cmd = original_cmd
        self.state = state
    
    def __str__(self):
        return "DiffeoAction(%s, %s)" % (self.label, self.get_original_cmds())
    
    def inverse(self):
        """ Return the action with swapped diffeomorphisms. """
        # label = '(i%s)' % self.label
        # let's just use the same label
        label = self.label
        
        diffeo = self.diffeo_inv # <-- note swapped
        diffeo_inv = self.diffeo # <-- note swapped
        cmd = [-x for x in self.get_original_cmds()]
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
    
    @contract(returns=Diffeomorphism2D)
    def get_diffeo2d_forward(self):
        return self.diffeo
    
    @contract(returns=Diffeomorphism2D)
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
        if not hasattr(self, 'state'):
            state_str = 'None'
        else:
            state_str = str(self.state)
            
        report.text('summary', 'Label: %s\noriginal: %s\nState: %s' % 
                    (self.label, self.original_cmd, state_str))
        report.data('label', self.label)
        report.data('original_cmd', self.original_cmd)
        
        with report.subsection('forward') as s1:
            self.diffeo.display(s1)
             
        with report.subsection('backward') as s2:
            self.diffeo_inv.display(s2)
        
    @staticmethod
    def compose(a1, a2):
        label = '%s%s' % (a1.label, a2.label)
        # This is the correct order
        diffeo = Diffeomorphism2D.compose(a2.diffeo, a1.diffeo)
        diffeo_inv = Diffeomorphism2D.compose(a1.diffeo_inv, a2.diffeo_inv)
        #diffeo = Diffeomorphism2D.compose(a1.diffeo, a2.diffeo)
        #diffeo_inv = Diffeomorphism2D.compose(a2.diffeo_inv, a1.diffeo_inv)
        original_cmds = a1.get_original_cmds() + a2.get_original_cmds()
        return DiffeoAction(label, diffeo, diffeo_inv, original_cmds)
        
    def update_uncertainty(self):
        field = self.diffeo.d
        field_inv = self.diffeo_inv.d
        
        Y, X = np.meshgrid(range(field.shape[1]), range(field.shape[0]))
    
        D = np.zeros(X.shape + (2,))
        D[:, :, 0] = field[:, :, 0] - X
        D[:, :, 1] = field[:, :, 1] - Y
        
        D_inv = np.zeros(X.shape + (2,))
        D_inv[:, :, 0] = field_inv[:, :, 0] - X
        D_inv[:, :, 1] = field_inv[:, :, 1] - Y
    
        E = np.zeros(X.shape)
        E_inv = np.zeros(X.shape)
        
        for c in itertools.product(range(X.shape[0]), range(X.shape[1])):
            E[tuple(c)] = la.norm(D[c] + D_inv[tuple(D[c])]) / (1 + la.norm(D[c]))
            E_inv[tuple(c)] = la.norm(D_inv[c] + D[tuple(D[c])]) / (1 + la.norm(D_inv[c]))
        
#        
#        Eln = np.zeros(E.shape)
#        Eln_inv = np.zeros(E_inv.shape)
#        for c in itertools.product(range(X.shape[0]), range(X.shape[1])):
#            x0 = np.clip(c[0] - 1, 0, X.shape[0] - 1)
#            x1 = np.clip(c[0] + 1, 0, X.shape[0] - 1)
#            y0 = np.clip(c[1] - 1, 0, X.shape[1] - 1)
#            y1 = np.clip(c[1] + 1, 0, X.shape[1] - 1)
#            
#            Eln[tuple(c)] = np.std(E[x0:x1, y0:y1])
#            Eln_inv[tuple(c)] = np.std(E_inv[x0:x1, y0:y1])
#        
#        E = np.clip(E, 0, np.percentile(E, 99))
#        E_inv = np.clip(E_inv, 0, np.percentile(E_inv, 99))
#        
#        
#        self.diffeo.variance = 1.0 - (Eln > np.std(Eln)).astype('int')
#        self.diffeo_inv.variance = 1.0 - (Eln > np.std(Eln)).astype('int')
        
        self.diffeo.variance = 1 - E / np.max(E)
        self.diffeo_inv.variance = 1 - E_inv / np.max(E_inv)
