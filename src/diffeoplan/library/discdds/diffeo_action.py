from .. import UncertainImage
from . import  contract
from boot_agents.diffeo.diffeo_estimator import Diffeomorphism2D
from reprep import Report
from boot_agents.diffeo.diffeo_basic import diffeo_distance_L2

    
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
        
    @contract(report=Report, image=UncertainImage)
    def display(self, report, image):
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
        
    @staticmethod
    def distance_L2_mixed(a1, a2):
        """ 
            Returns the distance between two DiffeoActions
            as the average of the L2 distance between
            forward and backward diffeomorphism.
        """
        assert isinstance(a1, DiffeoAction)
        assert isinstance(a2, DiffeoAction)
        # TODO: does not use uncertainty
        # TODO: check it is similar to last year's code
        # Note:  forward, forward
        d = diffeo_distance_L2(a1.diffeo.d, a2.diffeo.d)
        # Note: backward, backward
        d_inv = diffeo_distance_L2(a1.diffeo_inv.d, a2.diffeo_inv.d)
        return 0.5 * d + 0.5 * d_inv
        

    @staticmethod
    def anti_distance_L2_mixed(a1, a2):
        """ 
            Returns the anti-distance between two DiffeoActions
            as the average of the L2 anti-distances.
        """
        assert isinstance(a1, DiffeoAction)
        assert isinstance(a2, DiffeoAction) 
        # Note: forward, backward
        d = diffeo_distance_L2(a1.diffeo.d, a2.diffeo_inv.d)
        # Note: backward, forward 
        d_inv = diffeo_distance_L2(a1.diffeo_inv.d, a2.diffeo.d)
        return 0.5 * d + 0.5 * d_inv
    
    @staticmethod
    def comm_distance_L2_mixed(a1, a2):
        """ 
            Returns the commutation-distance between two DiffeoActions
            which is d(a1a2, a2a1).
        """
        assert isinstance(a1, DiffeoAction)
        assert isinstance(a2, DiffeoAction)
        a1a2 = DiffeoAction.compose(a1, a2)
        a2a1 = DiffeoAction.compose(a2, a1)
        return DiffeoAction.distance_L2_mixed(a1a2, a2a1)
    
