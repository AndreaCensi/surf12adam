from .. import UncertainImage
from . import  contract
from boot_agents.diffeo.diffeo_estimator import Diffeomorphism2D
from reprep import Report

    
class DiffeoAction():
    """ 
        An "action" is a couple of diffeomoprhims
    """
    @contract(diffeo=Diffeomorphism2D,
              diffeo_inv=Diffeomorphism2D,
              label=str,
              original_cmd='None|array')
    def __init__(self, label, diffeo, diffeo_inv, original_cmd):
        """
            :param label: A descriptive label for the command.
            :param diffeo: The diffeomorphism.
            :param diffeo_inv: The inverse of the diffeomorphism.
            :param original_cmd: The original command (e.g. [0,100])
        """
        self.label = label
        self.diffeo = diffeo
        self.diffeo_inv = diffeo_inv
        self.original_cmd = original_cmd
    
    @contract(y0=UncertainImage, returns=UncertainImage)
    def predict(self, y0):
        """ 
            Returns the prediction of applying this action to the 
            given input y0. 
        """
        y1, var1 = self.diffeo.apply(y0.get_values(),
                                     y0.scalar_uncertainty)
        return UncertainImage(y1, var1) 
        
    @contract(report=Report, image=UncertainImage)
    def display(self, report, image):
        report.data('label', self.label)
        report.data('original_cmd', self.original_cmd)
        
        # TO finish
        
        
