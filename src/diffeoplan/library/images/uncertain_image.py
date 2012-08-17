from . import contract, np
from PIL import Image #@UnresolvedImport
from boot_agents.diffeo.diffeo_estimator import scalaruncertainty2rgb

class UncertainImage():
    
    @contract(values='array[HxWx...]', scalar_uncertainty='None|array[HxW]')
    def __init__(self, values, scalar_uncertainty=None):
        """ 
            To avoid errors later, the values are stored as float values.
            (If we use uint8, then taking differences might introduce
             strange effects.)      
        """  
        self._values = values.astype('float32')
        self.scalar_uncertainty = scalar_uncertainty
    
    @contract(returns='array[HxW](float32)|array[HxWxN](float32)')
    def get_values(self):
        return self._values


    # TODO: we don't really need these, now that we have
    # a generic mechanism for instantiating images
    
    @staticmethod
    @contract(returns='dict(str:*)')
    def compute_all_distances(y0, y1):  # TODO: remove
        """ Computes all measures of difference between y0 and y1. 
            Returns a dictionary. """
        r = {}
        r['values_L2'] = UncertainImage.dist_values_L2(y0, y1)
        r['values_L1'] = UncertainImage.dist_values_L1(y0, y1)
        # add here
        return r
    
    @staticmethod
    def dist_values_L2(y0, y1): # TODO: remove
        diff = (y0.get_values() - y1.get_values()).flatten()
        return float(np.linalg.norm(diff, 2)) #/ y0.size

    @staticmethod
    def dist_values_L1(y0, y1): # TODO: remove
        diff = (y0.get_values() - y1.get_values()).flatten()
        return float(np.linalg.norm(diff, 1)) #/ y0.size


    def resize(self, size):
        # FIXME: this assumes that the image is a uint8 with 3 channels
        # we want to have float images fields
        y = np.array(Image.fromarray(self.get_values().astype('uint8')).resize(size))
        if self.scalar_uncertainty is not None:
            var = np.array(Image.fromarray(np.array(self.scalar_uncertainty*255).astype(np.uint8), 'L').resize(size)).astype(np.float)/255
        else:
            var = None
        return UncertainImage(y, var)

    @contract(returns='array[HxWx3](uint8)')
    def get_rgb(self):
        """ Returns an RGB representation of the certain part of the image. """
        # TODO: make it work with "float" data
        rgb = self.get_values().astype('uint8')        
        return rgb 


    @contract(returns='array[HxWx3](uint8)')
    def get_rgb_uncertain(self):
        """ Returns an RGB representation of the uncertainty of the image. """
        return scalaruncertainty2rgb(self.scalar_uncertainty)        
         
