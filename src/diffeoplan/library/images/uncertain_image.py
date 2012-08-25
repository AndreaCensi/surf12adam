from . import contract, np
from PIL import Image #@UnresolvedImport
from boot_agents.diffeo import scalaruncertainty2rgb

class UncertainImage():
    
    @contract(values='array[HxWx3](uint8)|array[HxWx...]((float32|float64),>=0,<=1)',
              scalar_uncertainty='None|array[HxW]')
    def __init__(self, values, scalar_uncertainty=None):
        """ 
            To avoid errors later, the values are stored as 0-1 float values.
            (If we use uint8, then taking differences might introduce
             strange effects.)      
            
            If the input is uint8, it will be rescaled in the 0-1 range.
            
        """  
        if values.dtype == 'uint8':
            values = values.astype('float32') / 255
            
        vmin = np.min(values)
        vmax = np.max(values)
        if not (vmin >= 0 and vmax <= 1):
            msg = 'Invalid bounds for image ([%s,%s])' % (vmin, vmax)
            raise ValueError(msg)

        self._values = values.astype('float32')
        
        if scalar_uncertainty is None:
            H, W = values.shape[:2]
            scalar_uncertainty = np.ones((H, W), dtype='float32')
            
        self.scalar_uncertainty = scalar_uncertainty
    
    @contract(returns='array[HxW](float32,>=0,<=1)|array[HxWxN](float32,>=0,<=1)')
    def get_values(self):
        return self._values

    def get_scalar_uncertainty(self):
        """ 1: sure; 0: unknown """
        return self.scalar_uncertainty
    
    def resize(self, size):
        # FIXME: this assumes that the image is a uint8 with 3 channels
        # we want to have float images fields
        rgb = self.get_rgb()
        rgb_resized = Image.fromarray(rgb).resize(size) # TODO: more general
        
        unc1 = np.array(self.scalar_uncertainty * 255).astype(np.uint8)
        resized = Image.fromarray(unc1, 'L').resize(size)
        var = resized.astype(np.float) / 255
        return UncertainImage(rgb_resized, var)

    @contract(returns='array[HxWx3](uint8)')
    def get_rgb(self):
        """ Returns an RGB representation of the certain part of the image. """
        # TODO: make it work with "float" data
        v = self.get_values()
        if v.ndim == 2:
            raise NotImplementedError
        rgb = (v / 255).astype('uint8')        
        return rgb 

    @contract(returns='array[HxWx3](uint8)')
    def get_rgb_uncertain(self):
        """ Returns an RGB representation of the uncertainty of the image. """
        return scalaruncertainty2rgb(self.scalar_uncertainty)        
         
    def __str__(self):
        return ("UncertainImage(%s;y in [%s,%s];u in [%s,%s])" %
                (self._values.shape, self._values.min(), self._values.max(),
                 self.scalar_uncertainty.min(), self.scalar_uncertainty.max()))
                 
