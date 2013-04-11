from . import contract, np
from boot_agents.diffeo import scalaruncertainty2rgb
from diffeoplan.utils import resample_signal
from diffeoplan.utils import resample_signal_2d
#from numpy.core.numeric import dtype

class UncertainImage(object):
    
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

        self._values = values.astype('float32').copy()
        
        if scalar_uncertainty is None:
            H, W = values.shape[:2]
            scalar_uncertainty = np.ones((H, W), dtype='float32')
            
        self.scalar_uncertainty = scalar_uncertainty.copy()
    
        # Make them immutable
        self._values.setflags(write=False)
        self.scalar_uncertainty.setflags(write=False)
    
    def __sizeof__(self):
        """ Returns approximate memory usage in bytes. """
        def sizeof_array(a):
            return a.size * a.itemsize
        m = 0
        m += sizeof_array(self._values)
        m += sizeof_array(self.scalar_uncertainty)
        return m

    @contract(returns='array[HxW](float32,>=0,<=1)|array[HxWxN](float32,>=0,<=1)')
    def get_values(self):
        return self._values

    def get_scalar_uncertainty(self):
        """ 1: sure; 0: unknown """
        return self.scalar_uncertainty
    
    @contract(size='seq[2](int)')
    def resize(self, size):
        values2 = resample_signal(self.get_values(), tuple(size))
        unc2 = resample_signal_2d(self.get_scalar_uncertainty(), tuple(size), mode='F')
        return UncertainImage(values2, unc2)

    def crop(self, top, right, bottom, left):
        shape = self._values.shape[:2]
        stop = int(top * shape[0])
        sbottom = int(bottom * shape[0])
        sright = int(right * shape[1])
        sleft = int(left * shape[1]) 

        def cropit(x):
            return x[sleft:-sright, stop:-sbottom]

        values2 = cropit(self.get_values())
        unc2 = cropit(self.get_scalar_uncertainty())
        return UncertainImage(values2, unc2)

    @contract(returns='array[HxWx3](uint8)')
    def get_rgb(self):
        """ Returns an RGB representation of the certain part of the image. """
        # TODO: make it work with "float" data
        v = self.get_values()
        if v.ndim == 2:
            raise NotImplementedError
        rgb = (v * 255).astype('uint8')        
        
        # set to gray parts where the certainty is 0
        w = self.get_scalar_uncertainty() == 0
        rgb[:, :, 0][w] = 125
        rgb[:, :, 1][w] = 125
        rgb[:, :, 2][w] = 125
        
        return rgb
    
    
    @contract(returns='array[HxWx4](uint8)')
    def get_rgba(self):
        """ Returns an RGBalpha representation of the image, where uncertain is transparent. """
        # TODO: make it work with "float" data
        v = self.get_values()
        if v.ndim == 2:
            raise NotImplementedError
        rgb = (v * 255).astype('uint8')        
        rgba = np.zeros((v.shape[0], v.shape[1], 4), dtype=np.uint8)
        rgba[:, :, :3] = rgb
        
        
        w = self.get_scalar_uncertainty()
        rgba[:, :, 3] = (w * 255).astype('uint8')
#        rgb[:, :, 0][w] = 125
#        rgb[:, :, 1][w] = 125
#        rgb[:, :, 2][w] = 125
        
        return rgba

    @contract(returns='array[HxWx3](uint8)')
    def get_rgb_uncertain(self):
        """ Returns an RGB representation of the uncertainty of the image. """
        return scalaruncertainty2rgb(self.scalar_uncertainty)        
         
    def __str__(self):
        return ("UncertainImage(%s;y in [%s,%s];u in [%s,%s])" % 
                (self._values.shape, self._values.min(), self._values.max(),
                 self.scalar_uncertainty.min(), self.scalar_uncertainty.max()))
                 
