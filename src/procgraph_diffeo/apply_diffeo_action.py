from procgraph import Block
from contracts import contract
from diffeoplan.library.diffeo_action.from_symdiffeo import diffeo_action_from_symdiffeo
import numpy as np
from diffeoplan.configuration.master import get_dp_config
from diffeoplan.library.images.uncertain_image import UncertainImage
import contracts


class ApplyDiffeoAction(Block):
    ''' 
        Applies a diffeomorphism.
    
        The diffeo is created on the fly.
    '''

    Block.alias('apply_diffeo_action')

    Block.config('id_symdiffeo')
    
    Block.input('rgb', 'Input image (either gray or RGB)')
    Block.output('rgba', 'Output image')

    def init(self):
        self.diffeo_action = None

    @contract(shape='seq[2](>=1)')
    def init_diffeo(self, shape):
        id_symdiffeo = self.config.id_symdiffeo
        symdiffeo = get_dp_config().symdiffeos.instance(id_symdiffeo)
        label = 'tmp'
        original_cmd = [np.zeros(1)]
        self.info('creating diffeo_action')
        contracts.disable_all()
        self.diffeo_action = \
            diffeo_action_from_symdiffeo(symdiffeo, shape,
                                         label, original_cmd)
        self.info('..done')
        
    def update(self):
        rgb = self.input.rgb
        
        if self.diffeo_action is None:
            self.init_diffeo(shape=rgb.shape[:2])
        
        y0 = UncertainImage(rgb)
        y1 = self.diffeo_action.predict(y0)
        
        rgb1 = y1.get_rgba() 
        self.output.rgba = rgb1



