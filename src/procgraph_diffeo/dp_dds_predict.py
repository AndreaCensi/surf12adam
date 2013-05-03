from procgraph import Block
from diffeoplan.configuration.master import get_dp_config
from diffeoplan.library.images.uncertain_image import UncertainImage
from procgraph_pil.pil_operations import resize

class DPDDSPredict(Block):
    Block.alias('dp_discdds_predict')
    
    Block.config('id_discdds')
    Block.config('plan')
    Block.config('config_dir', default=[])
    
    Block.input('rgb')
    Block.output('prediction')
    
    
    def init(self):
        id_discdds = self.config.id_discdds
        
        dp_config = get_dp_config()
    
        dp_config.load(self.config.config_dir)
        self.discdds = dp_config.discdds.instance(id_discdds)
        
        plan = self.config.plan
        self.action = self.discdds.plan2action(plan)
        
        
    def update(self):
        rgb0 = self.input.rgb
        H, W = self.discdds.get_shape()
        rgb = resize(rgb0, width=W, height=H)
        
        y0 = UncertainImage(rgb)
        y1 = self.action.predict(y0)
        pred = y1.get_rgba_fill()
        
        pred2 = resize(pred, height=rgb0.shape[0], width=rgb0.shape[1])
        self.output.prediction = pred2[:, :, :3]
    
