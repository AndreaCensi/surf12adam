from contracts import contract
from diffeoplan.configuration.master import get_dp_config
from diffeoplan.library.images import UncertainImage
from procgraph import Block
from procgraph_pil import resize
import itertools
import numpy as np
from procgraph_images.imggrid import make_images_grid
import contracts


class DPDDSPredictMatrix(Block):
    Block.alias('dp_discdds_predict_matrix')
    
    Block.config('id_discdds')
    Block.config('config_dir', default=[])

    Block.config('order', default=[0, 1, 2, 3])
    Block.config('nsteps', default=2)
    Block.config('mult', default=6)
    Block.config('pad', default=2)
    
    Block.input('rgb')
    Block.output('prediction')
    
    def init(self):
        contracts.disable_all()
        id_discdds = self.config.id_discdds
        
        dp_config = get_dp_config()
    
        dp_config.load(self.config.config_dir)
        self.discdds = dp_config.discdds.instance(id_discdds)
        
        N = self.config.nsteps
        m = make_matrix(nsteps=N, mult=self.config.mult)
        
        cmd_to_action = {'a': 3, 'b': 2, 'c': 1, 'd': 0}
        cmd_inv = {'a': 'c', 'b': 'd'}
        
        mult = self.config.mult
        sequence = {0: 0, 1: 1 * mult, 2: 2 * mult, 3: 3 * mult}
        
        @contract(ins='tuple(int,str)', returns='list[int]')
        def normalize_instruction(ins):
            num, cmd = ins
            if num < 0:
                cmd = cmd_inv[cmd]
                num = -num 
            assert num >= 0
            k = cmd_to_action[cmd]
            n = sequence[num]
            return [k] * n
        
        @contract(splan='list[P](tuple(int,str))', returns='list(int)')
        def normalize_splan(splan):
            plan = []
            for ins in splan:
                plan.extend(normalize_instruction(ins))
            return plan
        
        M = 2 * N + 1
        self.actions = []
        for i, j in itertools.product(range(M), range(M)):
            m[i][j] = normalize_splan(m[i][j])
            action = self.discdds.plan2action(m[i][j])
            self.actions.append(action)
        
        self.info('shape: %s' % str(self.discdds.get_shape()))
        self.M = M
        
    def update(self):
        rgb0 = self.input.rgb
        pad = self.config.pad
        
        H, W = self.discdds.get_shape()
        rgb = resize(rgb0, width=W, height=H)
        
        y0 = UncertainImage(rgb)
        
        uimages = [a.predict(y0) for a in self.actions]
        
        images = [x.get_rgba_fill()[..., :3] for x in uimages]
        images = [resize(x, height=rgb0.shape[0], width=rgb0.shape[1]) 
                  for x in images]
        canvas = make_images_grid(images, pad=pad)
        
        self.output.prediction = canvas
    


@contract(nsteps='int,>=1,N', mult='K,int,>=1',
          returns='list[2*N+1](list[2*N+1](list(tuple(int, str))))')
def make_matrix(nsteps, mult):
    steps = np.array(range(-nsteps, +nsteps + 1))
    assert -steps[0] == steps[-1]
    
    def get_plan(u, v):
        return [(u, 'a')] * mult + [(v, 'b')] * mult
    
    res = []
    for i in steps:
        row = []
        for j in steps:
            plan = get_plan(i, j)
            row.append(plan)
        res.append(row)
    return res



