from contracts import contract
from learner.programs.diffeo_learner.bag_reader import get_image_array
import numpy as np
from PIL import Image #@UnresolvedImport
from procgraph_pil.pil_operations import resize

class Zoomer:
    
    @contract(min_zoom='>=100,x', max_zoom='>=x', use_zoom='bool')
    def __init__(self, min_zoom, max_zoom, use_zoom):
        self.min_zoom = min_zoom
        self.max_zoom = max_zoom
        self.use_zoom = use_zoom
        self.queue_image = []
        self.queue_command = []
        
        self.current_zoom = min_zoom
    
    def received_image(self, t, imgmsg):
        # first thing, convert to numpy
        image = get_image_array(imgmsg)
        if self.use_zoom:
            processed = resize(image,
                               height=self.output_size[1],
                               width=self.output_size[0])
        else:
            processed = zoom_image_center(image, self.current_zoom, self.output_size)
        
        self.queue_image.append((t, processed))
    
    def received_command(self, t, msg):
        command = msg.data
        _, _, zoom = command

        next_zoom = self.current_zoom + zoom 
        self.current_zoom = np.clip(next_zoom, self.min_zoom, self.max_zoom)
        if next_zoom != self.current_zoom:
            # We are clipping the zoom, so we ignore it
            pass # ..
        else:
            self.queue_command.append((t, command)) 

    def get_image_queue(self):
        old = self.queue_image 
        self.queue_image = []
        return old
    
    def get_command_queue(self):
        old = self.queue_command 
        self.queue_command = []
        return old

                

@contract(rgb='array[HxWx3]', zoom='>=100', output_size='tuple(M,N)',
          returns='array[MxNx3]')
def zoom_image_center(image, zoom, output_size):
    """ Zoom an image with zoom center. """
    H, W, _ = image.shape
    pim = pil_from_np(image)
    
    z = float(zoom)
    z0 = 100.0 # original size zoom
    r = z0 / z
    assert r >= 1
    
    x0 = int(W / 2.0 * (1.0 - r)) 
    y0 = int(H / 2.0 * (1.0 - r)) 
    x1 = int(W / 2.0 * (1.0 + r)) 
    y1 = int(H / 2.0 * (1.0 + r)) 
    cropped = pim.crop((x0, y0, x1, y1))
    
    W1, H1 = cropped.size
    
    assert W1 == (x1 - x0)
    assert H1 == (y1 - y0)
    
    pim_out = cropped.resize(output_size)    

    assert pim_out.size == output_size
    
    return np_from_pil(pim_out)


def np_from_pil(image):
    return np.asarray(image.convert("RGB"))

def pil_from_np(a):
    return Image.from_array(a)    





