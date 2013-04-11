from contracts import contract
import numpy as np
from PIL import Image #@UnresolvedImport
from procgraph_pil.pil_operations import resize
from diffeoplan.library.logs.rosbag.bag_reader import get_image_array
#factor = 25

class Zoomer:
    
    @contract(min_zoom='>=100,x', max_zoom='>=x', use_zoom='bool')
    def __init__(self, min_zoom, max_zoom, use_zoom, output_size, zoom_factor):
        self.min_zoom = min_zoom
        self.max_zoom = max_zoom
        self.use_zoom = use_zoom
        self.output_size = output_size
        self.min_zoom = min_zoom
        self.max_zoom = max_zoom
        self.use_zoom = use_zoom
        self.queue_image = []
        self.queue_command = []
        self.factor = zoom_factor
        
        self.current_zoom = min_zoom
    
    def received_image(self, t, imgmsg):
        # first thing, convert to numpy
        image = get_image_array(imgmsg)
        if self.use_zoom:
            processed = zoom_image_center(image, self.current_zoom, self.output_size)
        else:
            processed = resize(image,
                               height=self.output_size[1],
                               width=self.output_size[0])
        
        self.queue_image.append((t, processed))
    
    def received_command(self, t, msg):
        
        command = msg.data
#        _, _, zoom = command
        zoom = command[2]

        if self.use_zoom or zoom == 0:
            
            next_zoom = self.current_zoom + zoom * self.factor 
            self.current_zoom = np.clip(next_zoom, self.min_zoom, self.max_zoom)
    
            if next_zoom != self.current_zoom:
                # We are clipping the zoom, so we ignore it
                pass # ..
            else:
                self.queue_command.append((t, msg))

    def get_image_queue(self):
        old = self.queue_image 
        self.queue_image = []
        return old
    
    def get_command_queue(self):
        old = self.queue_command 
        self.queue_command = []
        return old

@contract(image='array[HxWx3]', output_size='tuple(M,N)', zoom='>=100',
          returns='array[NxMx3]')
def zoom_image_center(image, zoom, output_size):
    """ Zoom an image with zoom center. """
    H, W, _ = image.shape
    pim = pil_from_np(image)
    
    z = float(zoom)
    z0 = 100.0 # original size zoom
    r = z0 / z
    assert r <= 1
    
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
    return Image.fromarray(a)    





