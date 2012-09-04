import numpy as np
from contracts import contract


@contract(image1='array', image2='array')
def image_distance_L1(image1, image2):
    """
     Compares two images (/sensor_msgs/Image) and estimates first norm 
     of the difference between the pixel values by calculating the 
     average difference for the set of each <step>'th pixel.
     
     Output
         Average pixel value difference
    """
    im1 = image1.astype('float32')
    im2 = image2.astype('float32')
    return np.mean(np.abs(im1 - im2)) 
