from PIL import Image #@UnresolvedImport
import pdb
import numpy as np
from roslib.msgs import EXT



'''
im.transform(size, AFFINE, data, filter) ⇒ image

Applies an affine transform to the image, and places the result in a new image with the given size.

Data is a 6-tuple (a, b, c, d, e, f) which contain the first two rows from an affine transform matrix. For each pixel (x, y) in the output image, the new value is taken from a position (a x + b y + c, d x + e y + f) in the input image, rounded to nearest pixel.

This function can be used to scale, translate, rotate, and shear the original image.
'''
# Load an image for testing
image = Image.open('rockys.png')

# Set the output size
ext_size = (200, 200)
h, w = ext_size

# Fit the affine parameters to a quadrilateral with the coorners c using matrix A
c1 = (1500, 500)
c2 = (1300, 700)
c3 = (1500, 900)
c4 = (1700, 700)

A = np.mat([[0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 1],
            [h, 0, 1, 0, 0, 0], [0, 0, 0, h, 0, 1],
            [h, w, 1, 0, 0, 0], [0, 0, 0, h, w, 1],
            [0, w, 1, 0, 0, 0], [0, 0, 0, 0, w, 1]])

b = np.mat(c1 + c2 + c3 + c4).T

# Solve for the parameters
data_mat = (A.T * A).I * A.T * b
data = np.array(data_mat.T)[0]
print(data)

# Transform the image
ext_image = image.transform(ext_size, Image.AFFINE, data)

# Show the transformed image
ext_image.show()
