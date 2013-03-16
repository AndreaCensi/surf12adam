import pylab
import numpy as np
from mpl_toolkits.mplot3d import axes3d
from matplotlib.patches import Arrow
from PIL import Image #@UnresolvedImport
import pdb
X, Y = np.meshgrid(np.linspace(-5, 5, 5), np.linspace(-5, 5, 5))

U = -Y
V = -X


def quiver_image(X, Y, U, V):
    pylab.figure(1)
    pylab.quiver(X, Y, U, V)
    canvas = pylab.get_current_fig_manager().canvas
    canvas.draw()
    pil_image = Image.fromstring('RGB', canvas.get_width_height(), canvas.tostring_rgb())
    return pil_image

def arrow3d(x, y, u, v, zdir='z'):
    head = []

quiv = quiver_image(X, Y, U, V)

pylab.figure(0)
pylab.gca(projection='3d')
#pylab.contourf(X, Y, U + V, offset=10, alpha=0.5)
#pylab.contourf(X, Y, U * V, offset=0, alpha=0.5)
pylab.imshow(quiv)

pylab.show()
#pdb.set_trace()

