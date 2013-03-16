import pylab
import numpy as np
import numpy.random
import pdb
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
self = np.random.randint(0, 255, (40, 30))
#self = np.array([[1, 2, 3, 4, 5, 6], [5, 6, 7, 8, 9, 10], [8, 9, 10, 11, 12, 13] , [10, 11, 12, 13, 14, 15]])
f = pylab.figure()
pylab.xlim((0, 60))
pylab.ylim((60, 0))
pylab.imshow(self, interpolation='nearest', origin='upper')
pylab.arrow(0, 0, 50, 50, color='lightgray')
ax = pylab.subplot(111)
ax.xaxis.set_major_locator(MultipleLocator(self.shape[1] / 10))
ax.xaxis.set_minor_locator(MultipleLocator(1))
ax.yaxis.set_major_locator(MultipleLocator(self.shape[0] / 10))
ax.yaxis.set_minor_locator(MultipleLocator(1))
pylab.grid(True, 'major', linewidth=1, alpha=0.5, linestyle='solid')
pylab.grid(True, 'minor', linewidth=.2, alpha=0.5, linestyle='solid')
pylab.show()


