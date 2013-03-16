import numpy as np
import pdb
import pylab

d = 1.5
X = np.linspace(-3, 3, 100)
x = np.linspace(-3, 3, 7)
Xs = np.linspace(-3 + d, 3 + d, 100)
xs = np.linspace(-3 + d, 3 + d, 7)

def env(x):
    return 2 * x + np.sin(x)
pylab.figure()
pylab.subplot(2, 1, 1)
pylab.plot(X, env(X), color='b')
pylab.plot(X - d, env(X), color='g')
pylab.plot(x, env(x), marker='o', color='b')
pylab.plot(x - d, env(x), marker='o', color='g')
pylab.plot(x - d, env(x), marker='o', linestyle='none', color='r')
pylab.xlim((-3, 3))

pylab.subplot(2, 1, 2)

pylab.plot(x - d, np.abs(env(x)), marker='o', color='r')
pylab.plot(x, np.abs(env(x + d)), marker='o', color='g')
pylab.xlim((-3, 3))

pylab.show()
#pdb.set_trace()
