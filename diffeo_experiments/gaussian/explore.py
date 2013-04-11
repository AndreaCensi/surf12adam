



import numpy as np
import numpy.random
import pylab
from scipy.linalg import toeplitz
import pdb

n = 100

e = np.random.ranf(n)

x = np.array(range(19))
ker = np.exp(-(x - 9) ** 2 / 50)
print ker
es = np.convolve(e, ker, mode='same')
smoothing = 5
A = np.matrix(1.0 / (smoothing + toeplitz(range(n / 2) + range(n / 2, 0, -1))))
#pdb.set_trace()

#pylab.show()

Y = -np.ones(n)
D = np.ones(n)
unevaled = range(n)
evaled = []



    

for _ in range(50):
    pmf = np.matrix(1.0 / (1 + D)) * A
    pmf = np.array(pmf).flatten()
    pmf = pmf - np.min(pmf)
    i = unevaled[randpmf(pmf[unevaled])]
#    pdb.set_trace()
    print('random selection: %g' % i)
    Y[i] = es[i]
    D[i] = es[i]
    
    evaled.append(i)
    unevaled.remove(i)
    
    mean = np.mean(D[evaled])
    D[unevaled] = mean
#    pdb.set_trace()
    
    pylab.figure()
    pylab.subplot(2, 1, 1)
    pylab.plot(es)
    pylab.plot(evaled, D[evaled], '.r')
    
    pylab.subplot(2, 1, 2)
    pylab.plot(pmf)
    print pmf
    
    pylab.show()
    
