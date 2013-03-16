

import numpy as np
import numpy.random
import pdb
import pylab




def get_gp_function():
    r_coef = np.random.randn(3)
    def f(X):
        if X.__class__ in [tuple, list, np.ndarray]:
            return np.array([x * r_coef[0] + np.sin(r_coef[1] * x) + r_coef[2] for x in X])
        else:
            return X * r_coef[0] + np.sin(r_coef[1] * X) + r_coef[2]
    return f

def fplot(f, lim, n=100):
    x = np.linspace(lim[0], lim[1], n);
    y = f(x)
    pylab.plot(x, y)

f = get_gp_function()
print f(0)
print f(1)
print f(0)
pylab.figure()
pylab.hold(True)
fplot(f, [0, 10])

n = 250

Y0 = np.zeros(n)
Y5 = np.zeros(n)
Y10 = np.zeros(n)

for i in range(n):
    f = get_gp_function()
    Y0[i] = f(0)
    Y5[i] = f(5)
    Y10[i] = f(10)
    fplot(f, [0, 10])
    
pylab.figure()
pylab.subplot(1, 3, 1)
pylab.hist(Y0)
pylab.subplot(1, 3, 2)
pylab.hist(Y5)
pylab.subplot(1, 3, 3)
pylab.hist(Y10)

pylab.show()
