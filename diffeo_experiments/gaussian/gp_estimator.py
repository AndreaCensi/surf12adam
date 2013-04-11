

import numpy as np
import numpy.random
import pdb
import pylab
from sklearn.gaussian_process.gaussian_process import GaussianProcess


d = np.concatenate([np.random.randint(2, 4, size=20), np.random.randint(-4, 4, size=8)])
print(d.tolist())

sigmaf = 10
sigman = 10
l = 1

def k(x, y):
    return sigmaf ** 2 * np.exp(-(x - y) ** 2 / (2 * l ** 2))
    


n = len(d)
K = np.zeros((n, n))
for i in range(n):
    for j in range(n):
#        print [i, j]
        K[i, j] = k(i, j)
        if i == j:
            K[i, j] += sigman ** 2

K = np.mat(K)
#pdb.set_trace()

Y = K * K.I * np.mat(d).T
print np.array(Y.T).reshape(Y.size).astype(int).tolist()

#pdb.set_trace()

pylab.figure()
pylab.hold(True)
pylab.plot(d, 'r*')
#pylab.plot(Y, 'bo')

def plot_gp_pred(sigma, **fillargs):
#    pdb.set_trace()
    nugget = (sigma ** 2 / (0.1 + d.astype('float') ** 2))
    gp = GaussianProcess(corr='squared_exponential', nugget=nugget)
    gp.fit(np.atleast_2d(range(n)).T, np.atleast_2d(d).T)
    x = np.atleast_2d(np.linspace(0, n - 1)).T
    y_pred, MSE = gp.predict(x, eval_MSE=True)
    pylab.plot(x, y_pred)
    pylab.fill_between(x.T[0], y_pred + MSE, y_pred - MSE, **fillargs)
#plot_gp_pred(1, color='r')
#plot_gp_pred(2, color='g')

def smooth_vector(vector, ker=[.5, 0, .5]):
    d_s = np.convolve(vector, ker, 'same')
    pylab.plot(d_s)
    return d_s
gaus = np.exp(-np.array([-1, 0, 1, 2, 3]) ** 2) + np.exp(-np.array([-3, -2, -1, 0, 1]) ** 2)
smooth_vector(d, ker=gaus / np.sum(gaus))
#smooth_vector(smooth_vector(d))

pylab.show()
