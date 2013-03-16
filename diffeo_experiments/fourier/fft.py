import numpy as np
import numpy.linalg as la
import numpy.random
import pdb
import math
import scipy
import pylab

'''
init vars
'''

n = 20
x = np.array(range(n)).astype('float')
a = np.sin(x * 2.0 / 18.0 * np.pi) + np.random.ranf(n)

# Define a imaginary unit
j = complex(0, 1)


print('Original image:     ' + str(a.tolist()))

# Setup basis vectors with original size
fk = [float(k) / n for k in range(n / 2)] + [float(k) / n - 1 for k in range(n / 2, n)] 
M = np.array([scipy.exp(j * 2 * np.pi * fk[k] * np.array(x)) for k in range(n)])

# Calculate Fourier coefficients 
S = np.fft.fft(a)
print(S)
#pdb.set_trace()
# Recreate image
y = np.array(np.mat(M).T * np.mat(S).T / n).reshape(n)

# Calculate residual
r = y - a

pylab.figure()
pylab.subplot(2, 1, 1)
pylab.hold(True)
pylab.plot(x, a, 'b', linewidth=4)
pylab.plot(x, y, 'r', linewidth=1.5)


print('Recreated image:    ' + str(np.round(np.real(y), -4).tolist()))
print('Recreated image residual norm is: ' + str(la.norm(r)))
#pdb.set_trace()

# Interpolate ultra high resolution image
N = 200
X = np.linspace(0, n - 1, N)
M = np.array([scipy.exp(j * 2 * np.pi * fk[k] * np.array(X)) for k in range(n)])
yh = np.array(np.mat(M).T * np.mat(S).T / n).reshape(N)
pylab.plot(X, yh, 'g')
pylab.legend(['Original Image', 'Recreated Image', 'High-Res Interpolation'])

pylab.subplot(2, 1, 2)
pylab.hold(True)

ll = 5
rl = 10
N = (rl - ll) + 1
N = 100
X = np.linspace(x[ll], x[rl], N)
M = np.array([scipy.exp(j * 2 * np.pi * fk[k] * np.array(X)) for k in range(n)])

yh = np.array(np.mat(M).T * np.mat(S).T / n).reshape(N)

pylab.plot(x[ll:rl + 1], a[ll:rl + 1], 'b', linewidth=4)
pylab.plot(X, yh, 'g', linewidth=1.5)





pylab.show()



