import numpy as np
import numpy.linalg as la
import numpy.random
import pdb
import math
import scipy
import pylab
import itertools

# Define a imaginary unit
j = complex(0, 1)

H = 2
W = 3
I = np.random.ranf((H, W))

S = np.fft.fft2(I)

C = list(itertools.product(range(H), range(W)))


def f(k, n):
    return (2 * float(k) / n) % 1 - (2 * k / n)

S_flat = [S[c] for c in C]
F = np.array([[f(c[0], H), f(c[1], W)] for c in C])

Hr = H
Wr = W
np.zeros((Hr, Wr))

XY = list(itertools.product(range(Hr), range(Wr)))

pdb.set_trace()
M = np.array([[scipy.exp(j * np.pi * np.sum(F[i] * XY[xy])) for i in range(len(F))] for xy in range(len(XY))])

Ir_flat = np.real(np.mat(M) * np.mat(S_flat).T / (H * W))

Ir = Ir_flat.reshape((Hr, Wr))

pdb.set_trace()
