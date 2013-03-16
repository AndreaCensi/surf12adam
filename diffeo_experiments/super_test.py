'''
Created on Jan 30, 2013

@author: adam
'''
import pdb

import pylab
        
from scipy.optimize import leastsq
import numpy as np
import numpy.linalg as la

def f(k, x):
    return k * np.array(x) - np.array([1, 2, 2])

A = leastsq(f, [1], [1, 2, 3], diag=[1, 1, 100])

pdb.set_trace()
def fa(A):
    A = np.array(A)
    d = np.linspace(0, 5, 6)
    return np.array(A[0] * (1 - np.exp(-A[1] * d)))

def res(A, y):
#    pdb.set_trace()
    return fa(A) - y

def fp(P0, P1, A):
    d = la.norm(P0 - P1)
    return A[0] * (1 - np.exp(-A[1] * d))
def fd(d, A):
    return A[0] * (1 - np.exp(-A[1] * d))

def Jfa(A, N):
    A = np.array(A)
    d = np.linspace(0, 5, 6)
    return np.array([(1 - np.exp(-A[1] * d)), A[0] * d * (np.exp(-A[1] * d))]).T

y = fd(np.linspace(0, 5, 6), [1, 1])
d = np.linspace(0, 5, 6)

print('y:\t' + str(y))
print('d:\t' + str(d))
#print('res:' + str(res([1, 1], d)))

A = leastsq(res, [2, 1], y, Dfun=Jfa)
print('A: ' + str(A))

#pdb.set_trace()
