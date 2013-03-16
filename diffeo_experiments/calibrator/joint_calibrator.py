
import numpy as np
import pdb

r = 3.
#theta0 = -2. / 9 * np.pi
theta0 = 0.
#beta = np.pi * 2 / 3
beta = 0


P = np.mat([1.7, 2]).T

def Rot(theta):
    ex = np.array([np.cos(theta), np.sin(theta)])
    ey = np.array([-np.sin(theta), np.cos(theta)])
    return np.mat([ex, ey]).T


def P1(theta, P):
    M = Rot(theta)
    C = Rot(beta + theta)
    R = np.mat([r, 0]).T
    return C.T * (P - M * R)

def s(theta, P):
    p1 = P1(theta, P)
    return np.arctan(p1[0] / p1[1])

p1 = P1(theta0, P)
print s(theta0, P)

s0 = s(theta0, P)
delta_theta = 0.1
theta1 = theta0 + delta_theta * 1
theta2 = theta0 + delta_theta * 2
theta3 = theta0 + delta_theta * 3
s1 = s(theta1, P)
s2 = s(theta2, P)
s3 = s(theta3, P)

from sympy import Symbol, nsolve #@UnresolvedImport
import sympy #@UnusedImport @UnresolvedImport
from sympy import sin, cos #@UnresolvedImport

r = Symbol('r')
beta = Symbol('beta')
d0 = Symbol('d0')
d1 = Symbol('d1')
d2 = Symbol('d2')
d3 = Symbol('d3')



fx1 = r + cos(beta - s0) * d0 - (cos(theta1) * r + cos(theta1 + beta - s1) * d1)
fy1 = 0 + sin(beta - s0) * d0 - (sin(theta1) * r + sin(theta1 + beta - s1) * d1)
fx2 = r + cos(beta - s0) * d0 - (cos(theta2) * r + cos(theta2 + beta - s2) * d2)
fy2 = 0 + sin(beta - s0) * d0 - (sin(theta2) * r + sin(theta2 + beta - s2) * d2)
fx3 = r + cos(beta - s0) * d0 - (cos(theta3) * r + cos(theta3 + beta - s3) * d3)
fy3 = 0 + sin(beta - s0) * d0 - (sin(theta3) * r + sin(theta3 + beta - s3) * d3) 

print nsolve((fx1, fx2, fx3, fy1, fy2, fy3), (r, beta, d0, d1, d2, d3), (3.1, 0.1, 5, 5, 5, 5))
pdb.set_trace()
