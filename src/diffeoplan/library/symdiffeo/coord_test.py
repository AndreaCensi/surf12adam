'''
Some quick tests
'''
import numpy as np
from geometry import SE2, se2
from diffeoplan.utils import construct_matrix_iterators
from reprep import Report

def dictmap(f, d):
    return dict([(k, f(v)) for k, v in d.items()])
    
def test_coords1():
    vl = np.array([0, 1, 0, ])
    va = np.array([np.deg2rad(20), 0, 0])


    vel = {
       'F': vl,
       'FL': vl + va,
       'FR': vl - va,
       'B': (-vl),
       'BL': (-vl + va),
       'BR': (-vl - va),
    }

    def make_motion(v):
        A = se2.algebra_from_vector(v)
        Q = SE2.group_from_algebra(A)
        return Q
    
    motions = dictmap(make_motion, vel)
    
    print motions
    for k, v in motions.items():
        
        print(' - %s:  %s -> %s' % (k, vel[k], SE2.friendly(v)))

    names = sorted(vel.keys())
    
    def commuting(a, b):
        q1 = motions[a]
        q2 = motions[b]
        return SE2.distance(SE2.multiply(q1, q2),
                            SE2.multiply(q2, q1))
    
    def same(a, b):
        q1 = motions[a]
        q2 = motions[b]
        return SE2.distance(q1, q2)
    
    def anti(a, b):
        q1 = motions[a]
        q2 = motions[b]
        return SE2.distance(q1, SE2.inverse(q2))     
        
    cD = construct_matrix_iterators((names, names), commuting)
    aD = construct_matrix_iterators((names, names), anti)
    D = construct_matrix_iterators((names, names), same)
    
    r = Report('test_coords1')
    r.table('D', data=D, cols=names, rows=names, fmt='%f')
    r.table('aD', data=aD, cols=names, rows=names, fmt='%f')
    r.table('cD', data=cD, cols=names, rows=names, fmt='%f')
    r.to_html('out/test_coords1/test_coords1.html')
    
if __name__ == '__main__':
    test_coords1()
