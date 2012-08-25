from . import PlanReducer, np
from geometry.utils import assert_allclose
import itertools


def plan_reducer_test():
    
    labels = ['+0', '0+', '-0', '0-']
    
    same = np.eye(4, dtype='bool')
    inverse = np.zeros((4, 4), dtype='bool')
    commute = np.ones((4, 4), dtype='bool')
    
    inverse[0, 2] = inverse[2, 0] = 1
    inverse[1, 3] = inverse[3, 1] = 1
    
    
    pr = PlanReducer.from_matrices(labels, commute, inverse, same)

    expected = []
    
    for x in itertools.permutations(labels):
        expected.append((x, ()))
        
    for a, b in expected:
        a = tuple(a)
        b = tuple(b)
        b2 = pr.get_canonical(a)
        assert_allclose(b, b2) 



def plan_reducer_test2():
    
    labels = [0, 1, 2, 3]
    
    same = np.eye(4, dtype='bool')
    inverse = np.zeros((4, 4), dtype='bool')
    commute = np.ones((4, 4), dtype='bool')
    
    inverse[0, 3] = inverse[3, 0] = 1
    inverse[1, 2] = inverse[2, 1] = 1
    
    
    pr = PlanReducer.from_matrices(labels, commute, inverse, same)

    expected = []
    for x in itertools.permutations(labels):
        expected.append((x, ()))
        
    expected.append(((0, 0, 0, 0, 2, 2, 3),
                    (0, 0, 0, 2, 2)))
    
    expected.append(((2, 2, 0, 2, 2),
                     (0, 2, 2, 2, 2)))

    expected.append(((3, 2), (2, 3)))
        
    for a, b in expected:
        a = tuple(a)
        b = tuple(b)
        b2 = pr.get_canonical(a)
        assert_allclose(b, b2) 
