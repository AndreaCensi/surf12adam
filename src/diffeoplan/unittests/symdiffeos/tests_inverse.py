from diffeoplan.unittests.tests_generation import for_all_symdiffeos
import numpy as np
from geometry import printm


@for_all_symdiffeos
def check_symdiffeo_inverse1(id_symdiffeo, symdiffeo): #@UnusedVariable
    inverse = symdiffeo.get_inverse()
    manifold = symdiffeo.get_topology()
     
    p = np.array([0, 0])
    p1 = symdiffeo.apply(p)
    p2 = inverse.apply(p1)

    try:    
        print manifold
        print manifold.assert_close
        manifold.assert_close(p, p2)
    except:
        print('Function: %s' % symdiffeo)
        print('Inverse: %s' % inverse)
        printm('p', p)
        printm('f(p)', p1)
        printm('g(f(p))', p2)
        raise
    
