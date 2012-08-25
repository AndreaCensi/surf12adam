from . import DistanceNorm, DistanceNormWeighted
from .. import UncertainImage
from geometry.utils.numpy_backport import assert_allclose
import numpy as np

def test_distance_norm_weighted1():
    
    shape = (5, 3)
    one = np.ones(shape)
    v1 = 0.5 * one
    v2 = 0.3 * one
    sure = one
    zero = 0 * one
    
    L2 = DistanceNorm(2)
    L1 = DistanceNorm(1)
    L2w = DistanceNormWeighted(2)
    L1w = DistanceNormWeighted(1)
    
    cases = [
         (0.0, L2, UncertainImage(one, sure), UncertainImage(one, sure)),
         (1.0, L2, UncertainImage(zero, sure), UncertainImage(one, sure)),
         (1.0, L1, UncertainImage(zero, sure), UncertainImage(one, sure)),
         (1.0, L2w, UncertainImage(zero, sure), UncertainImage(one, sure)),
         (1.0, L1w, UncertainImage(zero, sure), UncertainImage(one, sure)),
         (0.5, L1, UncertainImage(zero, sure), UncertainImage(one * 0.5, sure)),
         (0.5, L2, UncertainImage(zero, sure), UncertainImage(one * 0.5, sure)),
         (0.5, L1w, UncertainImage(zero, sure), UncertainImage(one * 0.5, sure)),
         (0.5, L2w, UncertainImage(zero, sure), UncertainImage(one * 0.5, sure)),
    ]
    
    v1 = one * 0.5
    u1 = one
    v2 = v1.copy()
    u2 = u1.copy()
    # Modify random pixels, and put the uncertainty at zero
    for _ in range(5):
        i = np.random.randint(v1.size - 1)
        v2.flat[i] = np.random.rand()
        u2.flat[i] = 0 
    # then the distane should still be 0
    cases.append((0, L1w, UncertainImage(v1, u1), UncertainImage(v2, u2)))
    cases.append((0, L2w, UncertainImage(v1, u1), UncertainImage(v2, u2)))
    
    for expected, d, y0, y1 in cases:
        distance = d.distance(y0, y1)
        try:
            assert_allclose(distance, expected)
        except:
            print(' distance: %s' % d)
            print('       y0: %s' % y0)
            print('       y1: %s' % y1)
            print(' expected: %s' % expected)
            print(' obtained: %s' % distance) 
            raise
