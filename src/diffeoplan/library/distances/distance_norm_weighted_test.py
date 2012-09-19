from . import DistanceNorm, DistanceNormWeighted, np
from .. import UncertainImage
from geometry.utils import assert_allclose

def test_distance_norm_weighted3():
    def random_uncertain_image(shape=(20, 30)):
        return UncertainImage(np.random.rand(*shape), np.round(np.random.rand(*shape)))
    
    L2w = DistanceNormWeighted(2)
    y1 = random_uncertain_image()
    y2 = random_uncertain_image()
    d = L2w.distance(y1, y2)
    assert 0 <= d <= 1
    
def test_distance_norm_weighted2():
    
    shape = (5, 3)
    one = np.ones(shape)
    zero = 0 * one

    L2w = DistanceNormWeighted(2)
    L1w = DistanceNormWeighted(1)

    r = np.random.rand(*shape)
    
    # two equal but with strange uncertainty
    y1 = UncertainImage(r, np.round(np.random.rand(*shape)))
    y2 = UncertainImage(r, np.round(np.random.rand(*shape)))
    
    assert_allclose(L2w.distance(y1, y2), 0)
    assert_allclose(L1w.distance(y1, y2), 0)
    
    
    # two completely different but with strange uncertainty
    y1 = UncertainImage(one, np.round(np.random.rand(*shape)))
    y2 = UncertainImage(zero, np.round(np.random.rand(*shape)))
    
    assert_allclose(L2w.distance(y1, y2), 1)
    assert_allclose(L1w.distance(y1, y2), 1)
    
def test_distance_norm_weighted1():
    
    shape = (5, 3)
    one = np.ones(shape)
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
