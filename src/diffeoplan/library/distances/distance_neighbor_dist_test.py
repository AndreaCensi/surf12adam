from . import DistanceNeighborDist, np, DistanceNorm
from .. import UncertainImage
from diffeoplan.utils import assert_allclose
from diffeo2d import FlatStructure

def test_distance_neighbor_dist1():
    gs = FlatStructure((100, 100), (1, 1))
    D = gs.get_distances()
    assert_allclose(D, 0)

def test_distance_neighbor_dist3():
    gs = FlatStructure((100, 100), (3, 3))
    D = gs.get_distances()
    assert_allclose(np.max(D), np.sqrt(2))
    
def test_distance_neighbor_dist2():
    
    # A random pattern
    a = np.random.rand(100, 100, 3)

    # Let's get some translated slices
    w = 80
    N = 10
    A = []
    for i in range(N):
        A.append(a[i:i + w, 0:0 + w, ...])
    print()
    areas = [0.01, 0.03, 0.05, 0.07]
    Ds = [DistanceNeighborDist((a, a)) for a in areas
          ]
    L2 = DistanceNorm(2)
    for i in range(N):
        y0 = UncertainImage(A[0][:, :, 0])
        y1 = UncertainImage(A[i][:, :, 0])
        vl2 = L2.distance(y0, y1)
        ds = [x.distance(y0, y1) for x in Ds]
        s = ";".join("%1.5f" % x for x in ds)
        print('- step %d L2: %1.5f %s' % (i, vl2, s))


def test_distance_neighbor_dist2_rgb():
    
    # A random pattern
    a = np.random.rand(100, 100, 3)

    # Let's get some translated slices
    w = 80
    N = 10
    A = []
    for i in range(N):
        A.append(a[i:i + w, 0:0 + w, ...])
    print()
    areas = [0.01, 0.03, 0.05, 0.07]
    Ds = [DistanceNeighborDist((a, a)) for a in areas
          ]
    L2 = DistanceNorm(2)
    for i in range(N):
        y0 = UncertainImage(A[0])
        y1 = UncertainImage(A[i])
        vl2 = L2.distance(y0, y1)
        ds = [x.distance(y0, y1) for x in Ds]
        s = ";".join("%1.5f" % x for x in ds)
        print('- step %d L2: %1.5f %s' % (i, vl2, s))


def test_distance_neighbor_dist2_repeated():
    # A constant pattern
    a = np.ones((100, 100, 3))

    # Let's get some translated slices
    w = 80
    N = 10
    A = []
    for i in range(N):
        A.append(a[i:i + w, 0:0 + w, ...])
    print()
    areas = [0.01, 0.03, 0.05, 0.07]
    Ds = [DistanceNeighborDist((a, a)) for a in areas]
    L2 = DistanceNorm(2)
    for i in range(N):
        y0 = UncertainImage(A[0])
        y1 = UncertainImage(A[i])
        vl2 = L2.distance(y0, y1)
        ds = [x.distance(y0, y1) for x in Ds]
        assert_allclose(ds, 0)  # < this must be zero
        s = ";".join("%1.5f" % x for x in ds)
        print('- step %d L2: %1.5f %s' % (i, vl2, s))
