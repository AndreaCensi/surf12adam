from . import np, contract
from geometry.utils.numpy_backport import assert_allclose

from boot_agents.diffeo.plumbing.flat_structure import FlatStructure

class DistanceNeighborEff():
    """ A more efficient version. """
    
    @contract(size='seq[2](int)', neighborarea='seq[2](int)')
    def __init__(self, size=[160, 120], neighborarea=[8, 8]):
        self.fs = FlatStructure(shape=size, neighborarea=neighborarea)
        self.size = size
        
    def distance(self, y0, y1):
        """
            Compare Y1(s) with best matching pixel in Y2(s_n) 
            where s_s \in (the neighbourhood of s)
        """

        Y1 = y0.resize(self.size).get_values()
        Y2 = y1.resize(self.size).get_values()
        
        Y2_unrolled = self.fs.image2unrolledneighbors(Y2)
        Y1_repeated = self.fs.image2repeated(Y1)
        assert_allclose(Y2_unrolled.shape, Y1_repeated.shape) 
        
        diff1 = np.abs(Y2_unrolled - Y1_repeated)
        myres = np.mean(np.min(diff1, axis=1))
        
        if False:
            # old method, equivalent
            neighbor_indices_flat = self.fs.neighbor_indices_flat
            nchannels = Y1.shape[2]
            nsensel = Y1[:, :, 0].size 
            best = np.zeros((nsensel, Y1.shape[2])) 
            for c in range(nchannels):
                y1_flat = Y1[:, :, c].astype(np.int16).flat 
                y2_flat = Y2[:, :, c].astype(np.int16).flat 
                for k in range(nsensel):
                    a = y1_flat[k].astype(np.float)
                    b = y2_flat[neighbor_indices_flat[k]]
                    diff = np.abs(a - b) 
                    best[k, c] = np.min(diff) 
            res = np.mean(best)#/self.maxval_distance_neighborhood_bestmatch
            assert_allclose(res, myres)
    
        return myres
