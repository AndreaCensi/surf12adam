from . import np, contract
from boot_agents.diffeo import Flattening, cmap, coords_iterate
from geometry.utils.numpy_backport import assert_allclose
import pdb


class FlatStructure():
    
    def __init__(self, shape, neighborarea):
        """
            Suppose 
                shape = (H, W)
                nighborarea = (h, w)
                A = h * w
                N = H * W
            Then 
                self.neighbor_indices_flat  
            is a K x N array.

        """
    
        # for each sensel, create an area 
        cmg = cmap(np.array(neighborarea))
        area_shape = cmg.shape[0], cmg.shape[1]

        H, W = shape
        h, w = area_shape
        N = H * W
        A = h * w 
        
        neighbor_coords = [None] * N
        self.neighbor_indices_flat = [None] * N
    
        self.flattening = Flattening.by_rows(shape)
        
        self.cmg = cmg
        for coord in coords_iterate(shape):
            k = self.flattening.cell2index[coord]
            cm = cmg.copy()
            cm[:, :, 0] += coord[0]
            cm[:, :, 1] += coord[1]
            cm[:, :, 0] = cm[:, :, 0] % shape[0]
            cm[:, :, 1] = cm[:, :, 1] % shape[1]
            neighbor_coords[k] = cm
            indices = np.zeros(area_shape, 'int32')
            for a, b in coords_iterate(indices.shape):
                c = tuple(cm[a, b, :])
                indices[a, b] = self.flattening.cell2index[c]
    
            self.neighbor_indices_flat[k] = np.array(indices.flat)

        self.neighbor_indices_flat = np.array(self.neighbor_indices_flat)
        
        
        
    @contract(value='array[HxW]', returns='array[NxA]')
    def values2unrolledneighbors(self, value):
        # first convert the array to unrolled N
        valueflat = self.flattening.rect2flat(value)
        return valueflat[self.neighbor_indices_flat]

    @contract(value='array[HxW]', returns='array[NxA]')
    def values2repeated(self, value):
        valueflat = self.flattening.rect2flat(value)
        N, A = self.neighbor_indices_flat.shape
        return np.repeat(valueflat.reshape((N, 1)), A, axis=1)
        
    @contract(value='array[HxWxC]', returns='array[NxAxC]')
    def image2unrolledneighbors(self, value):
        _, _, C = value.shape
        N, A = self.neighbor_indices_flat.shape
        valueflat = np.zeros((N, A, C), value.dtype)
        for c in range(C):
            valueflat[:, :, c] = self.values2unrolledneighbors(value[:, :, c])
        return valueflat

    @contract(value='array[HxWxC]', returns='array[NxAxC]')
    def image2repeated(self, value):
        _, _, C = value.shape
        N, A = self.neighbor_indices_flat.shape
        res = np.zeros((N, A, C), value.dtype)
        for c in range(C):
            res[:, :, c] = self.values2repeated(value[:, :, c])
        return res



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
