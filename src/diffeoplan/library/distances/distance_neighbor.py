from . import np, contract
from diffeo2d import Flattening, cmap, coords_iterate


class DistanceNeighbor():
    
    @contract(size='seq[2](int)', neighborarea='seq[2](int)')
    def __init__(self, size=[160, 120], neighborarea=[8, 8]):
        self.size = size
        shape = size
        nsensels = shape[0] * shape[1] 
    
        # for each sensel, create an area
        lengths = np.array(neighborarea)
    
        neighbor_coords = [None] * nsensels
        self.neighbor_indices_flat = [None] * nsensels
    
        flattening = Flattening.by_rows(tuple(shape))
        cmg = cmap(lengths)
        self.cmg = cmg
        for coord in coords_iterate(shape):
            k = flattening.cell2index[coord]
            cm = cmg.copy()
            cm[:, :, 0] += coord[0]
            cm[:, :, 1] += coord[1]
            cm[:, :, 0] = cm[:, :, 0] % shape[0]
            cm[:, :, 1] = cm[:, :, 1] % shape[1]
            neighbor_coords[k] = cm
    
            indices = np.zeros(lengths, 'int32')
            for a, b in coords_iterate(indices.shape):
                c = tuple(cm[a, b, :])
                indices[a, b] = flattening.cell2index[c]
    
            self.neighbor_indices_flat[k] = np.array(indices.flat)
        
    def distance(self, y0, y1):
        """
            Compare Y1(s) with best matching pixel in Y2(s_n) 
            where s_s \in (the neighbourhood of s)
        """

        Y1 = y0.resize(self.size).get_values()
        Y2 = y1.resize(self.size).get_values()
        neighbor_indices_flat = self.neighbor_indices_flat

        nsensel = Y1[:, :, 0].size 
        best = np.zeros((nsensel, Y1.shape[2])) 
        for c in range(Y1.shape[2]):
            y1_flat = Y1[:, :, c].astype(np.int16).flat 
            y2_flat = Y2[:, :, c].astype(np.int16).flat 
            for k in range(nsensel):
                a = y1_flat[k].astype(np.float)
                b = y2_flat[neighbor_indices_flat[k]]
                diff = np.abs(a - b) 
                best[k, c] = np.min(diff) 
        return np.mean(best)  # /self.maxval_distance_neighborhood_bestmatch
    
