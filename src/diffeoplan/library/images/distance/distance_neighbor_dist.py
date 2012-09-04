from . import contract, logger, np
from boot_agents.diffeo.plumbing import FlatStructure
from diffeoplan.utils import assert_allclose


class DistanceNeighborRatio():
    """ A more efficient version. """
    
    @contract(ratios='seq[2](float,>=0,<=0.5)')
    def __init__(self, ratios):
        self.fs = None
        self.ratios = ratios
    
    def _get_flat_structure(self, shape):
        if self.fs is None:
            def make_size(x):
                x = int(np.ceil(x))
                if x % 2 == 0:
                    x += 1
                return x
            
            ax = make_size(shape[0] * self.ratios[0])
            ay = make_size(shape[1] * self.ratios[1])
            area = (ax, ay)
            if np.min(area) == 1:
                logger.warning('Area is too small. Ratios: %s shape: %s area: %s' % 
                               (self.ratios, shape, area))
            logger.info('ratios: %s shape: %s area: %s' % (self.ratios,
                                                           shape, area))
            self.fs = FlatStructure(shape=shape, neighborarea=area)
        
        return self.fs
        
    def distance(self, y0, y1):
        """
            Compare Y1(s) with best matching pixel in Y2(s_n) 
            where s_s \in (the neighbourhood of s)
        """

        Y1 = y0.get_values()
        Y2 = y1.get_values()
        
        fs = self._get_flat_structure(Y1.shape[:2])
        
        Y2_unrolled = fs.image2unrolledneighbors(Y2)
        Y1_repeated = fs.image2repeated(Y1)
        assert_allclose(Y2_unrolled.shape, Y1_repeated.shape) 
        
        diff1 = np.abs(Y2_unrolled - Y1_repeated)
        myres = np.mean(np.min(diff1, axis=1))
        
        if False:
            # old method, equivalent
            neighbor_indices_flat = fs.neighbor_indices_flat
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


class DistanceNeighborDist(DistanceNeighborRatio):
    
    
    def distance(self, y0, y1):
        """
            Compare Y1(s) with best matching pixel in Y2(s_n) 
            where s_s \in (the neighbourhood of s)
        """

        Y1 = y0.get_values()
        Y2 = y1.get_values()
        
        fs = self._get_flat_structure(Y1.shape[:2])
        
        Y2_unrolled = fs.ndvalues2unrolledneighbors(Y2)
        Y1_repeated = fs.ndvalues2repeated(Y1)
        assert_allclose(Y2_unrolled.shape, Y1_repeated.shape) 
        
        diff1 = np.abs(Y2_unrolled - Y1_repeated)
        if diff1.ndim == 3:
            diff1 = np.mean(diff1, axis=2)
            
        D = fs.get_distances()
        
        N, _ = D.shape
        distance_to_closest = np.zeros(N)
        for i in range(N):
            diff_i = diff1[i, :]
            # best matches
            matches, = np.nonzero(diff_i == np.min(diff_i))
            distance_to_matches = D[i, matches]
            distance_to_closest[i] = np.min(distance_to_matches)
            
            if False:
                if i == N / 2:
                    print('i: %s' % i)
                    print('distances[i]: %s' % D[i, :])
                    print('diff1[i]: %s' % diff_i)
                    print('matches: %s' % matches)
                    print('matches dist: %s' % distance_to_matches)
                    print('dist[i]: %s' % distance_to_closest[i])

        myres = np.mean(distance_to_closest)
        return myres



