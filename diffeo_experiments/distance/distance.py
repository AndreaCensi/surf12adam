
import numpy as np
from PIL import Image
import pdb
import boot_agents
from boot_agents import *


class distance:    
    maxval_distance_scaled_mean = 1.0 
    maxval_distance_scaled_std = 1.0
    maxval_distance_neighborhood_bestmatch = 1.0
#    maxval_distance_neighborhood_distance
    
    def calculate_max(self,size):
        
        """
        Calculate max distance for the metric
        """
        pdb.set_trace()
        imr1 = np.random.randint(0,255,(size[1],size[0],3)).astype(np.uint8)
        imr2 = np.random.randint(0,255,(size[1],size[0],3)).astype(np.uint8)
        self.maxval_distance_scaled_mean = self.distance_scaled_mean(imr1,imr2)
        self.maxval_distance_scaled_std = self.distance_scaled_std(imr1,imr2)
        self.maxval_distance_neighborhood_bestmatch = self.distance_neighborhood_bestmatch(imr1,imr2)
    
    def __init__(self):
        self.neighbor_indices_flat = np.array([0])
        
    
    def distance_scaled_mean(self,Y1,Y2,scale=1):
        size = [Y1.shape[1]*scale,Y1.shape[0]*scale]
    #    pdb.set_trace()
        y1 = np.array(Image.fromarray(Y1).resize(size).getdata(),np.int16)
        y2 = np.array(Image.fromarray(Y2).resize(size).getdata(),np.int16)
        return np.mean(np.abs(y1-y2))/self.maxval_distance_scaled_mean
        
    def distance_scaled_std(self,Y1,Y2,scale=1):
        size = [Y1.shape[1]*scale,Y1.shape[0]*scale]
    #    pdb.set_trace()
        y1 = np.array(Image.fromarray(Y1).resize(size).getdata(),np.int16)
        y2 = np.array(Image.fromarray(Y2).resize(size).getdata(),np.int16)
        return np.std(np.abs(y1-y2))/self.maxval_distance_scaled_std
    

    def neighbor_indices_flat_init(self,Y1,neighborarea):
#        global neighbor_indices_flat
        shape = Y1.shape[:2]
        nsensels = shape[0]*shape[1]
    
        # for each sensel, create an area
        lengths = np.array(neighborarea)
        print(' Field Shape: %s' % str(shape))
    #        print('    Fraction: %s' % str(max_displ))
        print(' Search area: %s' % str(lengths))
    
        neighbor_coords = [None] * nsensels
    #            neighbor_indices = [None] * nsensels
        self.neighbor_indices_flat = [None] * nsensels
    #            neighbor_similarity_flat = [None] * nsensels
    
        flattening = Flattening.by_rows(shape)
#        pdb.set_trace()
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
    
    #                neighbor_indices[k] = indices
            self.neighbor_indices_flat[k] = np.array(indices.flat)
#        pdb.set_trace()
    #                neighbor_similarity_flat[k] = np.zeros(indices.size,dtype='float32')
        print('done')
        
    def distance_neighborhood_bestmatch(self,Y1, Y2, neighborarea=[5,5]):
        """
            Fompare Y1 with best matching pixel in the neighbourhood in Y2
        """
        neighbor_indices_flat = self.neighbor_indices_flat
    #    pdb.set_trace()
    #    if neighbor_indices_flat == None:
    #        neighbor_indices_flat = np.array([0])
    #    if len(neighbor_indices_flat) <> Y1.shape[0]*Y1.shape[1]:
    #        neighbor_indices_flat_init(Y1)
    #        pdb.set_trace()
    #        size = np.array(Y1.shape[:2],dtype=np.float)
    #        estr = DiffeomorphismEstimator(neighborarea/size,"continuous")
    #    pdb.set_trace()
        nsensel = Y1[:,:,0].size
        best = np.zeros((nsensel,Y1.shape[2]))
        for c in range(Y1.shape[2]):
    #        pdb.set_trace()
            y1_flat = Y1[:,:,c].astype(np.int16).flat
            y2_flat = Y2[:,:,c].astype(np.int16).flat
            for k in range(nsensel):
#                pdb.set_trace()
                a = y1_flat[k].astype(np.float)
                b = y2_flat[neighbor_indices_flat[k]]
                diff = np.abs(a-b)
                best[k,c] = np.min(diff)
        return np.mean(best)/self.maxval_distance_neighborhood_bestmatch

    
    def distance_neighborhood_distance(self,Y1, Y2, scale=1.0):
        """
            Distance from s to best matching pixel in Y2
        """
        # Scale down image
        shape = scale*np.array(Y1.shape).astype(np.int)
        size = np.array([Y1.shape[1]*scale,Y1.shape[0]*scale],dtype=np.int)
        Y1 = np.array(Image.fromarray(Y1).resize(size).getdata(),np.int16).reshape(shape)
        Y2 = np.array(Image.fromarray(Y2).resize(size).getdata(),np.int16).reshape(shape)
        
        
        neighbor_indices_flat = self.neighbor_indices_flat
#        pdb.set_trace()
#        print('nsensel = Y1[:,:,0].size')
        nsensel = Y1[:,:,0].size
#        print('done')
        best_dist = np.zeros((nsensel,Y1.shape[2]))
        for c in range(Y1.shape[2]):
    #        pdb.set_trace()
            y1_flat = Y1[:,:,c].astype(np.int16).flat
            y2_flat = Y2[:,:,c].astype(np.int16).flat
            for k in range(nsensel):
#                pdb.set_trace()
                a = y1_flat[k].astype(np.float)
                b = y2_flat[neighbor_indices_flat[k]]
                diff = np.abs(a-b)
                
                Xd = self.cmg[:,:,0].flat
                Yd = self.cmg[:,:,1].flat
                min_ind = np.nonzero(diff == np.min(diff))
                
                best_dist[k,c] = np.sqrt(np.sum(Xd[min_ind]**2 + Yd[min_ind]**2)/len(min_ind))
        return np.mean(best_dist)

if __name__ == '__main__':
    dist = distance()
#    pdb.set_trace() 
    dist.calculate_max((160,120))
