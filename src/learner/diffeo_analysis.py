'''
Created on Aug 27, 2012
Generates statistics images of diffeo estimator

@author: Adam Nilsson
'''
from PIL import Image #@UnresolvedImport
import numpy as np
from boot_agents.diffeo.misc_utils import coords_iterate
from matplotlib import cm
import matplotlib.pyplot as plt
import os
import pickle

# The peak plots are generated for the points in c_peak_list
c_peak_list = ((15, 15), (20, 15), (25, 15), (100, 100), (140, 120), (0, 0), (0, 60))

class DiffeoAnalysis:
    def __init__(self, estimator, name, shape, neighbor_shape, folder=''):
        self.name = name
        self.shape = shape
        self.neighbor_shape = neighbor_shape
        self.estimator = estimator
        self.folder = folder
        if self.folder[-1] != '/':
            self.folder += '/'
        if not os.path.exists(folder):
            os.makedirs(folder)
        if not os.path.exists(folder + 'peaks'):
            os.makedirs(folder + 'peaks')
        
    
    def get_order_image(self, order_array_flat):
        '''
        This gennerates an image search_area*image_size with the order 
        array (E4 error) for each pixel.
        :param order_array_flat:
        '''
        order_image = Image.new('L', np.flipud(self.shape) * np.flipud(self.neighbor_shape))
        
        max_E4 = np.max(order_array_flat)
        min_E4 = np.min(order_array_flat)
        
        for c in coords_iterate(self.shape):
            # find index in flat array
            k = self.estimator.flattening.cell2index[c]
            
            E4_local_flat = order_array_flat[k]
            '''
            Data is normalized such that 1 is no error and 0 is max error.
            This can easily be inverted by the commented line below
            '''
            E4_local_flat_normalized = 1 - (E4_local_flat - min_E4) / (max_E4 - min_E4)
#            E4_local_flat_normalized = (E4_local_flat - min_E4) / (max_E4 - min_E4)
            
            E4_local = E4_local_flat_normalized.reshape(self.neighbor_shape)
            
            
            if c in c_peak_list:
                fig = plt.figure()
                ax = fig.gca(projection='3d')
                X, Y = np.meshgrid(range(self.neighbor_shape[0]), range(self.neighbor_shape[1]))
                ax.plot_surface(X, Y, E4_local, rstride=1, cstride=1, cmap=cm.jet, #@UndefinedVariable
                                       linewidth=0, antialiased=False)
                if not os.path.exists(self.folder + self.name + '/peaks'):
                    os.makedirs(self.folder + self.name + '/peaks')
                plt.savefig(self.folder + self.name + '/peaks/order' + str(c) + '.png')
                plt.savefig(self.folder + self.name + '/peaks/order' + str(c) + '.pdf')
            
            # find the upper left corner to paste
            p0 = tuple(np.flipud(np.array(c) * self.neighbor_shape))
            # calculate the box to past into
            box = p0 + tuple(p0 + self.neighbor_shape)
            order_image.paste(Image.fromarray((E4_local * 255).astype('uint8')), box)
        return order_image
            
    def get_num_image(self, order_array_flat):
        '''
        This gennerates the image with information of how many times a pixel 
        was the best match.
        :param order_array_flat:
        '''
        
        order_image = Image.new('L', np.flipud(self.shape) * np.flipud(self.neighbor_shape))
        
        max_E4 = np.max(order_array_flat)
        min_E4 = np.min(order_array_flat)
        
        for c in coords_iterate(self.shape):
            # find index in flat array
            k = self.estimator.flattening.cell2index[c]
            
            E3_local_flat = order_array_flat[k]
            E3_local_flat_normalized = 1 - (E3_local_flat - min_E4) / (max_E4 - min_E4)
            
            E3_local = E3_local_flat_normalized.reshape(self.neighbor_shape)
            
            
            # find the upper left corner to paste
            p0 = tuple(np.flipud(np.array(c) * self.neighbor_shape))
            # calculate the box to past into
            box = p0 + tuple(p0 + self.neighbor_shape)
            order_image.paste(Image.fromarray((E3_local * 255).astype('uint8')), box)
        return order_image
    
    def get_similarity_image(self, sim_array):
        '''
        This gennerates a image with the simmilarity function E1 for each pixel.
        :param sim_array:
        '''
        
        sim_image = Image.new('L', np.flipud(self.shape) * np.flipud(self.neighbor_shape))
        
        max_sim = np.max(sim_array)
        min_sim = np.min(sim_array)

        for c in coords_iterate(self.shape):
            # find index in flat array
            k = self.estimator.flattening.cell2index[c]
            
            sim_local_flat = sim_array[k]
            '''
            Data is normalized such that 1 is no error and 0 is max error.
            This can easily be inverted by the commented line below
            '''
            sim_local_flat_normalized = 1 - (sim_local_flat - min_sim) / (max_sim - min_sim)
#            sim_local_flat_normalized = (sim_local_flat - min_sim) / (max_sim - min_sim)
            
            sim_local = sim_local_flat_normalized.reshape(self.neighbor_shape)
            
            if c in c_peak_list:
                fig = plt.figure()
                ax = fig.gca(projection='3d')
                X, Y = np.meshgrid(range(self.neighbor_shape[0]), range(self.neighbor_shape[1]))
                ax.plot_surface(X, Y, sim_local, rstride=1, cstride=1, cmap=cm.jet, #@UndefinedVariable
                                       linewidth=0, antialiased=False)
                if not os.path.exists(self.folder + self.name + '/peaks'):
                    os.makedirs(self.folder + self.name + '/peaks')
                plt.savefig(self.folder + self.name + '/peaks/sim' + str(c) + '.png')
                plt.savefig(self.folder + self.name + '/peaks/sim' + str(c) + '.pdf')
            
            # find the upper left corner to paste
            p0 = tuple(np.flipud(np.array(c) * self.neighbor_shape))
            # calculate the box to past into
            box = p0 + tuple(p0 + self.neighbor_shape)
            sim_image.paste(Image.fromarray((sim_local * 255).astype('uint8')), box)
        return sim_image
            
    def get_E2_image(self, best_array, sim_array):        
        max_sim = np.max(sim_array)
        
        scaled_size = np.flipud(self.shape) * np.flipud(self.neighbor_shape)
        normed_array = ((1 - best_array / (max_sim)) * 255).astype('uint8')
        reshaped_array = normed_array.reshape(np.flipud(self.shape))
        
        return Image.fromarray(reshaped_array, mode='L').resize(scaled_size)
        
    def make_E2_hist(self, best_array, sim_array, name):        
        max_sim = np.max(sim_array)
        
        normed_array = (best_array / (max_sim))
        assert((normed_array >= 0).all())
        assert((normed_array <= 1).all())
        plt.figure()
        plt.hist(normed_array, 100, facecolor='green')
        plt.savefig(name)
        
    def make_images(self):
        estimator = self.estimator
                
        sim_image = self.get_similarity_image(estimator.neighbor_similarity_flat)
        sim_image.save(self.folder + self.name + 'E1_image.png')
        sim_image.save(self.folder + self.name + 'E1_image.pdf')
        pickle.dump(np.array(sim_image), open(self.folder + self.name + 'E1_image.np.pickle', 'wb'))
        
        best_image = self.get_E2_image(estimator.neighbor_similarity_best, estimator.neighbor_similarity_flat)
        best_image.save(self.folder + self.name + 'E2_image.png')
        best_image.save(self.folder + self.name + 'E2_image.pdf')
        pickle.dump(np.array(best_image), open(self.folder + self.name + 'E2_image.np.pickle', 'wb'))
        
        self.make_E2_hist(estimator.neighbor_similarity_best, estimator.neighbor_similarity_flat, self.folder + self.name + 'E2hist.png')
        
        num_image = self.get_num_image(estimator.neighbor_num_bestmatch_flat)
        num_image.save(self.folder + self.name + 'E3_num.png')
        num_image.save(self.folder + self.name + 'E3_num.pdf')
        pickle.dump(np.array(num_image), open(self.folder + self.name + 'E3_image.np.pickle', 'wb'))

        order_image = self.get_order_image(estimator.neighbor_argsort_flat)
        order_image.save(self.folder + self.name + 'E4_image.png')
        order_image.save(self.folder + self.name + 'E4_image.pdf')
        pickle.dump(np.array(order_image), open(self.folder + self.name + 'E4_image.np.pickle', 'wb'))
