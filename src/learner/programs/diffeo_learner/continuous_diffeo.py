'''
Created on Aug 24, 2012

@author: adam
'''
from contracts import contract
import numpy as np
from PIL import Image #@UnresolvedImport
from boot_agents.diffeo.diffeo_display import diffeo_to_rgb_angle, \
    diffeo_to_rgb_norm
#from matplotlib import figure
import matplotlib.pyplot as plt

@contract(x='array[MxNx2]')
def get_valid_diffeomorphism(x):
    M, N = x.shape[0], x.shape[1]
    
    #assert (0 <= x[:, :, 0]).all()
    #assert (0 <= x[:, :, 1]).all()
    x[x < 0] = 0
    
    #assert (x[:, :, 0] < M).all()
    x[x[:, :, 0] < M] = M
    
    #assert (x[:, :, 1] < N).all()
    x[x[:, :, 1] < N] = N
    
    return x
    
@contract(x='array[MxNx2]')
def displacement_to_coord(x):
    Y, X = np.meshgrid(range(x.shape[0]), range(x.shape[1]))
    
    x[:, :, 0] = x[:, :, 0] + Y
    x[:, :, 1] = x[:, :, 1] + X
    
    return x 

@contract(diff='valid_diffeomorphism,array[MxNx2]')
def display_diffeo_images(diff):
    Image.fromarray(diffeo_to_rgb_angle(diff)).resize((400, 300)).show()
    Image.fromarray(diffeo_to_rgb_norm(diff)).resize((400, 300)).show()
    
def sim_square_check(sim_square):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    X, Y = np.meshgrid(range(sim_square.shape[0]), range(sim_square.shape[0]))
    
    surf = ax.plot_surface(X, Y, sim_square, rstride=1, cstride=1,
        linewidth=0, antialiased=False)
    
#    ax.zaxis.set_major_locator(LinearLocator(10))
#    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

    fig.colorbar(surf, shrink=0.5, aspect=5)

    plt.show()
    
def sim_square_modify(sim_square):
    mi = np.min(sim_square)
    ma = np.max(sim_square)
    
    mod_square = -(sim_square - ma) / (ma - mi)
    return mod_square
    
