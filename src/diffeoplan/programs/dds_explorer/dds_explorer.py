'''
Created on Jan 9, 2013

@author: adam
'''
from bzrlib.option import OptionParser
from diffeoplan.library.images.uncertain_image import UncertainImage
import numpy as np
import numpy.linalg as la
import pdb
import pickle
import urllib
import pylab
from PIL import Image #@UnresolvedImport
import itertools
from scipy.signal import convolve2d
def main():
    parser = OptionParser()
    parser.add_option("-d", "--dds",
                      default='http://lambda.cds.caltech.edu/~adam/data/out/dp-plearn/orbit-pt256-40-n35s/results/orbit-pt256-40-n35s.discdds.pickle',
                      help="Diffeo System pickle file")

    
    (options, args) = parser.parse_args()
    dds = pickle.load(urllib.urlopen(options.dds))
    
    dds_info(dds)
    
    displ_field(dds.actions[0])
#    displ_field(dds.actions[1])
#    displ_field(dds.actions[2])
#    displ_field(dds.actions[3])
    
def dds_info(dds):
    print('DDS has %s actions of shape %s' % (len(dds.actions), str(dds.get_shape())))
    print('')
    print('\tIndex \tCommand')
    print('\t----------------------------------')
    for i, action in enumerate(dds.actions):
        cmd_s = str(action.original_cmd.tolist())
        print('\t' + str(i) + '\t' + cmd_s)

def displ_field(action):
    field = action.diffeo.d
    field_inv = action.diffeo_inv.d
    
    Y, X = np.meshgrid(range(field.shape[1]), range(field.shape[0]))

    D = np.zeros(X.shape + (2,))
    D[:, :, 0] = field[:, :, 0] - X
    D[:, :, 1] = field[:, :, 1] - Y
    
    D_inv = np.zeros(X.shape + (2,))
    D_inv[:, :, 0] = field_inv[:, :, 0] - X
    D_inv[:, :, 1] = field_inv[:, :, 1] - Y

    D, _ = smooth(D)
    D_inv, _ = smooth(D_inv)

    E = np.zeros(X.shape)
    E_inv = np.zeros(X.shape)
    
    for c in itertools.product(range(X.shape[0]), range(X.shape[1])):
        E[tuple(c)] = la.norm(D[c] + D_inv[tuple(D[c])]) / (.1 + la.norm(D))
        E_inv[tuple(c)] = la.norm(D_inv[c] + D[tuple(D[c])]) / (.1 + la.norm(D_inv[c]))
        
    Eln = np.zeros(E.shape)
    Eln_inv = np.zeros(E_inv.shape)
    for c in itertools.product(range(X.shape[0]), range(X.shape[1])):
        x0 = np.clip(c[0] - 1, 0, X.shape[0] - 1)
        x1 = np.clip(c[0] + 1, 0, X.shape[0] - 1)
        y0 = np.clip(c[1] - 1, 0, X.shape[1] - 1)
        y1 = np.clip(c[1] + 1, 0, X.shape[1] - 1)
        
        Eln[tuple(c)] = np.std(E[x0:x1, y0:y1])
        Eln_inv[tuple(c)] = np.std(E_inv[x0:x1, y0:y1])
    
    
    E = np.clip(E, 0, np.percentile(E, 99))
    E_inv = np.clip(E_inv, 0, np.percentile(E_inv, 99))
    
    pdb.set_trace()
    
    Image.fromarray(((Eln > np.std(Eln)) * 255).astype('uint8')).resize((400, 300)).show()
    Image.fromarray(((Eln_inv > np.std(Eln_inv)) * 255).astype('uint8')).resize((400, 300)).show()
    
    Image.fromarray((E / np.max(E) * 255).astype('uint8')).resize((400, 300)).show()
    Image.fromarray((E_inv / np.max(E_inv) * 255).astype('uint8')).resize((400, 300)).show()

def smooth(array, kernel=[[0, .25, 0], [.25, 0, .25], [0, .25, 0]]):
    array = np.array(array)
    kernel = np.array(kernel)
    result = np.zeros(array.shape)
    if len(array.shape) > 2:
        for i in range(array.shape[2]):
            result[:, :, i] = convolve2d(array[:, :, i], kernel, mode='same')
    else:
        result = convolve2d(array, kernel, mode='same')
        
    noise = array - result
    
    return result, noise
    
