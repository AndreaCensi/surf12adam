'''
Created on Jan 24, 2013

@author: adam
'''
from . import logger
from diffeoplan.programs.utils import declare_command
import pdb
import numpy as np
import numpy.linalg as la
import itertools
import pylab
from PIL import Image #@UnresolvedImport

@declare_command('covariance', 'covariance  [-s <stream>]')
def covariance(config, parser):
    """ Calculate the covariance of the pixels in the images in the logs"""
    parser.add_option("-s", "--stream", help="ID stream.", default='orbit-pt256-40')
    options, _ = parser.parse()
    
    stream = config.streams.instance(options.stream)
    
    sum_X = None
    sum_XXT = None
    num_sample = 0
    
    # Go through all images y0
    for data in stream.read_all():
#        if num_sample > 100:
#            break
        img = data.y0
        # Go through each channel in the image
        for c in range(img.shape[2]):
            num_sample += 1
            layer = img[:, :, c].astype('float')
            X = np.mat(layer.flatten()).T
            if sum_X is None or sum_XXT is None:
                sum_X = np.mat(np.zeros(X.shape))
                sum_XXT = np.mat(np.zeros((X.shape[0], X.shape[0])))
                                 
            sum_X += X
            sum_XXT += X * X.T
            logger.info('cov samples ' + str(num_sample))
            
    cov_M = sum_XXT / num_sample - sum_X * sum_X.T / (num_sample * num_sample)
    
    
    
    
    max_cov = np.max(cov_M)
    min_cov = np.min(cov_M)
    max_val = np.max((max_cov, np.abs(min_cov)))
    
    pos = np.clip(cov_M, 0, max_cov) / max_val
    neg = np.clip(cov_M, min_cov, 0) / max_val
    
    red = (-neg * 255).astype('uint8')
    green = (pos * 255).astype('uint8')
    blue = ((1 + neg - pos) * 255).astype('uint8')
    
    shape = np.flipud(layer.shape)
    megashape = np.array(shape) ** 2
    
    megaimage = Image.new('RGB', megashape)
    coords = tuple(itertools.product(range(layer.shape[0]), range(layer.shape[1])))
    for i, coord in enumerate(coords):
        covimg = np.zeros(img.shape, dtype=np.uint8)
        covimg[:, :, 0] = red[i, :].reshape(layer.shape)
        covimg[:, :, 1] = green[i, :].reshape(layer.shape)
        covimg[:, :, 2] = blue[i, :].reshape(layer.shape)
        megaimage.paste(Image.fromarray(covimg), tuple(np.flipud(coord) * shape))
    
    megaimage.save('megaimage.png')
    pylab.figure()
    pylab.imshow(covimg)
    pylab.savefig('covimg.png')
    
    pdb.set_trace()
    
    
    logger.info('Processing')
    
    DX = np.zeros(cov_M.shape)
    DY = np.zeros(cov_M.shape)
    D = np.zeros(cov_M.shape)
    
    
    
    for r in range(sum_X.size):
        for c in range(sum_X.size):
            dy, dx = np.array(coords[r]) - np.array(coords[c])
            DX[r, c] = dx
            DY[r, c] = dy
            D[r, c] = la.norm([dx, dy])
    
    
    
    dists = np.unique(D)
    dists_data = []
    covs_mean = np.zeros(dists.shape)
    covs_std = np.zeros(dists.shape)
    
    for d in dists:
        dists_data.append(cov_M[D == d]) 

    for i in range(len(dists)):
        covs_mean[i] = np.mean(dists_data[i])
        covs_std[i] = np.std(dists_data[i])
        
    pylab.figure()
    pylab.hold(True)
    pylab.plot(dists, covs_mean)
    pylab.plot(dists, covs_mean + covs_std, 'r*')
    pylab.plot(dists, covs_mean - covs_std, 'r*')
    pylab.savefig('covfunc.png')
    pdb.set_trace()
