'''
Created on Sep 18, 2012

@author: adam
'''
#from . import logger
from PIL import Image #@UnresolvedImport
from diffeoplan.library.logs.rosbag.ros_conversions import imgmsg_to_pil
from scipy.signal import convolve2d
import numpy as np
import rosbag
import sys
import pdb
import matplotlib.pyplot as plt
from matplotlib.colors import Colormap, BoundaryNorm, LinearSegmentedColormap, \
    ColorConverter
from reprep import Report
from compmake.plugins import stats
from scipy import stats


print(sys.argv)

# Define kernels for differentiating with convolutions
ker_d1x = np.array([[0, 0, 0], [0, -1, 0], [0, 1, 0]])
ker_d2x = np.array([[0, .5, 0], [0, -1, 0], [0, .5, 0]])
ker_d1y = np.array([[0, 0, 0], [0, -1, 1], [0, 0, 0]])
ker_d2y = np.array([[0, 0, 0], [.5, -1, .5], [0, 0, 0]])

# Output data
S_d1x = None
S_d2x = None
S_d1y = None
S_d2y = None
#ovars = [[S_d1x, ker_d1x, '\frac{\partial Y}{\partial x}'],
#         [S_d2x, ker_d2x, '\frac{\partial^2 Y}{\partial x^2}'],
#         [S_d1y, ker_d1y, '\frac{\partial Y}{\partial y}'],
#         [S_d2y, ker_d2y, '\frac{\partial^2 Y}{\partial y^2}']]
ovars = [[S_d1x, ker_d1x, 'dYdx'],
         [S_d2x, ker_d2x, 'dYdx2'],
         [S_d1y, ker_d1y, 'dYdy'],
         [S_d2y, ker_d2y, 'dYdy2']]


topics = ['Y0']
n = 0 # Counter

# Handle input data
for bagfile in sys.argv[1:]:
    bag = rosbag.Bag(open(bagfile))
    
    for topic, msg, t in bag.read_messages(topics=topics):
        # Convert image_msg to numpy array via PIL
        img = np.array(imgmsg_to_pil(msg)[0])
        
        # Initiate shapes for output
        for i in range(len(ovars)):
            [S, _, _] = ovars[i]
            if S is None:
                size = img.shape[0:2]
                ovars[i][0] = np.zeros(size)
        
        # Merge img to one channel
        gray = np.mean(img, 2)
        # Calculate differences
    #    pdb.set_trace()
        for i in range(len(ovars)):
            [S, ker, _] = ovars[i]
            Si = convolve2d(gray, ker, boundary='wrap', mode='same')
            S += Si
        
        n += 1  # Refresh counter

print('Bag Processed')
#pdb.set_trace()
# Initiate report
report = Report('ImageStatistics')
report.text('summary', 'Statistics of images in processed bag')

vmax = 20
vmin = -20

for [S, _, name] in ovars:
    # Calculate mean value from sum
    Sn = S[2:-2, 2:-2] / n
    
    # Estimate parameters and plot analytic prob function
#    gkde = stats.gaussian_kde(Sn.flatten())

#    S_clip = np.zeros(Sn.shape)
#    S_clip[Sn < vmax] = Sn[Sn < vmax]
#    S_clip[Sn > vmin] = Sn[Sn > vmin]
#    pdb.set_trace()
    mu = np.mean(Sn)
    sigma = np.sqrt(np.std(Sn))
    f_mu = np.mean(np.abs(Sn))
    f_sigma = np.sqrt(np.std(np.abs(Sn)))
    report.data(name + 'mu', mu)
    report.data(name + 'sigma', sigma)
    report.data(name + 'folded_mu', f_mu)
    report.data(name + 'folded_sigma', f_sigma)
    def pdf(T):
        if len(T) is None:
            return 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(-.5 * ((t - mu) / sigma) ** 2)
        else:
            return np.array([1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(-.5 * ((t - mu) / sigma) ** 2) for t in T]) 
    
    x = np.linspace(0, vmax, 200)
#    pdb.set_trace()
    
#    Sn = np.abs(Sn)
    
    f = report.figure(cols=3)
    with f.plot(name, caption=name) as pylab:
        cax = pylab.imshow(np.clip(Sn, vmin, vmax), interpolation='nearest')
        pylab.colorbar(cax)
        
    with f.plot('hist' + name, caption=name) as pylab:
        cax = pylab.hist(Sn.flatten(), bins=(vmax - vmin) * 2, range=[vmin, vmax], normed=True)
        
#        pylab.plot(x, pdf(x), color='red')
        
    with f.plot('histexpr' + name, caption=name) as pylab:
        cax = pylab.hist(np.abs(Sn.flatten()), bins=(vmax - vmin) * 2, range=[0, vmax], normed=True)
        fpdf = np.array([pdf([t]) + pdf([-t]) for t in x])
        pylab.plot(x, fpdf, color='red')

# Save report
filename = '/home/adam/public_html/testrep.html'
report.to_html(filename)
