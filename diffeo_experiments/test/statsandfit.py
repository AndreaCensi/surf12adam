import numpy as np
import numpy.linalg as la
import itertools
from scipy.optimize.minpack import leastsq
import pickle
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import pdb
#pdb.set_trace()
neig_y_stats = pickle.load(open('neig_y_stats[i].pickle'))
neig_esim_score = pickle.load(open('neig_esim_score[i].pickle'))
area = np.array([21, 29])
c_it = itertools.product(range(-area[0] / 2 + 1, area[0] / 2 + 1), range(-area[1] / 2 + 1, area[1] / 2 + 1))
dist_n = []
dist = []
for c in c_it:
    dist.append(c)
    dist_n.append(la.norm(c))
dist = np.array(dist)
dist_n = np.array(dist_n)

def cov_function(P0, A, X, Y):
    dX = Y - P0[0]
    dY = X - P0[1]
#                pdb.set_trace()
    pdist = np.sqrt(dX ** 2 + dY ** 2)
    return np.array(A[0] * (1 - np.exp(-A[1] * pdist)))

def residual_stats(A):
    return (np.array(A[0] * (1 - np.exp(-A[1] * dist_n))) - neig_y_stats) / (1 + dist_n)

def jacobian_func(A):
    A = np.array(A)
    return np.array([(1 - np.exp(-A[1] * dist_n)), A[0] * dist_n * (np.exp(-A[1] * dist_n))]).T


def residual_P(P, A):
    d = np.array([la.norm(P - di) for di in dist])
    return (np.array(A[0] * (1 - np.exp(-A[1] * d))) - neig_esim_score)# / (1 + dist_n)


A = leastsq(residual_stats, [1, 1], Dfun=jacobian_func)[0]
A = [400, .2]
print(A)
#                   
P0 = leastsq(residual_P, [0, 0], A)[0]
print(P0)
dist_n.reshape(area)

def plot3d(data, mesh=None, figure=None, alpha=1, linewidth=0.5):
    if figure is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    else:
        fig, ax = figure
        fig.hold(True)
        
    area = data.shape
    if mesh is None:
        X, Y = np.meshgrid(range(-area[1] / 2 + 1, area[1] / 2 + 1), range(-area[0] / 2 + 1, area[0] / 2 + 1))
    else:
        X, Y = mesh
        
    surf = ax.plot_surface(X, Y, data, linewidth=linewidth, alpha=alpha, rstride=1, cstride=1, antialiased=True, cmap=cm.jet)
    cset = ax.contourf(X, Y, data, zdir='z', offset=0, cmap=cm.jet)
    
#    if figure is None:
    ax.set_zlim3d([0, 600])
    return (fig, ax)

#fig, ax = plot3d(neig_y_stats.reshape(area))
#
#X, Y = np.meshgrid(range(-area[1] / 2 + 1, area[1] / 2 + 1), range(-area[0] / 2 + 1, area[0] / 2 + 1))
#est_score_rect = cov_function([0, 0], A, X, Y)
#print(est_score_rect)
#plot3d(est_score_rect, figure=fig, axis=ax)

## Score
plot = plot3d(neig_esim_score.reshape(area) - np.min(neig_esim_score))
plot = plot3d(neig_y_stats.reshape(area), figure=plot)

X, Y = np.meshgrid(range(-area[1] / 2 + 1, area[1] / 2 + 1), range(-area[0] / 2 + 1, area[0] / 2 + 1))
est_score_rect = cov_function([5, 0], A, X, Y)
print(est_score_rect)
plot3d(est_score_rect, figure=plot, alpha=0.5)

plt.show()

#pdb.set_trace()
#fig = plt.figure()
#ax = fig.add_subplot(121, projection='3d')
##            ax = fig.gca(projection='3d')
#X, Y = np.meshgrid(range(-area[1] / 2 + 1, area[1] / 2 + 1), range(-area[0] / 2 + 1, area[0] / 2 + 1))
#surf = ax.plot_surface(X, Y, neig_esim_score.reshape(area), linewidth=0, antialiased=False, cmap=cm.jet)
#cset = ax.contourf(X, Y, neig_esim_score.reshape(area),
#                   zdir='z', offset=0, cmap=cm.jet)
##            pdb.set_trace()
#ax.set_zlim3d([0, np.max(neig_esim_score)])
#
#
#est_score_rect = cov_function(P0, A, X, Y)
#ax = fig.add_subplot(122, projection='3d')
#X, Y = np.meshgrid(range(-area[1] / 2 + 1, area[1] / 2 + 1), range(-area[0] / 2 + 1, area[0] / 2 + 1))
#surf = ax.plot_surface(X, Y, est_score_rect, linewidth=0, antialiased=False, cmap=cm.jet)
##            cset = ax.contourf(X, Y, est_score_rect, zdir='z', offset=0, cmap=cm.jet)
##            ax.set_zlim3d([0, np.max(self.neig_esim_score[i, :])])
#
#pdb.set_trace()

#plt.savefig('out/peaks/pk' + str(i) + '.png')
#plt.clf()
