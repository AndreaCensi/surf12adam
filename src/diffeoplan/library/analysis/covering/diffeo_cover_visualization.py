from . import logger, np, contract
from diffeoplan.utils import  assert_allclose
from geometry import mds
from ggs import (EDGE_EQUIV, EDGE_REGULAR, EDGE_REDUNDANT)
from networkx.algorithms.shortest_paths.dense import floyd_warshall_numpy
from networkx.classes.multigraph import MultiGraph
from reprep.plot_utils import turn_all_axes_off
import matplotlib


type2color = {}
type2color[frozenset([EDGE_REGULAR])] = [0, 0, 0]
type2color[frozenset([EDGE_REDUNDANT])] = [.8, .8, .8]
type2color[frozenset([EDGE_EQUIV])] = [0, 1, 0]

type2color[frozenset([EDGE_REGULAR, EDGE_REDUNDANT])] = [0, 0, 0] # regular wins
type2color[frozenset([EDGE_EQUIV, EDGE_REDUNDANT])] = [0, 1, 0] # equiv wins
type2color[frozenset([EDGE_EQUIV, EDGE_REGULAR])] = [0, 0, 0] # regular wins
type2color[frozenset([EDGE_REGULAR, EDGE_REDUNDANT, EDGE_REGULAR])] = [0, 0, 0] # regular wins

@contract(G=MultiGraph)
def edges_type_to_color(G, n1, n2):
    edges = G[n1][n2]
    assert len(edges)
    edge_types = frozenset([e['type'] for e in edges.values()])                        
    return type2color[edge_types]

def get_edges_points_and_color(G, plan2point, edges2color):
    """
        Assumes G multigraph
        plan2point: node -> point
    """
    for plan1 in G:
        for plan2 in G[plan1]:
            pi = plan2point(plan1)
            pj = plan2point(plan2)
            yield pi, pj, edges2color(plan1, plan2)


def plot_3d_graph(pylab, G, plan2point, plan2color, edges2color=None):
    if edges2color is None:
        edges2color = lambda n1, n2: [0, 0, 0] #@UnusedVariable

    cm = matplotlib.cm.get_cmap('RdYlBu')
    import mpl_toolkits.mplot3d.axes3d as plot3
    fig = pylab.gcf()
    ax = plot3.Axes3D(fig)
    for pi, pj, ec in get_edges_points_and_color(G, plan2point, edges2color):
        coords = np.vstack((pi, pj)).T
        ax.plot3D(xs=coords[0], ys=coords[1], zs=coords[2],
                      linestyle='-', color=ec)
    n = len(G)
    P = np.array(map(plan2point, G)).T
    assert_allclose(P.shape, (3, n))
    color = map(plan2color, G)
    
    patches = ax.scatter3D(P[0], P[1], P[2], s=40, c=color, cmap=cm)
    try:                    
        fig.colorbar(patches)
    except TypeError as e:
        logger.error('Cannot use colorbar')
        logger.exception(e)

def plot_2d_graph(pylab, G, plan2point, plan2color, edges2color=None, s=120):
    if edges2color is None:
        edges2color = lambda n1, n2: [0, 0, 0] #@UnusedVariable
        
    cm = matplotlib.cm.get_cmap('RdYlBu')
        
    for pi, pj, ec in get_edges_points_and_color(G, plan2point, edges2color):
        coords = np.vstack((pi, pj)).T
        pylab.plot(coords[0], coords[1], linestyle='-', color=ec, zorder=1)
        
    n = len(G)
    P = np.array(map(plan2point, G)).T
    assert_allclose(P.shape, (2, n))
    color = map(plan2color, G)
    pylab.scatter(P[0], P[1], s=s, c=color, cmap=cm, zorder=3)
    pylab.axis('equal')            
    turn_all_axes_off(pylab)
    pylab.colorbar()


def get_nodes_distance_matrix(G, nodelist, weight_field=None):
    n = len(nodelist)
    D = floyd_warshall_numpy(G, nodelist, weight=weight_field)
    assert_allclose(D.shape, (n, n))
    assert_allclose(np.isfinite(D), True)
    assert np.all(D >= 0)
    # there was some strange bug. Keep this even if you don't understand why
    D = np.array(D, dtype='float64')
    return D
    
    
@contract(plans='seq[N]', D='array[NxN]', ndim='K',
          returns='dict(seq: array[K])')
def get_embedding_mds(plans, D, ndim):
    """ Returns a dictionary plan -> position """
    plan2index = dict([(p, i) for i, p in enumerate(plans)])
    p2 = mds(D, ndim)
    plan2point = dict([(p, p2[:, plan2index[p]]) for p in plans])
    return plan2point
    
