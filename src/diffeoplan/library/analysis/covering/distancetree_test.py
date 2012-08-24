from . import make_distancetree, make_distancesequence, np
from ggs import draw_node_graph
from diffeoplan.utils import construct_matrix
from geometry import assert_allclose
import itertools


def get_domain(H, W, noise=0):
    """ returns points, distance """
    points = []
    for i, j in itertools.product(range(H), range(W)):
        p = np.array((i, j))
        p = p + np.random.randn(2) * noise
        points.append(p) 

    distance = lambda p1, p2: np.linalg.norm(np.array(p1) - np.array(p2))
    N = H * W
    D = construct_matrix((N, N), lambda i, j: distance(points[i], points[j]))
    return points, D

def distancetree_test1():
    points, D = get_domain(1, 10)    

    order = make_distancesequence(D)
    for x in order:
        print('point: ', points[x])

    assert_allclose(points[order[0]], (0, 4))
    assert_allclose(points[order[1]], (0, 7))

    order, G = make_distancetree(D)
    
    filename = 'out/distancetree_test1'
    draw_graph(G, points, filename)

def draw_graph(G, points, filename):
    print('writing on %r' % filename)
    pos_func = lambda i: points[i]
    draw_node_graph(filename + '_level_order.png', G, pos_func,
                    color_func=lambda x: G.node[x]['level'],
                    label_func=lambda x: G.node[x]['order'],
                    cmap='Set2')

    draw_node_graph(filename + '_order_order.png', G, pos_func,
                    color_func=lambda x: G.node[x]['order'],
                    label_func=lambda x: G.node[x]['order'],
                    cmap='Set2')
    
    
def distancetree_test2():
    points, D = get_domain(10, 10, noise=0.5)    
    _, G = make_distancetree(D)
    filename = 'out/distancetree_test2'    
    draw_graph(G, points, filename)

def distancetree_test3():
    points, D = get_domain(10, 10, noise=0.2)    
    _, G = make_distancetree(D)
    filename = 'out/distancetree_test3'    
    draw_graph(G, points, filename)
