import matplotlib.pyplot as plt
import networkx as nx
import os
import collections
from reprep.plot_utils.axes import turn_all_axes_off


def draw_node_graph(filename, G, pos_func,
                    color_func=lambda _: 0.5,
                    label_func=lambda x: str(x)):
    
    nodes = list(G)
    pos = dict((n, pos_func(n)) for n in nodes)
    
    all_positions = [tuple(pos_func(n)) for n in nodes] 

    if len(all_positions) != len(set(all_positions)):
        print('Warning, overlapping nodes')
        y = collections.Counter(all_positions)
        for p, num in y.items():
            if num > 1:
                print('- %d for %s' % (num, p))
                for node, node_pos in pos.items():
                    if tuple(node_pos) == p:
                        print('  - %s ' % str(node))
        pass
 
    node_color = map(color_func, G) 
    labels = dict((n, label_func(n)) for n in G)
    
    fig = plt.figure() 
    nx.draw_networkx(G, with_labels=True, pos=pos, labels=labels,
                     node_color=node_color)
    
    plt.colorbar()
    turn_all_axes_off(plt)

    dirname = os.path.dirname(filename)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    plt.savefig(filename)
    plt.close(fig)
    
    
    
