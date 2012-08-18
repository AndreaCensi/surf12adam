import matplotlib.pyplot as plt
import networkx as nx
import os


def draw_node_graph(filename, G, pos_func,
                    color_func=lambda _: 0.5,
                    label_func=lambda x: str(x)):
    pos = dict((n, pos_func(n)) for n in G)
    
    all_positions = [tuple(pos_func(n)) for n in G] 
    if len(all_positions) != len(set(all_positions)):
        print('Warning, overlapping nodes') 
        pass
 
    node_color = map(color_func, G) 
    labels = dict((n, label_func(n)) for n in G)
    
    fig = plt.figure() 
    nx.draw_networkx(G, with_labels=True, pos=pos, labels=labels,
                     node_color=node_color)
    dirname = os.path.dirname(filename)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    plt.savefig(filename)
    plt.close(fig)
    
    
