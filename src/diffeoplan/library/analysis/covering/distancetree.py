from contracts import contract
import networkx as nx
import numpy as np

@contract(D='array[NxN](>=0)', returns='list(int,>=0,<N)')
def make_distancesequence(D):
    """ Returns the sequence of indices that give best coverage. """
    N = D.shape[0]
    fixed = list()
    remaining = range(N)
    
    while remaining: 
        obj = [get_objective(D, fixed + [j]) for j in remaining]
        jbest = remaining[np.argmin(obj)]
        fixed.append(jbest)
        remaining.remove(jbest)
        
    return fixed
    
    
@contract(D='array[NxN]', fixed='list(int)')
def get_objective(D, fixed):
    N = D.shape[0]
    T = len(fixed)
    Ds = D[:, fixed]
    assert Ds.shape == (N, T)
    best = np.min(Ds, axis=1)
    assert best.size == N
    score = np.sum(best)

    #    print('-------------------')
    #    printm('fixed', fixed)
    #    printm('Ds', Ds, 'best', best.reshape(N, 1))
    #    print('score: %s' % score)
    #    
    return score
    

@contract(D='array[NxN](>=0)', returns='tuple(list(int,>=0,<N), *)')
def make_distancetree(D, nodes, max_branching):
    """ Returns the sequence of indices that give best coverage
        and a tree. """
        
    def get_closest_node_index(i, possibilities):
        Dslice = D[i, possibilities]
        best = np.argmin(Dslice)
        closest = possibilities[best]
        return closest
                    
    G = nx.Graph()
    fixed = []
    
    def available_to_link():
        """ Returns the subset in fixed that is available
            to link because the branching factor is small """
        ok = lambda i: len(G[nodes[i]]) < max_branching # num_edges
        return [i for i in fixed if ok(i)]
        
    for i in make_distancesequence(D):
        n_i = nodes[i]
        G.add_node(n_i)
        G.node[n_i]['order'] = len(fixed)
        if fixed: 
            available = available_to_link()
            linkto = get_closest_node_index(i, available)
            
            n_closest = nodes[linkto]
            G.add_edge(n_closest, n_i)
            G.node[n_i]['level'] = G.node[n_closest]['level'] + 1
        else:
            G.node[n_i]['level'] = 0
        fixed.append(i)
    return fixed, G
    

