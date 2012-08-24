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
def make_distancetree(D):
    """ Returns the sequence of indices that give best coverage
        and a tree. """
    G = nx.Graph()
    fixed = []
    for i in make_distancesequence(D):
        G.add_node(i)
        G.node[i]['order'] = len(fixed)
        if fixed: 
            distance_to_fixed = D[i, fixed]
            best = np.argmin(distance_to_fixed)
            closest = fixed[best]
            G.add_edge(closest, i)
            G.node[i]['level'] = G.node[closest]['level'] + 1
        else:
            G.node[i]['level'] = 0
        fixed.append(i)
    return fixed, G
    
