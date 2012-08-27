
    
    
def copy_nodes_attrs(Gfrom, Gto): # TODO: move away
    """ Copies the nodes attributes from one to the other """
    for node in Gto:
        if not node in Gfrom:
            print('warning, node %s not found in other one.' % str(node))
        for k, v in Gfrom.node[node].items():
            Gto[k] = v
            
            
    
