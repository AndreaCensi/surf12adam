
from diffeoplan.library.algo.graphsearch import GenericGraphPlanner
from diffeoplan.library.graph.node import Node

class GraphSearchQueue(GenericGraphPlanner): # todo: move
    """  

    """
    
    def __init__(self,  thresh, metric, max_ittr, nsteps):
        GenericGraphPlanner.__init__(self, thresh, metric, max_ittr)
        self.open = None
        self.nsteps = nsteps
    
    def get_new_node(self, tree):
        # First time: I put the initial one
        if self.open is None:
            self.open = [0]
    
        toexpand = self.get_next_index(tree, open) 
        
        dds = self.get_dds()
        all_actions = range(len(dds.actions))
        available = tree.actions_available_for_node(toexpand, all_actions)
        assert len(available) >= 1
        
        next_cmd = self.get_next_cmd(tree, toexpand, available)
                 
        next_node = get_next_node(tree, toexpand, next_cmd, 
                                    dds=self.get_dds())
        
        # put the new one
        next_index = len(tree.nodes) # xxx don't like
        
        if len(self.path) < self.nsteps:
            self.open.append(next_index)
        
        if len(available) == 1:
            # this node is now closed
            self.open.remove(toexpand)
        
        return next_node

    def get_next_cmd(self, tree, node_index, available):
        return available[0]

    def get_next_index(self, tree, open_nodes):
        raise ValueError('to implement')
def get_next_node(tree, parent_index, cmd, dds): # todo: move
    parent = tree.nodes[parent_index]
    path = list(parent.path) + [cmd]
    next_action = dds.actions[cmd]
    y_new = next_action.predict(parent.y)
    return Node(y=y_new, 
                path=path,
                parent=parent_index,
                children=[])

class GraphSearchBreadth(GraphSearchQueue):
    def get_next_index(self, tree, open_nodes):
        return open_nodes[0]
    