from diffeoplan.library.algo.generic_graph_planner import GenericGraphPlanner
from diffeoplan.library.graph.node import Node

class GraphSearchQueue(GenericGraphPlanner):
    """  
    GenericGraphPlanner using a queue of open nodes for expansion.
    """
    
    def __init__(self, thresh, metric, max_ittr, nsteps):
        GenericGraphPlanner.__init__(self, thresh, metric, max_ittr)
#        self.start_open = None
        self.nsteps = nsteps
    
    def get_new_node(self, tree):
        assert(tree.open_nodes != None)
        if len(tree.open_nodes) == 0:
            return None
        
        toexpand = self.get_next_index(tree, tree.open_nodes) 
        
        dds = self.get_dds()
        all_actions = range(len(dds.actions))
        available = tree.actions_available_for_node(toexpand, all_actions)
        assert len(available) >= 1
        
        next_cmd = self.get_next_cmd(tree, toexpand, available)
                 
        next_node = get_next_node(tree, toexpand, next_cmd, #@UndefinedVariable
                                    dds=self.get_dds())
        
        # put the new one
        next_index = len(tree.nodes) # xxx don't like
        
        if len(next_node.path) < self.nsteps:
            tree.open_nodes.append(next_index)
        
        if len(available) == 1:
            # this node is now closed
            tree.open_nodes.remove(toexpand)
        
        assert next_node.__class__ == Node
        return next_node

    def get_next_cmd(self, tree, node_index, available): #@UnusedVariable
        return available[0]

    def expand_start_tree(self, start_tree):
        return self.get_new_node(start_tree)
    
    def expand_goal_tree(self, goal_tree):
        return self.get_new_node(goal_tree)

    def get_next_index(self, tree, open_nodes):
        raise ValueError('to implement')
#    
#def get_next_node(tree, parent_index, cmd, dds): # todo: move
#    parent = tree.nodes[parent_index]
#    path = list(parent.path) + [cmd]
#    next_action = dds.actions[cmd]
#    y_new = next_action.predict(parent.y)
#    return Node(y=y_new,
#                path=path,
#                parent=parent_index,
#                children=[])
#    
    
