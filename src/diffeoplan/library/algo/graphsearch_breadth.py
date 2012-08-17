from diffeoplan.library.graph.graph import Graph
from diffeoplan.library.algo.generic_graph_planner import GraphSearchQueue

class GraphSearchBreadth(GraphSearchQueue):
    def get_next_index(self, tree, open_nodes):
        return open_nodes[0]
    
class GraphSearchBreadth2Dir(GraphSearchQueue):
    '''
    GraphSearch with expansion in 2 directions.
    '''
    
    def get_next_index(self, tree, open_nodes):
        return open_nodes[0]

    def init_goal_tree(self, node, metric, thresh):
        """
        Override init_goal_tree in GenericGraphPlanner to 
        have one open node for expansion.
        """
        goal_tree = Graph(node, metric, thresh)
        goal_tree.open_nodes = [0]
        return goal_tree