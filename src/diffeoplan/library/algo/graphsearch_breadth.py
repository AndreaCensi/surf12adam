from diffeoplan.library.graph.graph import Graph
from diffeoplan.library.algo.generic_graph_planner import GraphSearchQueue
from diffeoplan.library.algo.structured.structured_graph_planner import StructuredGraphPlanner
from diffeoplan.library.graph.node import Node

class GraphSearchBreadth(GraphSearchQueue):
    def get_next_index(self, tree, open_nodes):
        return open_nodes[0]
    
class GraphSearchBreadth2Dir(GraphSearchQueue):
    '''
    GraphSearch with expansion in 2 directions.
    '''
    
    def get_next_index(self, tree, open_nodes):
        return open_nodes[0]

    def init_goal_tree(self, y1, metric, thresh):
        """
        Override init_goal_tree in GenericGraphPlanner to 
        have one open node for expansion.
        """
        goal_node = Node(y=y1, path=[], parent=[], children=[])
        goal_tree = Graph(goal_node, metric, thresh)
        goal_tree.open_nodes = [0]
        return goal_tree
    
class GraphSearchBreadthStructured(StructuredGraphPlanner):
    def get_next_index(self, tree, open_nodes):
        return open_nodes[0]

    def init_goal_tree(self, y1, metric, thresh):
        """
        Override init_goal_tree in GenericGraphPlanner to 
        have one open node for expansion.
        """
        goal_node = Node(y=y1, path=[], parent=[], children=[])
        goal_tree = Graph(goal_node, metric, thresh)
        goal_tree.open_nodes = [0]
        return goal_tree
