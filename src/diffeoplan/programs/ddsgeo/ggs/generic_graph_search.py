import networkx as nx
from . import contains

class GenericGraphSearch:
    def __init__(self, choose_open_nodes, available_actions,
                       next_node, node_compare):
        self.choose_open_nodes = choose_open_nodes
        self.node_compare = node_compare
        self.available_actions = available_actions
        self.next_node = next_node
        
        self.G = nx.Graph()
        self.iterations = 0
         
    def go(self, first):
        self.G.add_node(first)
        self.open_nodes = [first]
        self.closed = set()
        
        while self.open_nodes:
            node = self.choose_open_nodes(self.open_nodes)
            self.log_chosen(node)
            assert node in self.open_nodes
            self.open_nodes.remove(node)
            self.closed.add(node)
            self.log_closed(node)
            for action in self.available_actions(node):
                child = self.next_node(node, action)
                self.log_child_generated(node, child)
                if (not contains(self.open_nodes, child, self.node_compare) and
                    not contains(self.closed, child, self.node_compare)):
                    self.open_nodes.append(child)
                    self.G.add_node(child)
                    self.G.add_edge(node, child, action=action)
                    self.log_child_open(node, child)
                else:
                    self.log_child_discarded(node, child)
            self.iterations += 1
            
    def log(self, s):
        print(s)
        
    def log_chosen(self, node):
        pass
    
    def color_open_closed(self, n):
        if n in self.closed:
            return 0.6
        else:
            return 0.8

    def log_closed(self, node):
        pass

    def log_child_open(self, node, child):
        pass
    
    def log_child_generated(self, node, child):
        pass

    def log_child_discarded(self, node, child):
        pass
    
    
 
