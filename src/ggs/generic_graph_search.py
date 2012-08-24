import networkx as nx
from ggs.simple import  breadth_first
from abc import abstractmethod, ABCMeta

EDGE_REGULAR = 'regular'
EDGE_REDUNDANT = 'redundant'
EDGE_EQUIV = 'equiv'

class GenericGraphSearch:
    
    __metaclass__ = ABCMeta
    
    def __init__(self):
        self.G = nx.MultiGraph()
     
    def has_next(self):
        return len(self.open_nodes) > 0
        
    def go(self, first):
        self.init_search(first)
        while self.has_next():
            _ = self.do_iteration()
                   
    def init_search(self, first):
        self.iterations = 0
        self.G.add_node(first)
        self.open_nodes = [first]
        self.closed = set()

    def do_iteration(self):
        """ Returns the list of new nodes added """
        assert self.has_next()
        node = self.choose_open_nodes(self.open_nodes)
        self.log_chosen(node)
        assert node in self.open_nodes
        self.open_nodes.remove(node)
        self.closed.add(node)
        self.log_closed(node)
        children_accepted = self.expand_node(node)
        self.iterations += 1
        return children_accepted
    
    def expand_node(self, node):
        """ Returns the list of new nodes added """
        children_accepted = []
        for action in self.available_actions(node):
            child = self.next_node(node, action)
            self.log_child_generated(node, action, child)
            # first case: exactly the same node
            if child in self.G:
                self.G.add_edge(node, child, action=action, type=EDGE_REDUNDANT)
                self.log_child_existed(node, action, child)
                continue
            # check equivalent nodes
            matches = list(self.get_matches(child, self.G.nodes())) 
            if not matches:
                # this is a new node
                children_accepted.append(child)
                self.open_nodes.append(child)
                self.G.add_node(child)
                self.G.add_edge(node, child, action=action, type=EDGE_REGULAR)
                self.log_child_open(node, action, child)
            else:
                # add extra edges
                for m in matches:
                    # XXX: multi actions?
                    self.log_child_equivalent_found(node, action, child, m)
                    self.G.add_edge(node, m, action=action, type=EDGE_EQUIV)
                self.log_child_discarded(node, action, child, matches)
        return children_accepted
    
    @abstractmethod        
    def next_node(self, node, action):
        pass
    
    @abstractmethod        
    def available_actions(self, node):
        pass
    
    def choose_open_nodes(self, open_nodes):
        """ Chose one of the open nodes. """
        return breadth_first(open_nodes)
        
    def get_matches(self, n, nodes):
        """ Returns a list of equivalent nodes that match ``n``. """
        for n2 in nodes:
            if self.node_compare(n, n2):
                yield n2
    
    def node_compare(self, n1, n2):
        """ Subclass this for adding better conditions """
        return n1 == n2
            
    def log(self, s):
        print(s)
        
    def log_chosen(self, node):
        pass
    
    def log_closed(self, node):
        pass

    def log_child_open(self, node, action, child):
        pass
    
    def log_child_generated(self, node, action, child):
        pass

    def log_child_discarded(self, node, action, child, matches):
        pass
    
    def log_child_equivalent_found(self, node, action, child, match):
        pass
 
    def log_child_existed(self, node, action, child):
        pass

