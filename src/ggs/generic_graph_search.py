import networkx as nx
from ggs.simple import  breadth_first
from abc import abstractmethod, ABCMeta
from contracts import contract

EDGE_REGULAR = 'regular'
EDGE_REDUNDANT = 'redundant'
EDGE_EQUIV = 'equiv'

class GenericGraphSearch(object):
    
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
        self.num_created = 0
        self.num_redundant = 0
        self.num_collapsed = 0
        self.G.add_node(first)
        self.open_nodes = [first]
        self.closed = set()
        
    @contract(returns='int,>=0')
    def get_num_closed(self):
        return len(self.closed)
    
    @contract(returns='int,>=0')
    def get_num_open(self):
        return len(self.open_nodes)
    
    @contract(returns='int,>=0')
    def get_num_created(self):
        """ Number of nodes created using next_node() """
        return self.num_created

    @contract(returns='int,>=0')
    def get_num_created_but_redundant(self):
        """ 
            Number of nodes that were created, but 
            found to be an exact duplicate. 
            equivalent to __eq__() used implicitly)
        """
        return self.num_redundant

    @contract(returns='int,>=0')
    def get_num_created_but_collapsed(self):
        """
            Returns the number of nodes that were created, but get_matches()
            found a match.self.get_matches().
        """ 
        return self.num_collapsed
        
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
        actions = self.available_actions(node)
        if not actions:
            self.log_actionless_node(node)
            return []
        for action in actions:
            child = self.next_node(node, action)
            self.num_created += 1
            self.log_child_generated(node, action, child)
            # first case: exactly the same node
            if child in self.G:
                self.G.add_edge(node, child, action=action, type=EDGE_REDUNDANT)
                self.log_child_existed(node, action, child)
                self.num_redundant += 1
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
                self.num_collapsed += 1
                
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
    
    def node_friendly(self, n1):
        """ Returns a string description of the node. """
        return n1.__str__()
            
    def log(self, s): # TODO: add private log
        print(s)
        
    def log_chosen(self, node):
        pass
    
    def log_closed(self, node):
        pass

    def log_actionless_node(self, node):
        """ This node has no actions. """
        
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

