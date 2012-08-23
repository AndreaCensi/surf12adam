import networkx as nx

EDGE_REGULAR = 'regular'
EDGE_REDUNDANT = 'redundant'
EDGE_EQUIV = 'equiv'

class GenericGraphSearch:
    def __init__(self, choose_open_nodes, available_actions,
                       next_node, node_compare):
        self.choose_open_nodes = choose_open_nodes
        self.node_compare = node_compare
        self.available_actions = available_actions
        self.next_node = next_node
        
        self.G = nx.MultiGraph()
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
            self.iterations += 1
            
    def get_matches(self, n, nodes):
        """ Returns a list of equivalent nodes that match ``n``. """
        for n2 in nodes:
            if self.node_compare(n, n2):
                yield n2
            
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

