from diffeoplan.library.algo.graphsearch_breadth import GraphSearchQueue

class GraphSearchDeep(GraphSearchQueue):
    def get_next_index(self, tree, open_nodes):
        return open_nodes[-1]
    