


class Node():
    def __init__(self, y, path, children=None):
        self.y = y
        self.path = path
        self.alt_paths = [[]]
        self.parent = None
        if children is None:
            children = []
        self.child_nodes = children
