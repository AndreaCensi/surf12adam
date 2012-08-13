


class Node():
    def __init__(self, y, path):
        self.y = y
        self.path = path
        self.alt_paths = [[]]
        self.parent = None
        self.child_nodes = []