from diffeoplan.library.images.uncertain_image import UncertainImage

from contracts import contract

class Node():
    
    @contract(y=UncertainImage, path='list(int)',
              parent='int', children='list(int)')
    def __init__(self, y, path, parent, children):
        self.y = y
        self.path = path
        self.parent = parent # index of the parent
        self.child_nodes = children

        self.alt_paths = [[]]

    def __repr__(self):
        return "Node(path=%s, parent=%s, children=%s)" % (self.path, self.parent, self.child_nodes)
