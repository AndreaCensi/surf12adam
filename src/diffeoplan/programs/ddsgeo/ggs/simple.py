""" Simple pieces for searching """
        
def deep_first(open_nodes):
    return open_nodes[-1]

def breadth_first(open_nodes):
    return open_nodes[0]

def contains(iterable, needle, comparison):
    if needle in iterable:
        return True
    for x in reversed(list(iterable)):
        if comparison(x, needle):
            return True
    return False
