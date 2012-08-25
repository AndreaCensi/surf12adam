import numpy as np

def construct_matrix_iterators(iterators, function):
    
    elements = map(list, iterators)
    shape = map(len, elements)
    
    def element(*args): # args is a tuple of indices = (i, j, k, ...)
        assert len(args) == len(shape)
        combination = [a[i] for a, i in zip(elements, args)]
        return function(*combination)
        
    return construct_matrix(shape, element)
    
           
def construct_matrix(shape, function):
    from boot_agents.misc_utils.tensors_display import iterate_indices
    ndim = len(shape)
    if ndim != 2:
        msg = 'Sorry, not implemented for ndim != 2 (got %d).' % ndim
        raise NotImplementedError(msg)
    D = np.zeros(shape) 
    for indices in iterate_indices(shape):
        result = function(*indices)
        if not isinstance(result, float):
            raise ValueError('%s(%s) = %s' % 
                             (function.__name__, indices, result))
        D[indices] = result
    return D
