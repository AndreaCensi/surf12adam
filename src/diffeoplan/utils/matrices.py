import numpy as np

def construct_matrix(shape, function):
    from boot_agents.misc_utils.tensors_display import iterate_indices

    if len(shape) != 2:
        raise ValueError()
    D = np.zeros(shape) 
    for indices in iterate_indices(shape):
        result = function(*indices)
        if not isinstance(result, float):
            raise ValueError('%s(%s) = %s' % 
                             (function.__name__, indices, result))
        D[indices] = result
    return D
