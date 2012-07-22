from . import contract
import numpy as np
from diffeoplan.library.square_domain import SquareDomain
from diffeoplan.library.sym_diffeo import SymbolicDiffeo
from diffeoplan.library.coordinate_change import LinearCoordinateChange
from boot_agents.diffeo.misc_utils import coords_iterate
 

@contract(diffeo=SymbolicDiffeo, viewport=SquareDomain, shape='valid_2d_shape',
          returns='tuple(valid_diffeomorphism, array[HxW])')
def diffeo_from_function_viewport(diffeo, manifold, viewport, shape):
    domain = SquareDomain([[0, shape[0]], [0, shape[1]]]) 
    domain2viewport = LinearCoordinateChange(domain.bounds, viewport.bounds)
    viewport2domain = domain2viewport.get_inverse()
    # Prepare the grid
    M = shape[0]
    N = shape[1]
    D = np.zeros((M, N, 2), dtype='int32')
    # Uncertainty (here, 0 or 1)
    info = np.zeros((M, N), dtype='float32')
    # Iterate at each cell
    for coords in coords_iterate(shape):
        # coords = (i,j)
        # coordinates of the center of the cell
        center_offset = np.array([0.5, 0.5])
        cell_center = coords + center_offset
        # the domain is ([0,M]x[0,N]) so it is still inside
        assert domain.belongs(cell_center)
        # transform to the "world"
        world = domain2viewport(cell_center)
        # apply the diffeomorphism
        world2 = manifold.normalize(diffeo(world)) 
        # Are we inside the viewport?
        if viewport.belongs(world2):
            cell2 = viewport2domain(world2)
            assert domain.belongs(cell2)
            new_cell = np.floor(cell2)
            approx = np.linalg.norm(cell2 - (new_cell + center_offset))
            info[coords[0], coords[1]] = np.exp(-approx)
            D[coords[0], coords[1], 0] = new_cell[0]
            D[coords[0], coords[1], 1] = new_cell[1]
        else:
            # we don't know
            info[coords[0], coords[1]] = 0
            # we put the identity as a dummy value
            D[coords[0], coords[1], 0] = coords[0]
            D[coords[0], coords[1], 1] = coords[1]
    D2 = np.transpose(D, (1, 0, 2))
    info2 = np.transpose(info, (1, 0))
    return D2, info2
