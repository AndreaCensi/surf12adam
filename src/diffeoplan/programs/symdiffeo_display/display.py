from boot_agents.diffeo.diffeo_display import (diffeo_to_rgb_norm,
    diffeo_to_rgb_angle, diffeo_to_rgb_curv)
from boot_agents.diffeo.diffeo_estimator import Diffeomorphism2D
from boot_agents.diffeo.tests.diffeo_creation_test import (CurvedPylab, plot_grid,
    plot_smiley)
from diffeoplan.library.square_domain import SquareDomain
from diffeoplan.library.viewport import diffeo_from_function_viewport
from reprep.graphics.filter_scale import scale
from reprep.plot_utils.axes import turn_all_axes_off
import numpy as np
from contracts import contract


@contract(resolution='R,int,>0', template_rgb='array[RxRx3](uint8)')
def symdiffeo_display(report, template_rgb, diffeo, resolution): #@UnusedVariable
    shape = [resolution, resolution]
    
    # This must be in the -1,1 -> -1,1 domain
    #change_basis = LinearCoordinateChange([[0, 1], [0, 1]],
    #                                      [[-1, +1], [-1, +1]])
    #diffeo2 = change_basis.conjugate(diffeo)
    
    viewport = SquareDomain([[-1, +1], [-1, +1]])
    manifold = diffeo.get_topology()
    D, Dinfo = diffeo_from_function_viewport(diffeo, manifold, viewport, shape)
    D2d = Diffeomorphism2D(D, Dinfo)

    def function(p):
        q = diffeo(np.array(p))
        q = manifold.normalize(q)
        return (q[0], q[1])        
   
    fig = report.figure(cols=4)
    

    M = 1
    n1 = 10
    n2 = 100
    bx = [-.99, +.99] # depends on fmod
    by = bx
    params = dict(figsize=(3, 3))

    def common_settings(pylab):
        pylab.axis('equal')
        pylab.axis((-M, M, -M, M))
        turn_all_axes_off(pylab)

    with fig.plot('grid1', **params) as pylab:
        curved = CurvedPylab(pylab, function)
        plot_grid(curved, n1=n1, n2=n2, bx=bx, by=by, hcol='k.', vcol='k.')
        common_settings(pylab)

    with fig.plot('grid2', caption="different colors", **params) as pylab:
        curved = CurvedPylab(pylab, function)
        plot_grid(curved, n1=n1, n2=n2, bx=bx, by=by, hcol='r.', vcol='b.')
        common_settings(pylab)

    with fig.plot('grid3', caption="smiley (might be deformed in some cases)",
                  **params) as pylab:
        curved = CurvedPylab(pylab, function)
        plot_grid(curved, n1=n1, n2=n2, bx=bx, by=by, hcol='r.', vcol='b.')
        common_settings(pylab)
        plot_smiley(curved)

    with fig.plot('grid4', caption="smiley (might be deformed in some cases)",
                   **params) as pylab:
        curved = CurvedPylab(pylab, function)
        plot_grid(curved, n1=n1, n2=n2, bx=bx, by=by, hcol='k.', vcol='k.')
        common_settings(pylab)
        plot_smiley(curved, '0.5')

    fig = report.figure(cols=4, caption='Visualization of the diffeomorphism.'
                        'Note: must transpose rgb data on diagonal to make it coherent with'
                        ' plots above.')
    
    report.data_rgb('norm_D', diffeo_to_rgb_norm(D),
                    caption="white=0, blue=maximum. Note: wrong in case of wraparound").add_to(fig)
    report.data_rgb('phase_D', diffeo_to_rgb_angle(D),
                    caption="note: wrong in case of wraparound").add_to(fig)

    def info2rgb(x):
        return scale(x, max_value=1, min_value=0,
                    min_color=[1, 0, 0], max_color=[0, 1, 0])
    
    report.data_rgb('D_info', info2rgb(Dinfo),
                     caption='Information (green=sure, red=unknown)').add_to(fig)

    report.data_rgb('curvD', diffeo_to_rgb_curv(D)).add_to(fig)


    
    if template_rgb is not None:
        
        iterations = 3
        
        im = template_rgb
        var = np.ones((im.shape[0], im.shape[1]))
        
        data = [(im, var)]
        for it in range(iterations):
            im, var = D2d.apply(im, var)
            data.append((im, var))
    
        k = 0
        for im, var in data:
            fig = report.figure(cols=4,
                                caption='Effect on template (iteration %d)' % k)
            
            fig.data_rgb('im%d' % (it + 1), im)
            fig.data_rgb('var%d' % (it + 1), info2rgb(var))
            
            k += 1
    
    
    
    
