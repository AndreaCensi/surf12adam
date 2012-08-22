from . import DiffeoCover
from diffeoplan.library.discdds import DiffeoAction
from diffeoplan.library.discdds.diffeo_structure import DiffeoStructure
import copy
from geometry.mds_algos import mds
from boot_agents.diffeo.diffeo_display import diffeo_stats
from boot_agents.diffeo.diffeomorphism2d import Diffeomorphism2D


class DiffeoSystemBounds:
    def __init__(self, id_dds, dds, tolerance, info_threshold, min_visibility, debug_it, max_it):
        self.dds = dds
        self.dds_hard = make_hard_choices(dds, info_threshold)
        
        self.ds = DiffeoStructure(dds, tolerance)
        
        self.cover = DiffeoCover(id_dds, self.dds_hard, self.ds,
                                 info_threshold=info_threshold,
                                 min_visibility=min_visibility,
                                 debug_it=debug_it,
                                 max_it=max_it)
        self.cover.go()
        print('Computing distances..')
        plans, D = self.cover.get_distance_matrix()
        print('done')
        
        
    def display(self, report):
        self.cover.draw_graph()
        #self.ds.display(report.section())

        # TODO: remove all of this
        f = report.figure('distances')
        plans, D = self.cover.get_distance_matrix()
        #Dn = D / self.ds.scalew
        Dn = D
        caption = 'min: %g max %g ' % (Dn.min(), Dn.max())
        report.data('D', Dn).display('scale').add_to(f, caption)
        with f.plot('D_stats') as pylab:
            pylab.hist(D.flat)
        
        if D.shape[0] < 20:
            report.table('D_table', Dn, fmt='%.1f')
        
        p2 = mds(D, 2)
        p3 = mds(D, 3)
        color = [self.cover.visibility(plan) for plan in plans]    
            
        f = report.figure('embedding')
        with f.plot('2D') as pylab:
            pylab.scatter(p2[0], p2[1], c=color)
            pylab.axis('equal')

        with f.plot('3D') as pylab:
            import mpl_toolkits.mplot3d.axes3d as plot3
            fig = pylab.gcf()
            ax = plot3.Axes3D(fig)
            ax.scatter(p3[0], p3[1], p3[2], c=color)
                        
        self.display_products(report, nsteps=10)

    def display_products(self, report, nsteps):
        for a in self.dds_hard.actions:
            f = report.figure(a.label, cols=nsteps)
            A = a
            for k in range(nsteps):
                A = DiffeoAction.compose(A, a)
                rgb = A.get_diffeo2d_forward().get_rgb_info()
                f.data_rgb('%s_%s' % (a.label, k), rgb)
    
import numpy as np

def make_hard_choices(dds, info_threshold, use_isomorphism_heuristics=True,
                      info_percentile=50): 
    dds = copy.deepcopy(dds)
    
    def make_hard(dd):
        print('--------')
        assert isinstance(dd, Diffeomorphism2D)
        dd.variance = (dd.variance > info_threshold).astype('float')
        if use_isomorphism_heuristics:
            stats = diffeo_stats(dd.d)
            limit = np.percentile(stats.norm, info_percentile) / 3.0
#            print('norm mean/mean: %g %g' % (np.mean(stats.norm), np.median(stats.norm)))            
#            for i in range(0, 100, 5):
#                print(' %3d%% = %g' % (i, np.percentile(stats.norm, i)))
#            limit = np.percentile(stats.norm, info_percentile)
#            if limit <= 1:
#                print('limit was %g' % limit)
#                limit = 4
            dd.variance = (stats.norm > limit).astype('float')
            print('limit: %g pixels' % limit)
            print('visibility: %g' % np.mean(dd.variance))
    for a in dds.actions:
        df = a.get_diffeo2d_forward()
        make_hard(df)
        db = a.get_diffeo2d_backward()
        make_hard(db)
    return dds
