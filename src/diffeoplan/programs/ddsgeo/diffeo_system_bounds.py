from diffeoplan.library.analysis import DiffeoStructure
from diffeoplan.library.analysis.covering.diffeo_cover import DiffeoCover
from diffeoplan.library.discdds import DiffeoAction
from diffeoplan.library.analysis.covering.hard_choices import make_hard_choices

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
        
        
    def display(self, report):
        self.cover.draw_graph()
        
    def display_products(self, report, nsteps):
        for a in self.dds_hard.actions:
            f = report.figure(a.label, cols=nsteps)
            A = a
            for k in range(nsteps):
                A = DiffeoAction.compose(A, a)
                rgb = A.get_diffeo2d_forward().get_rgb_info()
                f.data_rgb('%s_%s' % (a.label, k), rgb)
    
