from . import DiffeoCover
from diffeoplan.library.discdds import DiffeoAction
import copy
from diffeoplan.library.discdds.diffeo_structure import DiffeoStructure
from diffeoplan.utils.matrices import construct_matrix


class DiffeoSystemBounds:
    def __init__(self, id_dds, dds, info_threshold):
        self.dds = dds
        self.dds_hard = make_hard_choices(dds, info_threshold)
        
        self.ds = DiffeoStructure(dds)
        
        self.cover = DiffeoCover(id_dds, self.dds_hard, self.ds,
                                 info_threshold=info_threshold)
        self.cover.go()
        

    def display(self, report):
        self.cover.draw_graph()
        #self.ds.display(report.section())

        f = report.figure('distances')
        D = self.cover.get_distance_matrix()
        Dn = D / self.ds.scalew
        caption = 'min: %g max %g ' % (Dn.min(), Dn.max())
        report.data('Dn', Dn).display('scale').add_to(f, caption)
        
        report.table('D', Dn, fmt='%.1f')
        
        self.display_products(report, nsteps=10)

    def display_products(self, report, nsteps):
        for a in self.dds_hard.actions:
            f = report.figure(a.label, cols=nsteps)
            A = a
            for k in range(nsteps):
                A = DiffeoAction.compose(A, a)
                rgb = A.get_diffeo2d_forward().get_rgb_info()
                f.data_rgb('%s_%s' % (a.label, k), rgb)
    

def make_hard_choices(dds, info_threshold):
    dds = copy.deepcopy(dds)
    
    def make_hard(dd):
        dd.variance = (dd.variance > info_threshold).astype('float')
    for a in dds.actions:
        df = a.get_diffeo2d_forward()
        make_hard(df)
        db = a.get_diffeo2d_backward()
        make_hard(db)
    return dds
