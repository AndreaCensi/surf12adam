from contracts import contract
from diffeoplan.library.algo.planning_algo import DiffeoPlanningAlgo
from diffeoplan.library.algo.planning_result import PlanningResult
from diffeoplan.library.analysis import (DiffeoCover, DiffeoStructure,
    make_hard_choices)
from diffeoplan.library.discdds import DiffeoAction, DiffeoSystem
from diffeoplan.library.images import UncertainImage
from reprep import Report


class InformedTree(DiffeoPlanningAlgo):
    
    def __init__(self, diffeo_structure_params,
                        diffeo_cover_params,
                        hard_choice_params):
        DiffeoPlanningAlgo.__init__(self)
        self.diffeo_structure_params = diffeo_structure_params
        self.diffeo_cover_params = diffeo_cover_params
        self.hard_choice_params = hard_choice_params
        
    @contract(dds=DiffeoSystem)
    def init(self, id_dds, dds):
        DiffeoPlanningAlgo.init(self, dds)
        self.ds = DiffeoStructure(dds=dds, **self.diffeo_structure_params)
        self.dds_hard = make_hard_choices(dds, **self.hard_choice_params)
        self.cover = DiffeoCover(id_dds, self.dds_hard, self.ds, **self.diffeo_cover_params)
        self.cover.go()
        # precompute and save all distances
        self.cover.compute_distances()
#        self.cover.G = None
        
    @contract(report=Report)
    def init_report(self, report):
        """ Creates a report for the initialization phase. """
        self.display_products(report.section('actions'), 12)
        self.ds.display(report.section('diffeo_structure'))
        self.cover.draw_embeddings(report.section('embeddings'))
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): 
        return PlanningResult(success=False, plan=None, status='Not implemented')

    def display_products(self, report, nsteps):
        # XXX: make separate
        for i, a in enumerate(self.dds_hard.actions):
            f = report.figure('cmd%s' % i, cols=nsteps)
            A = a
            for k in range(nsteps):
                A = DiffeoAction.compose(A, a)
                rgb = A.get_diffeo2d_forward().get_rgb_info()
                f.data_rgb('%s_%s' % (i, k), rgb)
    
