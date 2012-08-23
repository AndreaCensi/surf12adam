from contracts import contract
from diffeoplan.library.algo.planning_algo import DiffeoPlanningAlgo
from diffeoplan.library.algo.planning_result import PlanningResult
from diffeoplan.library.analysis import DiffeoStructure
from diffeoplan.library.discdds import DiffeoSystem
from diffeoplan.library.images.uncertain_image import UncertainImage
from diffeoplan.library.analysis.cover.diffeo_cover import DiffeoCover


class InformedTree(DiffeoPlanningAlgo):
    
    def __init__(self, diffeo_structure_params,
                        diffeo_cover_params):
        DiffeoPlanningAlgo.__init__(self)
        self.diffeo_structure_params = diffeo_structure_params
        self.diffeo_cover_params = diffeo_cover_params
        
    @contract(dds=DiffeoSystem)
    def init(self, dds):
        DiffeoPlanningAlgo.init(self, dds)
        self.ds = DiffeoStructure(**self.diffeo_structure_params)
        self.cover = DiffeoCover(**self.diffeo_cover_params)
        
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): 
        return PlanningResult(success=False, plan=None, status='Not implemented')

