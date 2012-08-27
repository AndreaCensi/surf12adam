from . import contract, GenericGraphPlanner
from diffeoplan.library.analysis import DiffeoStructure, make_hard_choices
from diffeoplan.library.discdds import DiffeoAction, DiffeoSystem
from reprep import Report


class InformedPlanner(GenericGraphPlanner):
    """
        This one knows that some plans are redundant
    """
    def __init__(self, diffeo_structure_params,
                        hard_choice_params, *args, **kwargs):
        GenericGraphPlanner.__init__(self, *args, **kwargs)
        self.diffeo_structure_params = diffeo_structure_params
        self.hard_choice_params = hard_choice_params
    
    def __str__(self):
        return 'InformedPlanner(%s)' % (self.__strparams__())
        
    def __strparams__(self):
        # TODO: use classes
        p = GenericGraphPlanner.__strparams__(self)
        return p # + "%s;%s" % (self.diffeo_structure_params, self.hard_choice_params)
    
    @contract(dds=DiffeoSystem)
    def init(self, id_dds, dds):
        GenericGraphPlanner.init(self, id_dds, dds)
        self.ds = DiffeoStructure(dds=dds, **self.diffeo_structure_params)
        self.dds_hard = make_hard_choices(dds, **self.hard_choice_params)
        
        
    @contract(report=Report)
    def init_report(self, report):
        """ Creates a report for the initialization phase. """
        self.display_products(report.section('actions'), 12)
        self.ds.display(report.section('diffeo_structure'))

    def display_products(self, report, nsteps):
        # XXX: make separate
        for i, a in enumerate(self.dds_hard.actions):
            f = report.figure('cmd%s' % i, cols=nsteps)
            A = a
            for k in range(nsteps):
                A = DiffeoAction.compose(A, a)
                rgb = A.get_diffeo2d_forward().get_rgb_info()
                f.data_rgb('%s_%s' % (i, k), rgb)
 
    def get_plan_reducer(self):
        return self.ds.get_plan_reducer()
    
        
