from . import contract, GenericGraphPlanner
from diffeoplan.library.analysis import DiffeoStructure
from diffeoplan.library.discdds import DiffeoSystem
from reprep import Report

__all__ = ['InformedPlanner']

class InformedPlanner(GenericGraphPlanner):
    """
        This one knows that some plans are redundant
    """
    def __init__(self, diffeo_structure_params, *args, **kwargs):
        super(InformedPlanner, self).__init__(*args, **kwargs)
        self.diffeo_structure_params = diffeo_structure_params
    
    def __str__(self):
        return 'InformedPlanner(%s)' % (self.__strparams__())
        
    def __strparams__(self):
        # TODO: use classes
        p = GenericGraphPlanner.__strparams__(self)
        return p # + "%s;%s" % (self.diffeo_structure_params, self.hard_choice_params)
    
    @contract(dds=DiffeoSystem)
    def init(self, id_dds, dds):
        super(InformedPlanner, self).init(id_dds, dds)
        self.ds = DiffeoStructure(dds=dds, **self.diffeo_structure_params)
        
    @contract(report=Report)
    def init_report(self, report):
        """ Creates a report for the initialization phase. """
        super(InformedPlanner, self).init_report(report)
        self.ds.display(report.section('diffeo_structure'))
 
    def get_plan_reducer(self):
        return self.ds.get_plan_reducer()
        
