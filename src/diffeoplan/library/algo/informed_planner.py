from . import contract, GenericGraphPlanner
from diffeoplan.library.analysis import DiffeoStructure
from reprep import Report
import itertools
from diffeoplan.library.analysis.structure.plan_reducer import PlanReducer
from diffeo2dds.model.diffeo_system import DiffeoSystem

__all__ = ['InformedPlanner']

class InformedPlanner(GenericGraphPlanner):
    """
        This one knows that some plans are redundant
    """
    def __init__(self, diffeo_structure_params, pre_expand=[],
                 *args, **kwargs):
        super(InformedPlanner, self).__init__(*args, **kwargs)
        self.diffeo_structure_params = diffeo_structure_params
        self.pre_expand = pre_expand
        
    def __str__(self):
        return 'InformedPlanner(%s)' % (self.__strparams__())
        
    def __strparams__(self):
        # TODO: use classes
        p = GenericGraphPlanner.__strparams__(self)
        return p 

    @contract(dds=DiffeoSystem)
    def init(self, id_dds, dds):
        self.ds = DiffeoStructure(dds=dds, **self.diffeo_structure_params)
        self.original_dds = dds
        dds_expanded = diffeosystem_expand(dds, pr=self.ds.get_plan_reducer(),
                                           heuristics=self.pre_expand)
        
        # TODO: recompute structure
         
        self.info('pre_expand: %s' % str(self.pre_expand))
        self.info('Expanded to %s from %s actions' % 
                  (len(dds_expanded.actions), len(dds.actions)))
        
        super(InformedPlanner, self).init(id_dds, dds_expanded)
        
    
    def plan(self, *args, **kwargs):
        # we need to translate back the result according to the original plan
        result = super(InformedPlanner, self).plan(*args, **kwargs)
        if result.success:
            plan_extended = result.plan
            self.info('Solution found in extended space: %s' % str(plan_extended))
            plan_simple = self.get_dds().plan_with_simple_actions(plan_extended)
            self.info('With simple actions: %s' % str(plan_simple))
            result.plan = plan_simple
        
        return result
        
    @contract(report=Report)
    def init_report(self, report):
        """ Creates a report for the initialization phase. """
        super(InformedPlanner, self).init_report(report)
        self.ds.display(report.section('diffeo_structure'))
 
    def get_plan_reducer(self):
        return self.ds.get_plan_reducer()
        
@contract(dds=DiffeoSystem, pr=PlanReducer, heuristics='list(str)',
          returns=DiffeoSystem)
def diffeosystem_expand(dds, pr, heuristics):
    heuristics = list(heuristics)
    if 'bracket' in heuristics:
        heuristics.remove('bracket')
        dds = diffeosystem_expand_bracket(dds, pr)
    
    if heuristics:
        raise ValueError()

    return dds

@contract(dds=DiffeoSystem, pr=PlanReducer, returns=DiffeoSystem)
def diffeosystem_expand_bracket(dds, pr):
    # Find all pairs of inverse commands
    pairs = pr.get_inverse_pairs()
    actions = list(dds.actions)
    for pair1, pair2 in itertools.product(pairs, pairs):
        if pair1 == pair2:
            continue
        a, a_inv = pair1
        b, b_inv = pair2
        if pr.commute(a, b):
            print('%s and %s already commute' % (a, b))
            continue
        commutator = [a, b, a_inv, b_inv]
        print('Adding commutator %s' % commutator)
        commutator_action = dds.plan2action(commutator)
        
        print('a:     %s' % dds.actions[a])
        print('b    : %s' % dds.actions[b])
        print('[a,b]: %s' % commutator_action)
        actions.append(commutator_action)
        
    return DiffeoSystem(dds.label + '_bra', actions)
    
    
