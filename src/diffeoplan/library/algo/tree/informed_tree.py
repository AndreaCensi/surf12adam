from .. import DiffeoPlanningAlgo, PlanningResult
from contracts import contract
from diffeoplan.library.analysis import (DiffeoCover, DiffeoStructure,
    make_hard_choices, get_embedding_mds, plot_3d_graph, plot_2d_graph,
    make_distancetree)
from diffeoplan.library.discdds import DiffeoAction, DiffeoSystem
from diffeoplan.library.images import UncertainImage
from ggs.drawing import draw_node_graph
from reprep import MIME_PNG, Report


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
        self.cover = DiffeoCover(id_dds, self.dds_hard, self.ds,
                                 **self.diffeo_cover_params)
        self.cover.go()
        # precompute and save all distances
        self.cover.compute_distances()
        
    @contract(report=Report)
    def init_report(self, report):
        """ Creates a report for the initialization phase. """
        self.display_products(report.section('actions'), 12)
        self.ds.display(report.section('diffeo_structure'))
        if False:
            self.cover.draw_embeddings(report.section('embeddings'))
        
        self.display_distancetree(report.section('distancetree'))
        
        
    def display_distancetree(self, report, max_level=3, max_branching=5):
        plans, distances = self.cover.compute_distances()
        D = dict(distances)['pathlength']
        plan2point2 = get_embedding_mds(plans, D, ndim=2)
        plan2point3 = get_embedding_mds(plans, D, ndim=3)
        plan_level = lambda x: G.node[x]['level']
        _, G = make_distancetree(D, plans, max_branching=max_branching)
        copy_nodes_attrs(self.cover.G, G)
        
        
        frac = 0.5
        n = len(plans)
#        subnodes = [x for x in G if G.node[x]['level'] <= max_level]
        subnodes = [x for x in G if G.node[x]['order'] <= frac * n]
        Gsub = G.subgraph(subnodes)
        f = report.figure()
        with f.data_file('tree', MIME_PNG) as filename:
            draw_node_graph(filename, Gsub,
                            plan2point2.__getitem__,
                    color_func=plan_level,
                    label_func=lambda x: G.node[x]['order'],
                    cmap='Set2')
            
        with f.plot('2D') as pylab:
            plot_2d_graph(pylab, Gsub, plan2point2.__getitem__, plan_level)

        with f.plot('3D') as pylab:
            plot_3d_graph(pylab, Gsub, plan2point3.__getitem__, plan_level)
            
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): 

        while True:
        
        
            pass
        
        return PlanningResult(success=False, plan=None, status='Not implemented')



    def plan_report(self, report):
        """ Report after planning (using own data structures) """
        pass

    def display_products(self, report, nsteps):
        # XXX: make separate
        for i, a in enumerate(self.dds_hard.actions):
            f = report.figure('cmd%s' % i, cols=nsteps)
            A = a
            for k in range(nsteps):
                A = DiffeoAction.compose(A, a)
                rgb = A.get_diffeo2d_forward().get_rgb_info()
                f.data_rgb('%s_%s' % (i, k), rgb)
    
def copy_nodes_attrs(Gfrom, Gto):
    """ Copies the nodes attributes from one to the other """
    for node in Gto:
        if not node in Gfrom:
            print('warning, node %s not found in other one.' % str(node))
        for k, v in Gfrom.node[node].items():
            Gto[k] = v
            
            
    
    
