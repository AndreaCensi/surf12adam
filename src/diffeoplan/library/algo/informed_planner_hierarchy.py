from . import InformedPlanner
from contracts import contract
from diffeoplan.library.analysis import (DiffeoCover, get_embedding_mds,
    plot_3d_graph, plot_2d_graph, make_distancetree)
from diffeoplan.library.discdds import DiffeoSystem
from ggs.drawing import draw_node_graph
from ggs.utils import copy_nodes_attrs
from reprep import MIME_PNG, Report

__all__ = ['InformedPlannerHierarchy']

class InformedPlannerHierarchy(InformedPlanner):
    """
        This one knows that some plans are redundant
    """
    def __init__(self, diffeo_cover_params, *args, **kwargs):
        super(InformedPlannerHierarchy, self).__init__(*args, **kwargs)
        self.diffeo_cover_params = diffeo_cover_params
        
    @contract(dds=DiffeoSystem)
    def init(self, id_dds, dds):
        super(InformedPlannerHierarchy, self).init(id_dds, dds)
        self.cover = DiffeoCover(id_dds, self.dds_hard, self.ds,
                                 **self.diffeo_cover_params)
        self.cover.go()
        # precompute and save all distances
        self.cover.compute_distances()
        
    @contract(report=Report)
    def init_report(self, report):
        """ Creates a report for the initialization phase. """
        super(InformedPlannerHierarchy, self).init_report(report)
        if True:
            self.cover.draw_embeddings(report.section('embeddings'))
        
        self.display_distancetree(report.section('distancetree'))
       
    def display_distancetree(self, report, max_level=100, frac=0.5,
                             max_branching=5):
        plans, distances = self.cover.compute_distances()
        D = dict(distances)['pathlength']
        plan2point2 = get_embedding_mds(plans, D, ndim=2)
        plan2point3 = get_embedding_mds(plans, D, ndim=3)
        plan_level = lambda x: G.node[x]['level']
        _, G = make_distancetree(D, plans, max_branching=max_branching)
        copy_nodes_attrs(self.cover.G, G)

        n = len(plans)
        subnodes = [x for x in G if G.node[x]['level'] <= max_level
                                and G.node[x]['order'] <= frac * n]
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
   
    
    
    
