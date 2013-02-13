from diffeoplan.library.algo import DiffeoCoverExp, DiffeoActionL2iwNormalized
from diffeoplan.library.algo.memoize_strategy import dp_memoize_instance
from diffeoplan.library.analysis import DiffeoCover, DiffeoStructure
from diffeoplan.library.discdds import DiffeoAction
from diffeoplan.library.discdds.diffeo_action_distances import (
    diffeoaction_distance_L2_infow)
import itertools
import numpy as np


class DiffeoSystemBounds:
    def __init__(self, id_dds, dds, tolerance,
                 collapse_threshold,
                 min_visibility, debug_it, max_it):
        self.dds = dds
         
        self.ds = DiffeoStructure(dds, tolerance)
                
        self.cover = DiffeoCover(id_dds, self.dds, self.ds,
                                 collapse_threshold=collapse_threshold,
                                 min_visibility=min_visibility,
                                 debug_it=debug_it,
                                 max_it=max_it)
        self.cover.go()
             
    def display(self, report): #@UnusedVariable
        with report.subsection('draw_graph') as r:
            self.cover.draw_graph(r)

    def display_products(self, report, nsteps):
        for a in self.dds_hard.actions:
            f = report.figure(a.label, cols=nsteps)
            A = a
            for k in range(nsteps):
                A = DiffeoAction.compose(A, a)
                rgb = A.get_diffeo2d_forward().get_rgb_info()
                f.data_rgb('%s_%s' % (a.label, k), rgb)
    


class DiffeoSystemBounds2:
    def __init__(self, id_dds, dds, tolerance,
                 collapse_threshold,
                 min_visibility, debug_it, max_it):
        self.dds = dds
         
        self.ds = DiffeoStructure(dds, tolerance)
        
        collapse_metric = DiffeoActionL2iwNormalized(self.ds)
        
        self.cover = DiffeoCoverExp(id_dds=id_dds,
                                    dds=self.dds,
                                    plan_reducer=self.ds.get_plan_reducer(),
                                    collapse_metric=collapse_metric,
                                    #collapse_threshold=collapse_threshold,
                                    collapse_threshold=0.001,
                                    max_depth=3,
                                    debug_iterations=debug_it,
                                    max_iterations=max_it)
        self.cover.set_min_visibility(min_visibility)
        self.cover.go()
#       
        #self.make_bases(self.dds, self.ds)     
        
    def make_bases(self, dds, ds):
        n = len(dds.actions)
#        # Create all combinations of n of all elements
#        print('Creating all combinations of len %s' % n)
#        bplans = plans_of_max_length(n, n) 
#        print('Created %d ' % len(bplans)) 
#        minimal, mmap = ds.get_minimal_equiv_set(bplans)
#        print('Minimal %d ' % len(minimal))
##        print minimal 

#        self.find_non_red_plans(nactions=n, length=3, threshold=0.05)
        self.make_closure(nactions=n, length=3, threshold=0.05)
        
    
    @dp_memoize_instance
    def plan_distance_norm(self, plan1, plan2):
        dn = self.cover.plan_distance(plan1, plan2, diffeoaction_distance_L2_infow) / self.ds.scalew
        return dn
            
    def minimum_dist_to_set(self, plan, plans):
        d = [self.plan_distance_norm(plan, p) for p in plans]
        i = np.argmin(d) 
        return plans[i], min(d)
    
    def plan_inverse(self, plan):
        l = [self.ds.plan_reducer.action_get_inverse(a) for a in reversed(plan)]
        return tuple(l)
    
    def make_commutator(self, plan1, plan2):
        com = plan1 + plan2 + self.plan_inverse(plan1) + self.plan_inverse(plan2)
        com = self.ds.plan_reducer.get_canonical(com)
        return com
        
    def make_closure(self, nactions, length, threshold):
        if length == 1:
            return [(a,) for a in range(nactions)] + [()]

        prev = self.make_closure(nactions, length - 1, threshold)
        
        generated = []
        for prev1, prev2 in itertools.product(prev, prev):
            if prev1 == prev2:
                continue
            com = self.make_commutator(prev1, prev2)
            #print('[%s, %s] -> %s ' % (prev1, prev2, com))
            _, md = self.minimum_dist_to_set(com, prev + generated)
            #print('%s md %s to %s' % (com, md, closest))
            if md < threshold:
                #print('%s matches %s' % (com, closest))
                continue
            else:
                generated.append(com)
        
        print('Length %d generated %d:' % (length, len(generated)))
        for _, g in enumerate(generated):
            print(' - %s' % str(g))
            
        return generated + prev
    
    def find_non_red_plans(self, nactions, length, threshold):
        if length == 0:
            return [()]
        
        prev = self.find_non_red_plans(nactions, length - 1, threshold)
        cur = []
        cur.extend(prev)
        for p0 in prev:
            if len(p0) != length - 1:
                # only build in those from the previous level
                continue
            for action in range(nactions):
                if action in p0:
                    # no repeated actions
                    continue
                p1 = p0 + (action,)
                closest, md = self.minimum_dist_to_set(p1, cur)
                print('%s md %s to %s' % (p1, md, closest))
                if md < threshold:
                    print('%s matches %s' % (p1, closest))
                    continue
                else:
                    cur.append(p1)
            
        print('Of length %d, found: %s' % (length, len(cur)))
        return cur
        
        
        
    def display(self, report): #@UnusedVariable
        with report.subsection('draw_graph') as r:
            self.cover.draw_graph(r)

    def display_products(self, report, nsteps):
        for a in self.dds_hard.actions:
            f = report.figure(a.label, cols=nsteps)
            A = a
            for k in range(nsteps):
                A = DiffeoAction.compose(A, a)
                rgb = A.get_diffeo2d_forward().get_rgb_info()
                f.data_rgb('%s_%s' % (a.label, k), rgb)
    
