from contracts import contract
from diffeoplan.library.discdds.diffeo_action import DiffeoAction
import itertools
from boot_agents.misc_utils.tensors_display import iterate_indices
from html5lib.utils import deque

 
    
def compute_geometry(dds, report):
    
    report_distances(dds, report.section('distances'))
    

import numpy as np

def construct_matrix(shape, function):
    if len(shape) != 2:
        raise ValueError()
    D = np.zeros(shape) 
    for indices in iterate_indices(shape):
        result = function(*indices)
        if not isinstance(result, float):
            raise ValueError('%s(%s) = %s' % 
                             (function.__name__, indices, result))
        D[indices] = result
    print D
    return D
    
@contract(returns='array[NxN](>=0)')
def get_actions_distance_L2_mixed_matrix(dds):
    def entries(i, j):
        a1 = dds.actions[i]
        a2 = dds.actions[j]
        return DiffeoAction.distance_L2_mixed(a1, a2)
    K = len(dds.actions)
    return construct_matrix((K, K), entries)
    

@contract(returns='array[NxN](>=0)')
def get_actions_anti_distance_L2_mixed_matrix(dds):
    def entries(i, j):
        a1 = dds.actions[i]
        a2 = dds.actions[j]
        return DiffeoAction.anti_distance_L2_mixed(a1, a2)
    K = len(dds.actions)
    return construct_matrix((K, K), entries)



@contract(returns='array[NxN](>=0)')
def get_actions_comm_distance_L2_mixed_matrix(dds):
    def entries(i, j):
        a1 = dds.actions[i]
        a2 = dds.actions[j]
        return DiffeoAction.comm_distance_L2_mixed(a1, a2)
    K = len(dds.actions)
    return construct_matrix((K, K), entries)
    
class DiffeoStructure():
    """ Estimates the intrinsic structure of a given DiffeoSystem. """
    def __init__(self, dds):
        self.dds = dds
        self.compute_distances()
        
        self.labels = [a.label for a in self.dds.actions]
        
    def compute_distances(self):
        self.D = get_actions_distance_L2_mixed_matrix(self.dds)
        self.aD = get_actions_anti_distance_L2_mixed_matrix(self.dds)
        self.cD = get_actions_comm_distance_L2_mixed_matrix(self.dds)
        
        # This is the minimum size of a command
        self.scale = np.min(self.aD.diagonal()) / 2.0
        # This is our tolerance for comparisons
        self.eps = self.scale / 5.0
        
        self.same = self.D < self.eps
        self.opposite = self.aD < self.eps
        self.swappable = self.cD < self.eps
        
    def display(self, report):
        report.data('scale', self.scale)
        report.data('eps', self.eps)
        self.display_distances(report)
        self.display_structure(report)
        self.show_reduction_steps(report)
        self.show_reduction(report)
        
    
    def display_structure(self, report):
        f = report.figure(caption='Inferred structure')
        labels = self.labels
        f.table('same', self.same, cols=labels, rows=labels)
        f.table('opposite', self.opposite, cols=labels, rows=labels)
        f.table('swappable', self.swappable, cols=labels, rows=labels)
        
    
    def display_distances(self, report):
        f = report.figure()
        labels = self.labels
        report.table('distances_table', self.D, cols=labels, rows=labels,
                     caption='Distance between actions (L2 mixed)')
        report.table('anti_distances_table', self.aD, cols=labels, rows=labels,
                     caption='Anti-distance between actions (L2 mixed)')
        report.table('comm_distances_table', self.cD, cols=labels, rows=labels,
                     caption='Commutation error (L2 mixed)')
        
        caption = 'Distances between actions (black=0, white=max)'
        report.data('distances', self.D).display('scale', caption=caption).add_to(f)
        caption = 'Anti-distances between actions (black=0, white=max)'
        report.data('anti_distances', self.aD).display('scale', caption=caption).add_to(f)
        caption = 'Comm-distances between actions (black=0, white=max)'
        report.data('comm_distances', self.cD).display('scale', caption=caption).add_to(f)
            
    def plan2desc(self, plan):
        return ",".join(self.labels[i] for i in plan)
    
    def plan2point(self, plan):
        """ Assuming that we can add the original_cmd... (for debug only) """
        if not plan: # empty
            n = self.dds.actions[0].original_cmd.size
            return np.zeros(n)
        return np.sum(self.dds.actions[i].original_cmd for i in plan)
    
    
    def show_reduction_steps(self, report):
        nsteps, nplans, ncplans = self.compute_reduction_steps(max_nsteps=7)
        report.data('nsteps', nsteps)
        report.data('nplans', nplans)
        report.data('ncplans', ncplans)
        f = report.figure()
        
        caption = """
            Efficient plan generations. 
            Let L be the length, N the number of commands,
            and P the number of plans
            
            The naive grows exponentially.
            
                P_naive ~= N ^ L
                
            The reduced grows according to the ambient space.
            If the topology is \reals^K, then it grows like
            
                P_smart ~= L ^ K
        
            this is the *volume* of the area.
            
            Usually the number of commands is N = 2*K
            
            So we have:
              
                N^L     vs   L^(N/2)
            
            The logarithm is
            
                log(P_naive) = log(N) * L     
                
                log(P_smart) = N * log(L) 
            
            So fixing N and plotting as a function of L and in logarithmic
            coordinates, we have that the first plot should be a line,
            and the second one a logarithm.
            
                
        """
        report.text('explanation', caption)
        with f.plot('reduction') as pylab:
            pylab.semilogy(nsteps, nplans, label='naive')
            pylab.semilogy(nsteps, ncplans, label='reduced')
            pylab.legend()
            pylab.xlabel('plan length (L)')
            pylab.ylabel('number of plans (P)')
        
    @contract(returns='tuple(list[M], list[M], list[M])')
    def compute_reduction_steps(self, max_nsteps=5):
        K = len(self.dds.actions)
        nsteps = []
        nplans = []
        ncplans = []
        for n in range(1, max_nsteps + 1):
            nsteps.append(n)
            plans = plans_of_max_length(ncmd=K, maxsteps=n)
            cplans, _ = self.get_minimal_equiv_set(plans)
            print('%3d steps: from %5d to %5d' % 
                  (n, len(plans), len(cplans)))
            nplans.append(len(plans))
            ncplans.append(len(cplans))
        return nsteps, nplans, ncplans
    
    def show_reduction(self, report, nsteps=4):
        K = len(self.dds.actions)
        # all plans of length 4
        plans = plans_of_max_length(ncmd=K, maxsteps=nsteps)
        cplans, plan2cplans = self.get_minimal_equiv_set(plans)
         
        s = "\n".join('%s -> %s  (sum cmd: %s)' % 
                      (self.plan2desc(a), self.plan2desc(b),
                       self.plan2point(a)) #, self.plan2point(b))  
                      for a, b in plan2cplans.items())
        report.text('plans', s)
        s = "\n".join('%s    (sum: %s)' % (self.plan2desc(x), self.plan2point(x))
                      for x in cplans)
        report.text('cplans', s)
        
        
        f = report.figure()
        caption = 'Points reached by the %d plans of %d steps' % (len(plans), nsteps)
        with f.plot('plans_reached', caption=caption) as pylab:
            for plan in plans:
                p = self.plan2point(plan)
                pylab.plot(p[0], p[1], 'rx')
            pylab.axis('equal')
            
        caption = 'Points reached by %d canonical plans' % len(cplans)
        with f.plot('cplans_reached', caption=caption) as pylab:
            for plan in cplans:
                p = self.plan2point(plan)
                pylab.plot(p[0], p[1], 'gx')
            pylab.axis('equal')
        
    @contract(plans='seq(seq(int))')
    def get_minimal_equiv_set(self, plans, ignore_zero_len=True):
        """ 
            Returns a tuple: first the canonical plans,
            and then the map plan->cplan.
        """ 
        cplans = set()
        plan2cplan = {}
        for plan in plans:
            plan = tuple(plan)
            cplan = tuple(self.get_canonical(plan))
            plan2cplan[tuple(plan)] = cplan
            if len(cplan) == 0 and ignore_zero_len:
                continue
            else:
                cplans.add(cplan)            
        # sort the plans by length
        # TODO: lexicographically
        cplans = list(cplans)
        cplans.sort(key=lambda x: len(x))
        return cplans, plan2cplan
        
            
    def actions_commute(self, i, j):
        return self.swappable[i, j]
    
    def actions_are_opposite(self, i, j):
        return self.opposite[i, j]
    
        
    def get_canonical(self, plan):
        def log(s):
            # print(s)
            pass
   
        current = list()
        remaining = deque(plan)
        while remaining:
            log('- %s %s' % (current, remaining))
            # get a new action
            new_action = remaining.popleft()
            # if possible to do something...
            if current:
                prev_action = current[-1]
                # if they cancel each other
                if self.actions_are_opposite(prev_action, new_action):
                    log('canceling %s %s' % (prev_action, new_action))
                    # remove the current one
                    current.pop()
                    continue
                # if they commute, put them in order
                if self.actions_commute(prev_action, new_action):
                    log('commuting actions %s %s' % (prev_action, new_action))
                    current_order = [prev_action, new_action]
                    canonical_order = sorted([prev_action, new_action])
                    if current_order != canonical_order:
                        log('reversing order')
                        # they are not in the right order
                        # now, of course, we put them both in the stack
                        # because there might be other changes to do
                        current.pop()
                        remaining.appendleft(prev_action)
                        remaining.appendleft(new_action)
                    else:
                        # just append
                        current.append(new_action) 
                    
            else:
                # put it on current
                current.append(new_action)
                
        return current


@contract(ncmd='M,>=1', maxsteps='K',
          returns='seq(seq[<=K](int,>=0,<M))')
def plans_of_max_length(ncmd, maxsteps):
    """ Returns all plans of the given maximum length. """
    plans = []
    for M in range(maxsteps + 1):
        plans_M = itertools.product(range(ncmd), repeat=M) 
        plans.extend(plans_M)
    return plans

def report_distances(dds, report):
    """ Computes and displays the distance between each pair of actions """
    ds = DiffeoStructure(dds)
    ds.display(report)
    
    
    
    
    
    
    
    
