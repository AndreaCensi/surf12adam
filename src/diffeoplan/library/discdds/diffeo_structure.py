from . import DiffeoAction, np, contract, plans_of_max_length
from boot_agents.diffeo import diffeo_to_rgb_angle, diffeo_to_rgb_norm, scalaruncertainty2rgb
from boot_agents.misc_utils import iterate_indices
from collections import deque
from diffeoplan.utils import memoize #@UnresolvedImport
import pdb


class DiffeoStructure():
    """ 
        Estimates the intrinsic structure of a given DiffeoSystem
        and is able to tell you if a plan has a shorter equivalent.    
    """
    def __init__(self, dds, tolerance=0.2, use_weighted=True):
        '''
        
        :param dds: The DiffeoSystem
        :param tolerance:
        :param use_weighted:
        '''
        self.dds = dds

        
        # This is our tolerance for comparisons
        self.tolerance = tolerance
        # Names of actions
        self.labels = [a.label for a in self.dds.actions]
        
        if 'Uninterpreted' in self.labels[0]:
            def shortname(x):
                if x > 0:
                    return '+'
                elif x < 0: 
                    return '-'
                else:
                    return '0'
            self.labels = ["".join(map(shortname, x.original_cmd)) 
                           for x in self.dds.actions]
        
        self.compute_distances()
        self.use_weighted = use_weighted 
        
    def compute_distances(self):
        self.D = self.dds.actions_distance_L2()
        self.aD = self.dds.actions_anti_distance_L2()
        self.cD = self.dds.actions_comm_distance_L2()
        # This is the minimum size of a command
        self.scale = np.min(self.aD.diagonal()) / 2.0

        # we use it to normalize the distances
        self.D_n = self.D / self.scale 
        self.aD_n = self.aD / self.scale
        self.cD_n = self.cD / self.scale

        self.same = self.D_n < self.tolerance
        self.opposite = self.aD_n < self.tolerance
        self.swappable = self.cD_n < self.tolerance

        # weighted versions
        self.Dw = self.dds.actions_distance_L2_infow()
        self.aDw = self.dds.actions_anti_distance_L2_infow()
        self.cDw = self.dds.actions_comm_distance_L2_infow()
        
        self.scalew = np.min(self.aDw.diagonal()) / 2.0
        
        self.Dw_n = self.Dw / self.scalew 
        self.aDw_n = self.aDw / self.scalew
        self.cDw_n = self.cDw / self.scalew
        
        self.samew = self.Dw_n < self.tolerance
        self.oppositew = self.aDw_n < self.tolerance
        self.swappablew = self.cDw_n < self.tolerance
       

    # TODO
    def actions_commute(self, i, j):
        if self.use_weighted:
            return self.swappable[i, j]
        else:
            return self.swappablew[i, j]
    
    def actions_are_opposite(self, i, j):
        if self.use_weighted:
            return self.opposite[i, j]
        else:
            return self.oppositew[i, j]
        
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
        
            
    @memoize
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
                elif self.actions_commute(prev_action, new_action):
                    # if they commute, put them in order
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
                    current.append(new_action)
            else:
                # put it on current
                current.append(new_action)
                
        return current
    
    # Below, just visualization
    
    
    def display(self, report):
        report.data('scale', self.scale)
        report.data('tolerance', self.tolerance)

        self.display_distances(report)
        self.display_structure(report)
        self.show_reduction_steps(report, max_nsteps=4)
        self.show_reduction(report)
        #self.show_compositions(report)
    
    def display_structure(self, report):
        f = report.figure(caption='Inferred structure')
        labels = self.labels
#        f.table('same', self.same, cols=labels, rows=labels)
#        f.table('opposite', self.opposite, cols=labels, rows=labels)
#        f.table('swappable', self.swappable, cols=labels, rows=labels)
        
    
    def display_distances(self, report):
        f = report.figure('unweighted', cols=3)

        def table(n, x, caption):
            data = report.data(n, x)
            report.table(n + '_table', x, rows=self.labels, cols=self.labels)
            data.display('scale', caption=caption).add_to(f)
            
        def tableb(n, x, caption):
            data = report.data(n, x.astype('float'))
            report.table(n + '_table', x, rows=self.labels, cols=self.labels)
            data.display('scale', caption=caption + ' (green: true)', min_value=0,
                         max_value=1, min_color=[1, 0, 0], max_color=[0, 1, 0]).add_to(f)
            
        table('D', self.D, 'Distance between actions (L2 mixed)')
        table('aD', self.aD, 'Anti-distance between actions (L2 mixed)')
        table('cD', self.cD, 'Commutation error (L2 mixed)')
        table('D_n', self.D_n, 'Normalized distance between actions (L2 mixed)')
        table('aD_n', self.aD_n, 'Normalized Anti-distance between actions (L2 mixed)')
        table('cD_n', self.cD_n, 'Normalized Commutation error (L2 mixed)')
        tableb('same', self.same, 'same')
        tableb('opposite', self.opposite, 'opposite')
        tableb('commute', self.swappable, 'commute')
        
        f = report.figure('weighted', cols=3)
        table('Dw', self.Dw, 'Distance between actions (L2 mixed)')
        table('aDw', self.aDw, 'Anti-distance between actions (L2 mixed)')
        table('cDw', self.cDw, 'Commutation error (L2 mixed)')
        table('Dw_n', self.Dw_n, 'Normalized distance between actions (L2 mixed)')
        table('aDw_n', self.aDw_n, 'Normalized Anti-distance between actions (L2 mixed)')
        table('cDw_n', self.cDw_n, 'Normalized Commutation error (L2 mixed)')
        tableb('samew', self.samew, 'samew')
        tableb('oppositew', self.oppositew, 'oppositew')
        tableb('commutew', self.swappablew, 'commutew')
         
    
    def show_reduction_steps(self, report, max_nsteps):
        nsteps, nplans, ncplans = self.compute_reduction_steps(max_nsteps)
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
            If the topology is that of Reals^K, then it grows like
            
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
            pylab.semilogy(nsteps, nplans, 's-', label='naive')
            pylab.semilogy(nsteps, ncplans, 's-', label='reduced')
            pylab.legend()
            pylab.xlabel('plan length (L)')
            pylab.ylabel('number of plans (P)')
    
    def show_compositions(self, report):
        #f_ba = report.figure()
        n = len(self.dds.actions)
        f_ab_norm = report.figure(cols=n, caption='Norm of AB')
        f_ab_phase = report.figure(cols=n, caption='Phase of AB')
        f_ab_info = report.figure(cols=n, caption='Certainty of AB')
        for a, b in iterate_indices((n, n)):
            A = self.dds.actions[a]
            B = self.dds.actions[b]
            AB = DiffeoAction.compose(A, B)
            norm_rgb = diffeo_to_rgb_norm(AB.diffeo.d)
            phase_rgb = diffeo_to_rgb_angle(AB.diffeo.d)
            info_rgb = scalaruncertainty2rgb(AB.diffeo.variance)

            label = '%s-%s' % (A.label, B.label)
            caption = '%s, %s' % (A.label, B.label)
            f_ab_norm.data_rgb('%s_norm' % label, norm_rgb, caption=caption)
            f_ab_phase.data_rgb('%s_phase' % label, phase_rgb, caption=caption)
            f_ab_info.data_rgb('%s_info' % label, info_rgb, caption=caption)
            

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
        
        
        def plot_plan_points(pylab, plans):
            # points by rows
            points = np.array(map(self.plan2point, plans))
            print points.shape
            ndim = points.shape[1]
            if ndim == 1:
                pylab.plot(0 * points, points, 'rx')
            if ndim == 2:
                x = points[:, 0]
                y = points[:, 1]
                pylab.plot(x, y, 'rx')
            if ndim == 3:
                pass

        f = report.figure()
        caption = 'Points reached by the %d plans of %d steps' % (len(plans), nsteps)
        with f.plot('plans_reached', caption=caption) as pylab:
            plot_plan_points(pylab, plans)
            pylab.axis('equal')
            
        caption = 'Points reached by %d canonical plans' % len(cplans)
        with f.plot('cplans_reached', caption=caption) as pylab:
            plot_plan_points(pylab, cplans) 
            pylab.axis('equal')
        
    def plan2desc(self, plan):
        return ",".join(self.labels[i] for i in plan)
    
    def plan2point(self, plan):
        """ Assuming that we can add the original_cmd... (for debug only) """
        if not plan: # empty
            n = self.dds.actions[0].original_cmd.size
            return np.zeros(n)
        return np.sum(self.dds.actions[i].original_cmd for i in plan)
    
