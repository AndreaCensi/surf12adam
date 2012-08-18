from diffeoplan.library.discdds.diffeo_structure import DiffeoStructure
from diffeoplan.library.discdds.diffeo_action import DiffeoAction
import copy
from contracts import contract
import networkx as nx
import matplotlib.pyplot as plt
import os
import functools

class GenericGraphSearch:
    def __init__(self, choose_open_nodes, available_actions, next_node, node_compare):
        self.choose_open_nodes = choose_open_nodes
        self.node_compare = node_compare
        self.available_actions = available_actions
        self.next_node = next_node
        
        self.G = nx.Graph()
        self.iterations = 0
         
    def go(self, first):
        self.G.add_node(first)
        self.open_nodes = [first]
        self.closed = set()
        
        while self.open_nodes:
            node = self.choose_open_nodes(self.open_nodes)
            self.log_chosen(node)
            assert node in self.open_nodes
            self.open_nodes.remove(node)
            self.closed.add(node)
            self.log_closed(node)
            for action in self.available_actions(node):
                child = self.next_node(node, action)
                self.log_child_generated(node, child)
                if (not contains(self.open_nodes, child, self.node_compare) and
                    not contains(self.closed, child, self.node_compare)):
                    self.open_nodes.append(child)
                    self.G.add_node(child)
                    self.G.add_edge(node, child, action=action)
                    self.log_child_open(node, child)
                else:
                    self.log_child_discarded(node, child)
            self.iterations += 1
            
    def log(self, s):
        print(s)
        
    def log_chosen(self, node):
        pass
            
    
    def draw_graph(self):
        
        
        def label_plan_length(n):
            return '%d' % len(n)
        
        open_nodes = set(self.open_nodes)
        
        def label_child_open(n):
            # number of children in the open list
            children = self.G[n]
            return len(set(children) & open_nodes)
    
        
        point_func = lambda n: 5 * self.ds.plan2point(n)
        
        self.save_graph(pos_func=point_func,
                        base='planlength',
                        label_func=label_plan_length,
                        color_func=self.color_open_closed)
    
        self.save_graph(pos_func=point_func,
                        base='nchildopen',
                        label_func=label_child_open,
                        color_func=self.color_open_closed)
    
        self.save_graph(pos_func=point_func,
                        base='info',
                        label_func=lambda x: '%.2f' % self.visibility(x),
                        color_func=self.visibility)
    
    
    def color_open_closed(self, n):
        if n in self.closed:
            return 0.6
        else:
            return 0.8
            
    def save_graph(self, base, pos_func,
                   color_func=lambda _: 0.5,
                   label_func=lambda x: str(x)):
        pos = dict((n, pos_func(n)) for n in self.G.nodes())
        
        all_positions = [tuple(pos_func(n)) for n in self.G] 
        if len(all_positions) != len(set(all_positions)):
            print('Warning, overlapping nodes') 
            pass
     
        node_color = map(color_func, self.G.nodes()) 
        labels = dict((n, label_func(n)) for n in self.G.nodes())
        
        out = 'out/graphs/%s/' % self.id_dds
        
        fig = plt.figure() 
        nx.draw_networkx(self.G, with_labels=True, pos=pos, labels=labels,
                         node_color=node_color)
        filenames = [os.path.join(out, '%d-%s.png' % (self.iterations, base)),
                         os.path.join(out, 'last-%s.png' % base)]
        for f in filenames:
            print('writing on %r' % f)
            dirname = os.path.dirname(f)
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            plt.savefig(f)
        plt.close(fig)
        

    def log_closed(self, node):
        pass

    def log_child_open(self, node, child):
        pass
    
    def log_child_generated(self, node, child):
        pass

    def log_child_discarded(self, node, child):
        pass
    
    
        
def deep_first(open_nodes):
    return open_nodes[-1]

def breadth_first(open_nodes):
    return open_nodes[0]

def contains(iterable, needle, comparison):
    if needle in iterable:
        return True
    for x in iterable:
        if comparison(x, needle):
            return True
    return False
 
 
def memoize(obj):
    cache = obj.cache = {}

    @functools.wraps(obj)
    def memoizer(*args, **kwargs):
        if args not in cache:
            cache[args] = obj(*args, **kwargs)
        return cache[args]
    return memoizer

import numpy as np

class DiffeoCover(GenericGraphSearch):
    
    def __init__(self, id_dds, dds, ds, info_threshold):
        self.info_threshold = info_threshold
        self.dds = dds
        self.ds = ds
        self.id_dds = id_dds
        GenericGraphSearch.__init__(self,
                                    choose_open_nodes=breadth_first,
                                    available_actions=self.available_actions,
                                    next_node=self.next_node,
                                    node_compare=self.node_compare)
    
    @memoize
    def compute_action(self, plan):
        print('computing %s' % str(plan))
        last = self.dds.actions[plan[-1]]
        if len(plan) == 1:
            return last
        else:
            rest = self.compute_action(plan[:-1])
            return DiffeoAction.compose(last, rest)
         
    @memoize
    def compute_diffeomorphism(self, plan):
        action = self.compute_action(plan)
        return action.get_diffeo2d_forward()
    
    @memoize
    def visibility(self, plan):
        if len(plan) == 0:
            return 1
        diffeo = self.compute_diffeomorphism(plan)
        return np.mean(diffeo.variance)
    
    @contract(plan1='seq(int)', plan2='seq(int)')
    def node_compare(self, plan1, plan2):
        plan1 = tuple(plan1)
        plan2 = tuple(plan2)
        if plan1 == plan2:
            return True
        cplan1 = self.ds.get_canonical(plan1)
        cplan2 = self.ds.get_canonical(plan2)
        return cplan1 == cplan2
    
    def next_node(self, node, action):
        child = node + (action,)
        return tuple(self.ds.get_canonical(child))
    
    def available_actions(self, node): #@UnusedVariable
        if self.visibility(node) < self.info_threshold:
            return []
        nactions = len(self.dds.actions)
        return range(nactions)
     
    def go(self):
        first = ()
        return GenericGraphSearch.go(self, first)
        
    def log_chosen(self, node):
        print('It: %d  closed: %5d  Open: %5d (%5d) Chosen %r' % 
              (self.iterations, len(self.closed), len(self.open_nodes),
               len(set(self.open_nodes)), node))
        if self.iterations % 20 == 0:
            self.draw_graph()

class DiffeoSystemBounds:
    def __init__(self, id_dds, dds, info_threshold=0.01):
        self.dds = dds
        self.dds_hard = make_hard_choices(dds, info_threshold)
        
        self.ds = DiffeoStructure(dds)
        
        self.cover = DiffeoCover(id_dds, self.dds_hard, self.ds,
                                 info_threshold=info_threshold)
        self.cover.go()
        
    def display(self, report):
        #self.ds.display(report.section())

        self.display_products(report, nsteps=10)

    def display_products(self, report, nsteps):
        for i, a in enumerate(self.dds_hard.actions):
            f = report.figure(a.label, cols=nsteps)
            A = a
            for k in range(nsteps):
                A = DiffeoAction.compose(A, a)
                rgb = A.get_diffeo2d_forward().get_rgb_info()
                f.data_rgb('%s^%s' % (a.label, k), rgb)
                
                
    

def make_hard_choices(dds, info_threshold):
    dds = copy.deepcopy(dds)
    
    def make_hard(dd):
        dd.variance = (dd.variance > info_threshold).astype('float')
    for a in dds.actions:
        df = a.get_diffeo2d_forward()
        make_hard(df)
        db = a.get_diffeo2d_backward()
        make_hard(db)
    return dds
