import random
from . import DiffeoPlanningAlgo, PlanningResult, contract, logger
from .. import  UncertainImage
from diffeoplan.library.graph import *
import numpy as np
import time
import copy
import pdb

class RandomizedExpansion(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 

    """
    
    
    def __init__(self, max_ittr, tresh, region, directions=1):
        '''
        :param nsteps: Number of steps in the random guess.
        '''
#        dds = self.get_dds()
#        self.ncommands = len(dds.actions)
        self.max_ittr = max_ittr
        self.tresh = tresh
        self.directions = directions
        self.region = region
#        self.max_cpu_time = max_cpu_time
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): #@UnusedVariable
        
        print('Engering graphsearch plan()')
        dds = self.get_dds()
        
        start_node = Node(y0,[])
        start_tree = Tree(start_node)
        
        goal_node = Node(y1,[])
        goal_tree = Tree(goal_node)
        
        connector = TreeConnector(start_tree, goal_tree, self.tresh)
        
        for i in range(self.max_ittr):
            start_tree.add_node(self.get_new_node(start_tree))
            if self.directions == 2:
                goal_tree.add_node(self.get_new_node(goal_tree))
            
            nplans = connector.connect_update()
            if nplans > 0:
                plan = connector.get_connection()
                return PlanningResult(True, plan, 'Randomized Expansive Planner')
        return PlanningResult(False, None, 'Randomized Expansive Planner')
        
    def expand_alldir(self, plan):
        n = len(self.get_dds().actions)
        expanded = []
#        pdb.set_trace()
        for i in range(n):
            plani = copy.deepcopy(plan)
            plani.append(i)
            expanded.append(plani)
        return expanded
    
    def expand_level(self, plans):
        extplans = []
        for i in range(len(plans)):
            extplans.extend(self.expand_alldir(plans[i]))
        return extplans

    def get_next_node(self,tree):
        dds = self.get_dds()
        n = len(dds.actions)
        tresh = self.region
        pmf = 1/np.sum((tree.distances<tresh).astype(np.float),0)
        exp_index = random_pmf(pmf)
        return exp_index
    
    def get_next_command(self):
        dds = self.get_dds()
        n = len(dds.actions)
        return random.randint(0,n-1)
    
    def get_new_node(self,tree):
        exp_ind = self.get_next_node(tree)
        exp_cmd = self.get_next_command()
        
        parent = tree.nodes[exp_ind]
        new_node = copy.deepcopy(parent)
        new_node.path.append(exp_cmd)
#        new_node.parent = parent
#        pdb.set_trace()
        y_new = self.get_dds().actions[exp_cmd].predict(tree.nodes[exp_ind].y)
        new_node.y = y_new
        return new_node
                
def random_pmf(pmf):
    """
        returns a random integer from the probability mass function <pmf> given as a vector.    
    """
    total_prob = sum(pmf)
    pmf = np.array(pmf)/total_prob
    prob = pmf[0]
#    pdb.set_trace()
    rval = random.random()
    
    for i in range(len(pmf)-1):
        if rval < prob:
            return i
        prob = prob + pmf[i+1]
    return len(pmf)-1
    
    
    
    
    
    