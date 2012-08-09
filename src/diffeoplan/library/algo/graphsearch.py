from . import DiffeoPlanningAlgo, PlanningResult, contract, logger
from .. import  UncertainImage
from diffeoplan.library.graph import *
import numpy as np
import time
import copy
import pdb


class GraphSearch(DiffeoPlanningAlgo):
    """ 
        This is an algorithm that returns the best plan
        after trying all possible plans of exact length <nsteps> 

    """
    
    @contract(nsteps='int,>=1')
    def __init__(self, nsteps, tresh, directions=1):
        '''
        :param nsteps: Number of steps in the random guess.
        '''
        self.tresh = tresh
        self.nsteps = nsteps
        self.directions = directions
        
    @contract(y0=UncertainImage, y1=UncertainImage, returns=PlanningResult)
    def plan(self, y0, y1): #@UnusedVariable
        print('Engering graphsearch plan()')
        dds = self.get_dds()
        
        start_node = Node(y0,[])
        start_tree = Tree(start_node)
        
        goal_node = Node(y1,[])
        goal_tree = Tree(goal_node)
        
        connector = TreeConnector(start_tree, goal_tree, self.tresh)
        
        while True:
            new_start_node = self.get_new_node(start_tree)
            if len(new_start_node.path)<=self.nsteps:
                start_tree.add_node(new_start_node)
            else:
                break
            if self.directions == 2:
                new_goal_node = self.get_new_node(goal_tree)
                if len(new_goal_node.path)<=self.nsteps:
                    goal_tree.add_node(new_goal_node)
            
            nplans = connector.connect_update()
            if nplans > 0:
                plan = connector.get_connection()
                return PlanningResult(True, plan, 'Randomized Expansive Planner')
        return PlanningResult(True, [0], 'Randomized Expansive Planner')
        

    
    def get_new_node(self, tree):
        dds = self.get_dds()
        last_path = copy.deepcopy(np.array(tree.nodes[-1].path))
        ncommand = len(dds.actions)
        if np.sum(last_path==ncommand-1)==len(last_path):
            new_path = np.zeros(len(last_path)+1).astype(np.int)
#            new_path[:len(last_path)] = last_path
        else:
            new_path = np.zeros(len(last_path)).astype(np.int)
            add_next = 1
            for i in range(1,len(last_path)+1):
                new_path[-i] = (last_path[-i] + add_next)%ncommand
                add_next = (last_path[-i] + add_next)/ncommand
            print(new_path)
#        pdb.set_trace()
        new_node = Node(dds.predict(tree.nodes[0].y, new_path), new_path)
        return new_node
                
    
    def images_match(self, y0, y1):
        return True