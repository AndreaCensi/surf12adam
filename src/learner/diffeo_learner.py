from . import  logger
from PIL import Image #@UnresolvedImport
from boot_agents.diffeo import (DiffeomorphismEstimator, diffeo_to_rgb_angle,
    diffeo_to_rgb_norm)
from conf_tools.utils.friendly_paths import friendly_path
from diffeoplan.library import DiffeoAction, DiffeoSystem
import numpy as np
import pickle
import yaml
from contracts import contract
import os


class DiffeoLearner:
    ''' Organizes a list of diffeomorphism estimators to learn the diffeomorphisms '''
    
    @contract(size='seq[2](int)', search_area='seq[2](int)')
    def __init__(self, size, search_area):
        logger.info('Learner Initializing...')
        self.command_list = []
        self.estimators = []
        self.estimators_inv = []    
        self.size = size
        self.search_area = search_area
        
    def new_estimator(self):
        width = self.size[0]
        height = self.size[1]
        s_w = float(self.search_area[0])
        s_h = float(self.search_area[1])
        shape = (s_h / height, s_w / width)
        return DiffeomorphismEstimator(shape, "continuous")
    
    def command_index(self, command):
        if not command in self.command_list:    
            logger.info('Adding new command to command_list: %s ' % str(command))
            self.command_list.append(command)
            self.estimators.append(self.new_estimator())
            self.estimators_inv.append(self.new_estimator())
            
        index = self.command_list.index(command)
        return index
    
        
    def update(self, Y0, U0, Y1):
        cmd_ind = self.command_index(U0)
        logger.info('Updating estimator %s' % str(cmd_ind))
        for ch in range(3):
            self.estimators[cmd_ind].update(Y0[:, :, ch], Y1[:, :, ch])
            self.estimators_inv[cmd_ind].update(Y1[:, :, ch], Y0[:, :, ch])
            
                            
    def summarize(self):
        """
            Summarizes all estimators
            Output:
                All summarized diffeomorphisms stored in self.diffeo_list
        """
        n = len(self.estimators)
        action_list = [] 
        
        for i in range(n):
            diffeo = self.estimators[i].summarize()
            diffeo_inv = self.estimators_inv[i].summarize()
            command = np.array(self.command_list[i])
            name = 'Uninterpreted Diffeomorphism' + str(i)
            action = DiffeoAction(name, diffeo, diffeo_inv, command)
            action_list.append(action)
            
        name = 'Uninterpreted Diffeomorphism System'
        self.system = DiffeoSystem(name, action_list)
                
    def diffeo_dump(self, path, name):
        '''
            Save the summarized diffeomorphisms system to a pickle file
        
            :param path: output directory
            :param name: name of this system
        '''
        
        if not os.path.exists(path):
            os.makedirs(path)
 
        basename = os.path.join(path, '%s.discdds' % name)
        
        pickle_file = basename + '.pickle'
        with open(pickle_file, 'wb') as f:
            pickle.dump(self.system, f)
                
        filename_yaml = basename + '.yaml'
        description = {
                       'id': name,
                       'desc': 'Learned diffeomorphism',
                       'code': ['diffeoplan.library.load_pickle',
                                {'file:pickle': name + '.discdds.pickle'}]
                       }
        # XXX: repeated code
        logger.info('Writing to %r ' % friendly_path(filename_yaml))
        with open(filename_yaml, 'w') as f:
            yaml.dump([description], f,
                      default_flow_style=False, explicit_start=True)
            
    # TODO: remove
    def show_diffeomorphisms(self):
        for i in range(len(self.estimators)): #estr in self.estimators:
            D = self.estimators[i].summarize()
            cmd = self.command_list[i]
            save_path = '.'
            #Image.fromarray(diffeo_to_rgb_angle(D.d)).show()
            pim = Image.fromarray(diffeo_to_rgb_angle(D.d)) 
            pim.save(save_path + 'cmd' + str(cmd).replace(' ', '') + 'angle.png')
            pim = Image.fromarray(diffeo_to_rgb_norm(D.d))
            pim.save(save_path + 'cmd' + str(cmd).replace(' ', '') + 'nomr.png')
            pim = Image.fromarray((D.variance * 255).astype(np.uint8))
            pim.save(save_path + 'cmd' + str(cmd).replace(' ', '') + 'variance.png')
            #Image.fromarray(diffeo_to_rgb_angle(D.d)).save('diffeoimages/dir'+str(delta[0])+','+str(delta[1])+'_n'+str(n)+'_image_'+image_str+'_phase.png')
        #pdb.set_trace()
        
    
