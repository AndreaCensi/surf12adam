from . import logger, np
from .diffeo_analysis import DiffeoAnalysis
from PIL import Image #@UnresolvedImport
from boot_agents.diffeo import (DiffeomorphismEstimator, diffeo_to_rgb_angle,
    diffeo_to_rgb_norm)
from boot_agents.diffeo.learning import DiffeomorphismEstimatorFaster
from diffeoplan.library import DiffeoAction, DiffeoSystem
from diffeoplan.library.discdds.writing import ds_dump


class DiffeoLearner:
    ''' Organizes a list of diffeomorphism estimators to learn the diffeomorphisms '''
    
    def __init__(self, use_fast, diffeo_estimator_params):
        '''
        
        :param use_fast: Use DiffeomorphismEstimatorFaster.
        '''
        self.command_list = []
        self.estimators = []
        self.estimators_inv = []    
        self.use_fast = use_fast
        self.diffeo_estimator_params = diffeo_estimator_params
        
    def new_estimator(self):
        if self.use_fast:
            logger.warning('Using experimental diffeo estimator')
            return DiffeomorphismEstimatorFaster(**self.diffeo_estimator_params)
        else:
            return DiffeomorphismEstimator(**self.diffeo_estimator_params)
    
    def command_index(self, command):
        if not command in self.command_list:    
            logger.info('Adding new command %s' % str(command))
            self.command_list.append(command)
            self.estimators.append(self.new_estimator())
            self.estimators_inv.append(self.new_estimator())
            
        index = self.command_list.index(command)
        return index
    
    def merge(self, other):
        """ Merges the values obtained by "other" with ours. 
            Note that we don't make a deep copy of structures.
        """
        for i in range(len(self.command_list)):
            # Note that they are not necessarily in the right order.
            command = self.command_list[i]
            if not command in other.command_list:
                logger.info('The other does not have %s' % str(command))
                logger.info('Ours: %s' % self.command_list)
                logger.info('His:  %s' % other.command_list)
                continue
            j = other.command_list.index(command)
            
            self.estimators[i].merge(other.estimators[j])
            self.estimators_inv[i].merge(other.estimators_inv[j])
        # Now add the ones we don't have.
        for j in range(len(other.command_list)):
            command = other.command_list[j]
            if command in self.command_list:
                # Already have it
                continue
            logger.info('Adding command %s' % str(command))
            self.command_list.append(command)
            self.estimators.append(other.estimators[j])
            self.estimators_inv.append(other.estimators_inv[j])
        
    def update(self, Y0, U0, Y1):
        cmd_ind = self.command_index(U0)
        #logger.info('Updating estimator %s' % str(cmd_ind))
        for ch in range(3):
            self.estimators[cmd_ind].update(Y0[:, :, ch], Y1[:, :, ch])
            self.estimators_inv[cmd_ind].update(Y1[:, :, ch], Y0[:, :, ch])
            
                            
    def summarize(self, prefix=''):
        """
            Summarizes all estimators
            Output:
                All summarized diffeomorphisms stored in self.diffeo_list
        """
        n = len(self.estimators)
        action_list = [] 
        
        for i in range(n):
            command = np.array(self.command_list[i])
            name = prefix + str(list(command)).replace(' ', '')
            diffeo = self.estimators[i].summarize()
#            DiffeoAnalysis(self.estimators[i], name, self.estimators[i].shape,
#                           self.estimators[i].lengths).make_images()
#            pdb.set_trace()
#            self.estimators[i].summarize_continuous(prefix + str(command) + '.png')
            diffeo_inv = self.estimators_inv[i].summarize()
            name = 'Uninterpreted Diffeomorphism' + str(i)
            action = DiffeoAction(name, diffeo, diffeo_inv, command)
            action_list.append(action)
            
        name = 'Uninterpreted Diffeomorphism System'
        self.system = DiffeoSystem(name, action_list)
        return self.system
    
    def display(self, report):        
        for i in range(len(self.estimators)):
            logger.info('Report for %d-th action' % i)
            self.estimators[i].display(report.section('d%s' % i))
            self.estimators_inv[i].display(report.section('d%s-inv' % i))
            
        
    def analyze(self, prefix='', folder=''):
        """
            Make some analysis of all estimators
            Action: DiffeoAnalysis.make_images() for all estimators
        """
        n = len(self.estimators)
        
        for i in range(n):
            command = np.array(self.command_list[i])
            name = prefix + str(list(command)).replace(' ', '')
#            diffeo = self.estimators[i].summarize()
            DiffeoAnalysis(self.estimators[i], name, self.estimators[i].shape,
                           self.estimators[i].lengths, folder).make_images()
                
                
    def diffeo_dump(self, path, name):
        '''
            Save the summarized diffeomorphisms system to a pickle file
        
            :param path: output directory
            :param name: name of this system
        '''
        # TODO: remove this function, and remove local variable self.system
        ds_dump(self.system, path, name, "Learned")
        
            
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
        
    
